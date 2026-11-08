#!/usr/bin/env python3
"""
robot_agent — a LangGraph agent that drives the cleaning robot.

v10.4 (11 Oct): the FACTS sheet also says what the safety layer is doing
(/mission_state's traffic fields from coverage_nav2 v10.4, which reads the
people guard), and "why did you stop / what are you waiting for" is answered
from that state by code (explain_stop), never by the model.

What this adds over llm_commander, which sent every input down one prompt:

  ROUTING      a question ("how much is left?") is answered, not forced into
               a command. Keyword match first, LLM only for the ambiguous
               remainder -- because small models ignore prompt formatting
               often enough that pure-LLM routing is not dependable.

  MEMORY       the last few turns travel with the request, so "aisle 2 is
               blocked" followed by "skip it" resolves. That pair failed
               before, and it is the whole reason memory is here.

  STATE        /mission_state is fed into the agent, so it can answer from
               what the robot is actually doing rather than guessing.

What it deliberately does NOT do: LLM tool-calling. llama3.2 is unreliable at
it -- it hands lists where strings belong, which you have hit already. At this
model size, injecting the facts into the prompt beats asking the model to go
and fetch them. The concept is used; the fragile mechanism is not.

The safety boundary is unchanged. Every command still goes through
mission_command.parse_llm_output, and coverage_nav2 still re-validates on
receipt. The graph decides WHAT to ask for; it never gets to skip the checks.

    START -> classify -> (command) -> compose_command -> END
                      \\ (question) -> answer_question  -> END
"""

from __future__ import annotations

import json
import math
import re
from typing import Callable, List, Optional, Tuple, TypedDict

from langgraph.graph import END, START, StateGraph

from warehouse_planner.mission_command import (SYSTEM_PROMPT, MissionCommand,
                                                _ZONE_SYNONYMS, _coerce,
                                                _first_json_object,
                                                _normalise_token, _zones_in,
                                                has_chain, parse_chain,
                                                parse_direct, parse_llm_output,
                                                strip_fillers)


# --------------------------------------------------------------------------
# graph state
# --------------------------------------------------------------------------

class AgentState(TypedDict, total=False):
    text: str                       # what the human just said
    history: List[Tuple[str, str]]  # [(role, content)] recent turns
    robot: dict                     # latest /mission_state
    route: str                      # 'command' | 'question'
    route_why: str                  # how the route was decided
    command: Optional[dict]         # validated, ready for the wire
    reply: str                      # what to show the human
    notes: List[str]                # coercion / rejection notes
    handled: bool                   # did the fast path take it


# --------------------------------------------------------------------------
# routing — keywords first, model second
# --------------------------------------------------------------------------

_COMMAND_HINTS = (
    "clean", "sweep", "vacuum", "start", "go", "begin", "do ",
    "stop", "halt", "pause", "cancel", "abort",
    "resume", "continue", "carry on",
    "skip", "avoid", "ignore", "blocked", "obstructed", "blocking",
)
_QUESTION_HINTS = (
    "what", "which", "how", "where", "why", "when", "who",
    "status", "progress", "left", "remaining", "done", "finished",
    "are you", "are we", "is it", "have you", "did you", "tell me",
)


_IMPERATIVE = re.compile(
    r"^(clean|sweep|vacuum|start|begin|do|go|run|stop|halt|pause|cancel|abort|"
    r"resume|continue|skip|avoid|ignore)\b", re.I)
_BLOCKED_STMT = re.compile(
    r"\b(is|are|has been|was)\s+(blocked|obstructed|occupied|closed)\b", re.I)


def classify_keywords(text: str) -> Optional[str]:
    """'command' / 'question' when the wording is unambiguous, else None.

    A command must BEGIN with a verb. Matching a command word anywhere in the
    sentence is what made "so aisle 1 or aisle 2 you clean first" -- plainly a
    question -- route as a command and drive the robot off. Asking something
    must never move a machine.

    Checked before the model because routing is the one decision where being
    wrong sends the entire turn down the wrong branch.
    """
    raw = (text or "").strip()
    if not raw:
        return None
    t = strip_fillers(raw)

    # interrogatives win outright, even if a command verb appears later
    if raw.endswith("?"):
        return "question"
    if re.match(r"^(what|which|how|why|when|who|where|is|are|"
                r"have|has|can|could|should|will|would)\b", t, re.I):
        return "question"
    # "do/does/did" is an auxiliary only when a subject follows it. "do you
    # clean aisle 3" asks; "do aisle 3" tells. Treating every leading "do" as a
    # question sent "please do aisle 4 then aisle 2" down the question path.
    if re.match(r"^(do|does|did)\s+(you|we|i|they|it|he|she|the robot)\b", t, re.I):
        return "question"
    if re.search(r"\byou\s+(clean|do|go|sweep|start|finish|prefer|think)\b",
                 t, re.I):
        return "question"          # "...you clean first" is asking, not telling

    # Ask the PARSER whether this is a command, rather than keeping a second
    # verb list here. The two lists drifted: the parser learned "move to
    # aisle 1" and "aisle 2 blocked" while the router did not, so real
    # instructions were answered as questions instead of being executed.
    if parse_direct(t) is not None or parse_chain(t):
        return "command"
    if _IMPERATIVE.match(t) or _BLOCKED_STMT.search(t):
        return "command"

    if any(h in " " + t.lower() + " " for h in _QUESTION_HINTS):
        return "question"
    return None                    # genuinely ambiguous -> ask the model


ROUTER_PROMPT = """Classify the message as exactly one word.

"command"  - it asks the robot to do something, or reports a blocked area.
"question" - it asks about status, progress, or anything else.

Reply with the single word and nothing else."""


# --------------------------------------------------------------------------
# when does a turn actually need the previous turns?
# --------------------------------------------------------------------------

_ANAPHORA = re.compile(
    r"\b(it|that|this|there|them|those|these|again|same|as well|too|"
    r"the other|another one)\b", re.I)


def needs_context(text: str) -> bool:
    """True only if the message cannot stand alone.

    History is NOT attached to every turn. Attaching it unconditionally let an
    earlier "aisle 2 is blocked / skip it" bleed into a later bare "stop", and
    the model answered with skip. On a small model the most recent strong
    pattern in the prompt wins, so context is supplied only where a reference
    genuinely needs resolving -- "skip it" yes, "stop" no.
    """
    t = text.strip()
    if _ANAPHORA.search(t):
        return True
    return len(t.split()) <= 2 and not re.search(
        r"\b(stop|halt|pause|resume|continue|clean|sweep|start|go)\b", t, re.I)


# --------------------------------------------------------------------------
# describing the robot to the model
# --------------------------------------------------------------------------

ZONE_NAMES = {
    "front": "the front area",
    "aisle_1": "aisle 1", "aisle_2": "aisle 2", "aisle_3": "aisle 3",
    "back": "the back area", "home": "returning to the dock",
    "start": "starting out", "done": "finished",
}


ZONE_CENTRE = {                    # map frame, for distance questions
    "front":   (3.50, -1.10),
    "aisle_1": (-0.49, 7.50),
    "aisle_2": (3.51, 7.50),
    "aisle_3": (7.53, 7.50),
    "back":    (3.50, 14.70),
}
ALL_ZONES = ["front", "aisle_1", "aisle_2", "aisle_3", "back"]

ZONE_BOX = {                       # map frame (x0, x1, y0, y1): each zone's floor
    "front":   (-3.60, 10.60, -3.20,  1.00),
    "aisle_1": (-1.94,  0.96,  0.80, 14.40),   # 0.80/14.40, not 1.10/13.90:
    "aisle_2": (2.06,   4.96,  0.80, 14.40),   #   the sweep turns past the
    "aisle_3": (6.06,   9.01,  0.80, 14.40),   #   lane ends, out to y~14.1
    "back":    (-3.60, 10.60, 14.00, 15.30),
}


def _zone_gap(z, x, y, here=None):
    """Distance from (x, y) to zone z. ZERO when the robot is standing in it.

    `here` is robot["zone"] from /mission_state -- the robot's own answer to
    "which zone am I working in", which beats re-deriving it from coordinates.
    The box is only the fallback: with boxes alone, the lane-end turn at
    y=14.04 fell outside aisle_1 (1.10-13.90) and the reply said "aisle 1
    (6.5 m)" while the robot was in aisle 1.

    Outside a zone the old centre metric is kept on purpose: box distance
    would call the next aisle 1 m away when it is a 13 m drive around the
    rack. Travel distance comes with the roadmap.
    """
    if here and z == here:
        return 0.0
    b = ZONE_BOX.get(z)
    if b is not None and b[0] <= x <= b[1] and b[2] <= y <= b[3]:
        return 0.0
    cx, cy = ZONE_CENTRE[z]
    return math.hypot(cx - x, cy - y)


# "carry on" while paused is a resume, not a new mission. Kept deliberately
# narrow: a named zone, "again"/"over", or a robot that is not paused all fall
# through to the planner.
_CONTINUE = re.compile(
    r"^(carry on|continue|keep going|keep cleaning|keep at it|go on|go ahead|"
    r"proceed|resume|get going|start cleaning|start|finish it|finish up)\b", re.I)
_RESTART = re.compile(r"\b(over|from the start|from scratch|again)\b", re.I)
_WHAT_DONE = re.compile(
    r"^(what|how much|how many)\b.*\b(clean|cleaned|done|finished|swept|covered)\b",
    re.I)
_ABOUT_LEFT = re.compile(r"\b(left|remaining|still|to go|next|yet to)\b", re.I)


def describe(robot: dict) -> str:
    """A flat, unambiguous fact sheet.

    The old version said things like "On waypoint 2 of 13", and the model read
    those numbers as aisle numbers -- it replied "aisles 2-12 are empty, aisle
    13 is the current location". There are five zones and there always will be,
    so the sheet states them explicitly and never puts a bare count next to the
    word aisle.
    """
    if not robot:
        return "The robot has not reported anything yet."

    cleaned = [z for z in (robot.get("cleaned") or []) if z in ALL_ZONES]
    todo = [z for z in ALL_ZONES if z not in cleaned]
    name = lambda z: ZONE_NAMES.get(z, z)

    L = ["The warehouse has exactly five zones: the front area, aisle 1, "
         "aisle 2, aisle 3, and the back area. There is no other zone.",
         f"Zones finished end to end: "
         f"{', '.join(name(z) for z in cleaned) if cleaned else 'none yet -- but read the part-cleaned line before saying nothing has been done'}.",
         f"Still to clean: {', '.join(name(z) for z in todo) if todo else 'nothing, all done'}."]

    part = {z: v for z, v in (robot.get("partial") or {}).items()
            if z in ALL_ZONES and z not in cleaned}
    lanes = robot.get("swept") or {}      # v10.5: an aisle's % is its lanes swept
    if part:
        L.append("Partly cleaned but not finished: "
                 + "; ".join(f"{name(z)} about {v} percent "
                             + ("of its lanes swept" if z in lanes else "of its waypoints")
                             for z, v in part.items()) + ".")

    queue = [z for z in (robot.get("queue") or []) if z in ALL_ZONES]
    if queue:
        L.append(f"The job it was given is, in order: "
                 f"{', '.join(name(z) for z in queue)}.")
        if len(queue) > 1:
            L.append(f"So after the current one it goes to {name(queue[1])}.")
        else:
            L.append("Nothing is queued after the current one.")
    else:
        L.append("It has no zones queued.")

    zone = robot.get("zone")
    if zone in ALL_ZONES:
        L.append(f"Right now it is working in {name(zone)}.")
    elif zone == "home":
        L.append("Right now it is on its way back to the dock, not cleaning "
                 "any zone.")
    elif zone == "start" or (zone is None and not robot.get("done")):
        L.append("Right now it is idle at the dock, waiting for an instruction.")
    elif robot.get("done"):
        L.append("Right now it is idle; it finished its last job.")
    else:
        L.append("Right now it is between zones, not cleaning anything.")

    x, y = robot.get("x"), robot.get("y")
    if x is not None and y is not None:
        L.append(f"Robot position: x={x}, y={y} in map coordinates.")
        d = sorted(((_zone_gap(z, x, y, robot.get("zone")), z)
                    for z in ZONE_CENTRE), key=lambda t: t[0])
        L.append("Distance from the robot, where 0 m means it is standing in "
                 "that zone: "
                 + "; ".join(f"{name(z)} {dist:.1f} m" for dist, z in d) + ".")
        near = [(dist, z) for dist, z in d if z in todo]
        if near:
            L.append(f"The nearest zone that still needs cleaning is "
                     f"{name(near[0][1])}, about {near[0][0]:.1f} m away.")

    pct = robot.get("coverage_pct")
    if pct is not None:
        L.append(f"Floor cleaned overall: {pct} percent of reachable floor.")
    if robot.get("paused"):
        L.append("The robot is PAUSED and will not move until told to resume.")
    if robot.get("rescuing"):
        L.append("It is recovering from a blocked path.")
    L.extend(traffic_facts(robot))
    L.extend(lane_facts(robot))
    blocked = [z for z in (robot.get("blocked") or []) if z in ALL_ZONES]
    L.append("Reported blocked: " + (", ".join(name(z) for z in blocked)
                                     if blocked else "nothing") + ".")
    asked = robot.get("asked") or []
    if asked:
        L.append("What the operator has asked for this session, oldest first: "
                 + "; ".join(f"({i}) {a}" for i, a in enumerate(asked, 1)) + ".")
    L.append("What it can be told to do: clean one or more zones in a given "
             "order, skip a zone, stop, resume, return to the dock, clean "
             "everything, clean only what is still uncleaned, or clean "
             "everything except named zones.")
    L.append("What it does NOT track: dirt levels, time estimates, battery, "
             "anything outside these five zones.")
    return " ".join(L)


def traffic_facts(robot: dict) -> List[str]:
    """v10.4: what the safety layer (the people guard) is doing, from
    /mission_state's traffic fields. Plain sentences for the FACTS sheet."""
    L = []
    if not robot or "guard" not in robot:
        return L                      # an older coverage_nav2: say nothing
    tr = robot.get("traffic")
    if not tr:
        L.append("Its safety layer (the people guard) is not reporting right now.")
    elif tr.get("state") == "wait":
        held = tr.get("for_s") or 0
        L.append(f"Right now its safety layer is holding it: {tr.get('say')}"
                 + (f", for {held:.0f} seconds so far" if held >= 1 else "") + ".")
    elif tr.get("v_limit") is not None:
        L.append(f"Right now it is moving carefully: {tr.get('say')}.")
    else:
        L.append("Right now nothing around it is slowing it down.")
    if tr:
        if tr.get("person_m") is not None:
            L.append(f"The nearest person it can see is {tr['person_m']:.1f} m away.")
        if tr.get("vehicle_m") is not None:
            L.append(f"The nearest vehicle it can see is {tr['vehicle_m']:.1f} m away.")
    holds = robot.get("holds") or 0
    if holds:
        L.append(f"On this job it has stopped for people or vehicles {holds} "
                 f"time{'s' if holds != 1 else ''}, {robot.get('held_s') or 0:.0f} seconds in all.")
    last = robot.get("last_hold")
    if last and not (tr and tr.get("state") == "wait"):
        L.append(f"The last time it stopped, about {last.get('ago_s', 0):.0f} seconds ago, was "
                 f"for {last.get('for_s', 0):.0f} seconds: {last.get('say')}.")
    deferred = robot.get("deferred") or []
    if deferred:
        L.append(f"It has put off {len(deferred)} stretch"
                 f"{'es' if len(deferred) != 1 else ''} of floor because someone was in "
                 "the way; it will come back for them before going home.")
    if robot.get("revisiting") and not robot.get("patching"):
        L.append("Right now it is going back over floor it had to leave earlier.")
    skipped = robot.get("skipped") or []
    if skipped:
        L.append(f"It gave up on {len(skipped)} stretch"
                 f"{'es' if len(skipped) != 1 else ''} after trying twice.")
    return L


def lane_facts(robot: dict) -> List[str]:
    """v10.5: the lanes, from /mission_state: how much of each aisle's lanes is
    swept, the patch pass before going home, blockages gone round."""
    L = []
    if not robot or "swept" not in robot:
        return L                      # an older coverage_nav2: say nothing
    swept = robot.get("swept") or {}
    if swept:
        L.append("Lanes swept in each aisle of this job (an aisle counts as finished at "
                 f"{robot.get('clean_at') or 90:.0f} percent): "
                 + "; ".join(f"{ZONE_NAMES.get(z, z)} {p:.0f} percent" for z, p in swept.items())
                 + ".")
    if robot.get("patching"):
        L.append("Right now it is going back over stretches of lane it left dirty "
                 "(where it had to go round people), before going home.")
    elif robot.get("patches"):
        n = robot["patches"]
        L.append(f"Before going home it went back over {n} stretch{'es' if n != 1 else ''} "
                 "of lane it had left dirty.")
    lane = robot.get("lane") or {}
    if lane.get("going_round"):
        L.append("Right now it is going round something standing in its lane; then it "
                 "carries on along the lane.")
    n = robot.get("bypasses") or 0
    if n:
        L.append(f"On this job it has gone round {n} blockage{'s' if n != 1 else ''} standing "
                 "in a lane; what it went round is swept again before it goes home.")
    # v10.6
    if robot.get("safety_hold"):
        L.append("Right now it is holding still because its safety layer (the people guard) has "
                 "stopped reporting; it carries on when the guard is back.")
    if robot.get("nav2_down"):
        L.append("Right now it is holding still because its navigation (Nav2) is not taking goals; "
                 "it carries on when Nav2 does.")
    v = robot.get("vehicle_skips") or 0
    if v:
        L.append(f"It left {v} bit{'s' if v != 1 else ''} of lane to a vehicle working there, to "
                 "sweep before going home.")
    if (robot.get("lane") or {}).get("blocked_ahead"):
        L.append("Right now it has stopped short of something standing in its lane.")
    # v10.7
    if robot.get("awaiting_command"):
        L.append("It has not been given a job yet: it is waiting at the dock for an instruction.")
    if robot.get("manoeuvre") == "unstick":
        L.append("Right now it is freeing itself: it got stuck facing a rack or wall, so it is "
                 "backing off a little and turning to line up, then it tries again.")
    elif robot.get("manoeuvre") == "align":
        L.append("Right now it is turning on the spot to line up with a lane next to a rack "
                 "before driving it.")
    u = robot.get("unsticks") or 0
    if u:
        L.append(f"On this job it got itself unstuck from a rack or wall {u} "
                 f"time{'s' if u != 1 else ''} (backing off and turning).")
    return L


_WHY_STOP = re.compile(
    r"\bwhy\b.*(\b(stop|stopped|stopping|wait|waiting|waited|slow|slowed|slowing|"
    r"hold|holding|held|paus\w*|stuck|standing)\b|(\bnot|n't)\s+(you\s+)?moving\b)"
    r"|^(what are you waiting for|what('s| is) (stopping|holding) you)", re.I)


def explain_stop(robot: dict) -> Optional[str]:
    """v10.4: "why did you stop?" answered from state, never by the model."""
    if not robot:
        return None
    if robot.get("awaiting_command"):                      # v10.7
        return ("I haven't been given a job yet, so I'm waiting at the dock. Tell me what to "
                "clean, for example 'clean aisle 1'.")
    if robot.get("paused"):
        return "I'm paused because I was told to stop. Say resume and I'll carry on."
    if robot.get("done"):
        return "I've finished the job I was given, so I'm not moving."
    if "guard" not in robot:
        return None                    # old coverage_nav2: let the planner answer
    tr = robot.get("traffic") or {}
    first = lambda s: (s[0].upper() + s[1:]) if s else s
    if tr.get("state") == "wait":
        say = tr.get("say") or "my safety layer is holding me"
        held = tr.get("for_s") or 0
        lead = "I've " + say if say.startswith("stopped") else "I'm " + say
        out = lead + (f". That's been {held:.0f} seconds so far" if held >= 2 else "") + "."
        more = robot.get("waiting_s") or 0
        if more >= 30:
            out += (" If it goes on much longer I'll leave this bit and come back "
                    "for it before going home.")
        return out
    if robot.get("manoeuvre") == "unstick":                # v10.7
        return ("I got stuck facing a rack: my navigation couldn't drive on from there. So I'm "
                "backing off a little and turning to line up, then I'll try again.")
    if robot.get("manoeuvre") == "align":
        return ("I'm turning on the spot to line up with the lane first, so I don't swing into "
                "the rack next to it.")
    if robot.get("rescuing"):
        return "I got stuck against something, so I'm backing off and trying again."
    if robot.get("safety_hold"):
        return ("I've stopped because my safety layer, the people guard, has gone quiet. I'll hold "
                "still until it's back.")
    if robot.get("nav2_down"):
        return ("I've stopped because my navigation (Nav2) isn't taking goals. I'm holding where I "
                "am and trying again every few seconds; nothing is being skipped. If it doesn't come "
                "back, the stack needs a restart.")
    lane = robot.get("lane") or {}
    if lane.get("going_round"):
        return ("Someone was standing in my lane, so I'm going round them and then "
                "carrying on along it. I'll sweep the bit I went round before going home.")
    if lane.get("blocked_ahead"):
        gap = lane.get("clear_ahead_m")
        return ("Something is standing in my lane" + (f", {gap:.1f} m ahead" if gap is not None else "")
                + ", so I've stopped short of it. If it doesn't move soon I'll go round it and "
                "sweep that bit before going home.")
    if (lane.get("stood_s") or 0) >= 3 and lane.get("phase") == "follow":
        return (f"Something is standing in my lane, so I've been waiting "
                f"{lane['stood_s']:.0f} seconds. If it doesn't move soon I'll go round it "
                "and sweep that bit before going home.")
    if tr.get("v_limit") is not None:
        return first(f"{tr.get('say')}.")
    last = robot.get("last_hold")
    if last:
        return (f"Nothing is holding me right now. The last time I stopped, about "
                f"{last.get('ago_s', 0):.0f} seconds ago, I was {last.get('say')}, for "
                f"{last.get('for_s', 0):.0f} seconds.")
    if not tr:
        return ("My safety layer isn't reporting, so I can't tell you what stopped me. "
                "Is the guard running?")
    return "Nothing is holding me up right now, and I haven't had to stop on this job."


ANSWER_PROMPT = """You ARE the warehouse cleaning robot, talking to the person
who operates you. Speak in the first person. Be brief, natural and honest --
two or three short sentences at most.

You MAY, and should:
  - acknowledge when the person corrects you, and agree if they are right
  - say sorry, briefly, when you got something wrong or left something out
  - explain what you keep track of and what you do not
  - answer questions about this conversation itself
  - offer the obvious next step when it is genuinely useful

You MUST NOT:
  - name a zone that does not appear in the FACTS
  - state a distance, percentage or count that does not appear in the FACTS
  - claim to have cleaned anything the FACTS do not credit you with

If the person states something the FACTS contradict, correct them plainly and
give the right fact. If the FACTS simply do not cover it, say what you DO know
and what you do not keep track of -- never just refuse and stop.

FACTS:
{facts}"""


# --------------------------------------------------------------------------
# graph
# --------------------------------------------------------------------------

# --------------------------------------------------------------------------
# qualifiers resolved by CODE over robot state -- never by the model
# --------------------------------------------------------------------------
#
# "the nearest dirty aisle", "if aisle 2 is still blocked" are not language
# problems. They are a ranking and a lookup over data the robot already
# publishes. Handed to a 3B model with no state, "nearest dirty aisle" came
# back as "front" (not an aisle) and "whichever is fastest" as "all". The
# validator passed both because they are real zone names -- it can check that
# a zone EXISTS, never that it is the RIGHT one.
#
# So: the model may decide WHAT KIND of request this is. Code does the maths.

_NEAR = re.compile(r"\b(nearest|closest|nearer|closer)\b|\bnear\s+(?:to\s+)?(?:you|me|here)\b", re.I)
_FAR = re.compile(r"\b(farthest|furthest|farther|further)\b", re.I)
_DIRTY = re.compile(r"\b(dirty|dirtiest|uncleaned|unclean|remaining|"
                    r"unfinished|not done|left)\b", re.I)
_NO_DATA = re.compile(r"\b(fastest|quickest|slowest|dirtiest|messiest|"
                      r"cleanest|easiest|hardest)\b", re.I)
_IF_BLOCKED = re.compile(
    r"^\s*if\s+(?P<a>.+?)\s+(?:is|are)\s+(?:still\s+)?"
    r"(?:blocked|obstructed|closed)\s*,?\s*(?:then\s+)?"
    r"(?:do|clean|go\s+to|move\s+to)?\s*(?P<b>.+?)(?:\s+instead)?\s*$", re.I)


def _ranked(robot: dict, aisles_only: bool, dirty_only: bool):
    """Zones worth going to, nearest first, with distances."""
    x, y = robot.get("x"), robot.get("y")
    if x is None or y is None:
        return None
    cleaned = set(robot.get("cleaned") or [])
    blocked = set(robot.get("blocked") or [])
    here = robot.get("zone")
    out = []
    for z in ZONE_CENTRE:
        if aisles_only and not z.startswith("aisle_"):
            continue
        if dirty_only and z in cleaned:
            continue
        if z in blocked:
            continue
        out.append((_zone_gap(z, x, y, here), z))
    return sorted(out)


def resolve_with_state(text: str, robot: dict):
    """Returns (handled, command_or_None, reply). Pure code, no model."""
    t = strip_fillers(text or "")
    robot = robot or {}
    name = lambda z: ZONE_NAMES.get(z, z)

    # ---- v10.4: "why did you stop?" is a state lookup: the guard says why --
    if _WHY_STOP.search(text or "") and not _zones_in(t):
        why = explain_stop(robot)
        if why:
            return True, None, why

    # ---- "carry on" while paused is a RESUME, never a new mission ---------
    # Run 1: paused mid aisle 1, "ok carry on" went to the planner, which
    # planned clean [aisle_3] and said "Continuing with the back area" -- a
    # different zone from the one it named, and the queued aisle_1 was lost.
    if (robot.get("paused") and _CONTINUE.match(t) and not _zones_in(t)
            and not _RESTART.search(t)):
        zone = name(robot.get("zone") or "")
        g, tot = robot.get("goal"), robot.get("total")
        where = f" ({g} of {tot}, in {zone})" if g and tot and zone else ""
        return True, {"action": "resume", "zones": [],
                      "reason": "told to continue while paused"}, \
            f"Resuming from where I stopped{where}."

    # ---- "what have you cleaned" is a state lookup, not a language problem -
    # It answered "Nothing yet" while 3.3% of the floor was swept and aisle 1
    # was 36% done, because the model read the cleaned-zone line alone.
    if _WHAT_DONE.match(t) and not _ABOUT_LEFT.search(t):
        fin = [z for z in (robot.get("cleaned") or []) if z in ALL_ZONES]
        part = {z: p for z, p in (robot.get("partial") or {}).items()
                if z in ALL_ZONES and z not in fin}
        bits = ["Finished: " + ", ".join(name(z) for z in fin) + "."
                if fin else "Nothing finished end to end yet."]
        if part:
            bits.append("Part-way through " + ", ".join(
                f"{name(z)} at about {p} percent" for z, p in part.items()) + ".")
        pct = robot.get("coverage_pct")
        if pct is not None:
            bits.append(f"{pct} percent of the floor overall.")
        return True, None, " ".join(bits)

    # ---- conditional on the blocked set ----------------------------------
    m = _IF_BLOCKED.match(t)
    if m:
        a, b = _zones_in(m.group("a")), _zones_in(m.group("b"))
        if not a or not b:
            return False, None, ""
        blocked = set(robot.get("blocked") or [])
        hit = [z for z in a if z in blocked]
        if hit:
            return True, {"steps": [
                {"action": "skip", "zones": hit, "reason": "still blocked"},
                {"action": "clean", "zones": b, "reason": "instead of blocked zone"}]}, \
                f"{name(hit[0])} is still blocked, so skipping it and doing {name(b[0])}."
        told = ", ".join(name(z) for z in blocked) or "nothing"
        return True, None, (
            f"{name(a[0])} isn't marked blocked -- the only zone reported blocked "
            f"is {told}. Nothing changed. Say 'clean {name(b[0])}' if you still "
            f"want it.")

    # ---- things there is no data for: say so, offer what we can compute ----
    nd = _NO_DATA.search(t)
    if nd and re.match(r"^(clean|do|go|start)\b", t, re.I):
        r = _ranked(robot, "aisle" in t.lower(), True) or []
        offer = (f" The nearest one still to clean is {name(r[0][1])}, "
                 f"{r[0][0]:.1f} m away -- say 'clean the nearest dirty "
                 f"{'aisle' if 'aisle' in t.lower() else 'zone'}' for that."
                 ) if r else ""
        return True, None, (f"I don't measure which zone is {nd.group(1).lower()} "
                            f"-- I only know position, distance and what I've "
                            f"cleaned.{offer}")

    near, far = _NEAR.search(t), _FAR.search(t)
    if not (near or far):
        return False, None, ""

    # "front or back, whichever is nearer": choose among the NAMED zones.
    # Without this the candidates were every dirty zone in the building, so
    # "front or back" could come back as an aisle -- or, via the old parser,
    # as BOTH, which is what actually happened.
    named = [z for z in _zones_in(t) if z in ALL_ZONES]
    if len(named) >= 2:
        x, y = robot.get("x"), robot.get("y")
        if x is None or y is None:
            return True, None, "I don't know where I am yet, so I can't compare them."
        blk = set(robot.get("blocked") or [])
        c = sorted((_zone_gap(z, x, y, robot.get("zone")), z)
                   for z in named if z not in blk)
        if not c:
            return True, None, "Every zone you named is reported blocked."
        dist, pick = c[0] if near else c[-1]
        cmp_ = ", ".join(f"{name(z)} {d:.1f} m" for d, z in c)
        word = "nearer" if near else "farther"
        if re.match(r"^(clean|do|go|start|move|head)\b", t, re.I):
            return True, {"action": "clean", "zones": [pick],
                          "reason": f"{word} of the zones named"}, \
                f"Of those, {name(pick)} is {word} ({cmp_}). Going there."
        return True, None, f"{name(pick)} is {word}: {cmp_}."

    aisles_only = bool(re.search(r"\baisles?\b", t, re.I)) and not \
        re.search(r"\b(zone|area)s?\b", t, re.I)
    # a cleaning request for "the nearest X" means the nearest one still dirty
    dirty = bool(_DIRTY.search(t)) or bool(re.match(r"^(clean|do|go|start)\b", t, re.I))
    r = _ranked(robot, aisles_only, dirty)
    if r is None:
        return True, None, "I don't know where I am yet, so I can't tell which is nearest."
    if not r:
        what = "aisle" if aisles_only else "zone"
        return True, None, f"There's no {'uncleaned ' if dirty else ''}{what} left to go to."

    # ---- "clean everything but skip whichever is closest" ------------------
    if re.search(r"\b(except|skip|but|other than|apart from)\b", t, re.I) and \
            re.search(r"\b(everything|all|every)\b", t, re.I):
        pick = r[0][1] if near else r[-1][1]
        rest = [z for z in ALL_ZONES if z != pick
                and z not in set(robot.get("cleaned") or [])
                and z not in set(robot.get("blocked") or [])]
        if not rest:
            return True, None, "Once that's excluded there's nothing left to clean."
        return True, {"action": "clean", "zones": rest,
                      "reason": f"everything except the {'nearest' if near else 'farthest'}"}, \
            f"Cleaning everything still dirty except {name(pick)}: " \
            f"{', '.join(name(z) for z in rest)}."

    dist, pick = r[0] if near else r[-1]
    kind = (f"{'nearest' if near else 'farthest'} "
            f"{'uncleaned ' if dirty else ''}"
            f"{'aisle' if aisles_only else 'zone'}")
    where = "I'm in it already" if dist == 0.0 else f"about {dist:.1f} m away"
    if not re.match(r"^(clean|do|go|start|move|head)\b", t, re.I):
        return True, None, f"The {kind} is {name(pick)}, {where}."
    part = (robot.get("partial") or {}).get(pick)
    if not part:
        extra = ""
    elif pick.startswith("aisle_"):
        extra = (f" Carrying on from where I stopped; about {part} percent was "
                 f"already done.")
    else:
        extra = (f" Restarting it from the beginning; it was about {part} "
                 f"percent done.")
    return True, {"action": "clean", "zones": [pick],
                  "reason": f"{kind}, {dist:.1f} m"}, \
        f"The {kind} is {name(pick)} ({where}). Going there.{extra}"


PLANNER_PROMPT = """You are the planning brain of a warehouse cleaning robot. The
operator talks to you in everyday, sometimes misspelled language. You turn what
they mean into a plan the robot can run, and you answer them briefly.

Reply with ONE JSON object and nothing else:
  {"plan": [ ...steps... ], "say": "one or two sentences to the operator"}

A step is {"action": A, "zones": [Z, ...]}
  A is one of: clean, skip, stop, resume, home, reset
  Z is one of: front, aisle_1, aisle_2, aisle_3, back, all, uncleaned
  or a reference the ROBOT works out for itself -- use these, never guess:
      nearest_dirty          nearest_dirty_aisle
      farthest_dirty         farthest_dirty_aisle
      nearest_of:A,B         farthest_of:A,B      e.g. nearest_of:front,back

Rules
  - Asking, complaining or chatting -> "plan": [] and answer in "say".
    Only plan when they want the robot to DO something.
  - Read through typos: "fron" is front, "asile" is aisle, "bak" is back.
  - Never work out distances yourself. Use a nearest_/farthest_ reference.
  - If the FACTS say the robot is PAUSED and the operator says carry on,
    continue, keep going or start, with no zone named, the action is resume.
    Never turn that into a clean: it would restart a zone and drop the queue.
  - The FACTS are the current truth. They override anything said earlier in
    this conversation, INCLUDING YOUR OWN EARLIER ANSWERS. If you said
    something before that the FACTS now contradict, correct it.
  - Never claim a zone is cleaned unless the FACTS say so.
  - Cleaning a zone always restarts it from the beginning; the robot cannot
    pick up half-way through a zone. If asked to, say so and restart it.
  - If they ask for something the robot cannot do, say so plainly.

Examples
operator: clean fron and bak
{"plan":[{"action":"clean","zones":["front","back"]}],"say":"Cleaning the front, then the back."}

operator: do whichever of front or back is closer
{"plan":[{"action":"clean","zones":["nearest_of:front,back"]}],"say":"Going to whichever of those two is nearer."}

operator: stop that and do aisle 2 instead
{"plan":[{"action":"stop","zones":[]},{"action":"clean","zones":["aisle_2"]}],"say":"Stopping, then doing aisle 2."}

operator: ok carry on          (the FACTS say the robot is PAUSED)
{"plan":[{"action":"resume","zones":[]}],"say":"Picking up where I stopped."}

operator: carry on with aisle 3 from where you left off
{"plan":[{"action":"clean","zones":["aisle_3"]}],"say":"I can't resume part-way through a zone, so I'll sweep aisle 3 again from the start."}

operator: i asked for aisle 1 first didnt i
{"plan":[],"say":"Yes -- the first thing you asked for was aisle 1."}

operator: go do the nearest aisle you havent finished
{"plan":[{"action":"clean","zones":["nearest_dirty_aisle"]}],"say":"Heading to the nearest aisle I haven't finished."}

FACTS
@@FACTS@@"""


def resolve_zone_refs(zones, robot):
    """Turn the planner's references into real zones, using robot state.

    The model decides WHICH kind of choice is wanted; this does the maths.
    Returns (zones, notes). An unresolvable reference is dropped with a note,
    never guessed.
    """
    out, notes = [], []
    x, y = (robot or {}).get("x"), (robot or {}).get("y")
    cleaned = set((robot or {}).get("cleaned") or [])
    blocked = set((robot or {}).get("blocked") or [])

    def canon(tok):
        k = _normalise_token(tok)
        return _ZONE_SYNONYMS.get(k, k)

    for raw in zones or []:
        z = str(raw).strip().lower().replace(" ", "")
        m = re.match(r"^(nearest|farthest)_of:(.+)$", z)
        if m:
            cands = [c for c in (canon(t) for t in m.group(2).split(","))
                     if c in ALL_ZONES and c not in blocked]
            if x is None or not cands:
                notes.append(f"could not resolve {raw}")
                continue
            r = sorted((_zone_gap(c, x, y, (robot or {}).get("zone")), c)
                       for c in cands)
            pick = r[0][1] if m.group(1) == "nearest" else r[-1][1]
            notes.append(f"{raw} -> {pick} ({r[0][0]:.1f}..{r[-1][0]:.1f} m)")
            out.append(pick)
            continue
        m = re.match(r"^(nearest|farthest)_dirty(_aisle|_zone)?$", z)
        if m:
            r = _ranked(robot or {}, m.group(2) == "_aisle", True)
            if not r:
                notes.append(f"nothing left to resolve {raw}")
                continue
            dist, pick = r[0] if m.group(1) == "nearest" else r[-1]
            notes.append(f"{raw} -> {pick} ({dist:.1f} m)")
            out.append(pick)
            continue
        out.append(str(raw))
    return out, notes


def confirm(steps, robot):
    """What the robot says after a command, built from the VALIDATED steps.

    The model's own sentence is never used here. In run 1 it wrote "Continuing
    with the back area" over an accepted clean [aisle_3]: the plan was valid,
    so every existing check passed, and only the words were wrong. Those words
    are kept in the notes instead, which puts both versions in /dialogue and
    makes disagreement measurable.
    """
    nm = lambda z: ZONE_NAMES.get(z, z)
    part = (robot or {}).get("partial") or {}
    out = []
    for st in steps:
        a, zs = st.get("action"), st.get("zones") or []
        if a == "clean":
            bits = []
            for z in zs:
                p = part.get(z)
                if not p:
                    bits.append(nm(z))
                elif z.startswith("aisle_"):
                    bits.append(f"{nm(z)} (carrying on from where I stopped; "
                                f"about {p} percent was done)")
                else:
                    bits.append(f"{nm(z)} (restarting from the beginning; it "
                                f"was about {p} percent done)")
            out.append("cleaning " + ", then ".join(bits))
        elif a == "skip":
            out.append("skipping " + ", ".join(nm(z) for z in zs))
        elif a == "stop":
            out.append("stopping")
        elif a == "resume":
            out.append("resuming from where I stopped")
        elif a == "home":
            out.append("going back to the dock")
        elif a == "reset":
            out.append("forgetting which zones I've cleaned")
        else:
            out.append(str(a))
    s = ", then ".join(out)
    return s[0].upper() + s[1:] + "." if s else "Nothing to do."


def build_graph(llm: Callable[[str, str], str]):
    """
        START -> classify --question--> planner (may only talk)
                          --command---> fastpath --handled--> END
                          |                      +--missed--> planner
                          +--unsure---> planner (may act)
        planner -> END

    fastpath is deterministic and instant: exact commands, chains, and
    anything computable from state. The PLANNER owns everything else -- which
    is to say, every phrasing nobody wrote a rule for. That is the thing a
    language model can do and a regex cannot.
    """

    def classify(state: AgentState) -> AgentState:
        kw = classify_keywords(state["text"])
        if kw:
            return {"route": kw, "route_why": "keyword"}
        return {"route": "open", "route_why": "no rule matched -> planner decides"}

    def fastpath(state: AgentState) -> AgentState:
        robot = state.get("robot") or {}
        handled, cmd, reply = resolve_with_state(state["text"], robot)
        if handled:
            return {"command": cmd, "reply": reply, "handled": True,
                    "notes": ["computed from robot state, no model call"]}
        chain = parse_chain(state["text"])
        if chain:
            steps = [{"action": c.action.value,
                      "zones": [z.value for z in c.zones],
                      "reason": c.reason or ""} for c in chain]
            cmd = {"steps": steps} if len(steps) > 1 else steps[0]
            return {"command": cmd, "handled": True,
                    "reply": " then ".join(f"{st['action']} "
                                           f"[{', '.join(st['zones']) or '-'}]"
                                           for st in steps),
                    "notes": ["exact command, no model call"]}
        if not has_chain(state["text"]):
            d = parse_direct(state["text"])
            if d is not None:
                return {"command": {"action": d.action.value,
                                    "zones": [z.value for z in d.zones],
                                    "reason": d.reason or ""},
                        "handled": True,
                        "reply": f"{d.action.value} "
                                 f"[{', '.join(z.value for z in d.zones) or '-'}]",
                        "notes": ["exact command, no model call"]}
        return {"handled": False}

    def planner(state: AgentState) -> AgentState:
        robot = state.get("robot") or {}
        question = state.get("route") == "question"
        # anything computable is computed -- on the question route too
        handled, cmd_r, reply_r = resolve_with_state(state["text"], robot)
        if handled and (cmd_r is None or not question):
            return {"command": cmd_r, "reply": reply_r,
                    "notes": ["computed from robot state, no model call"]}
        facts = describe(robot)
        hist = state.get("history", [])[-6:]
        convo = "\n".join(f"{r}: {c}" for r, c in hist)
        user = (f"Conversation so far (may be out of date -- FACTS win):\n{convo}\n\n"
                f"=== THE OPERATOR NOW SAYS ===\n{state['text']}") if convo \
            else state["text"]
        try:
            raw = llm(PLANNER_PROMPT.replace("@@FACTS@@", facts), user)
        except Exception as e:
            return {"command": None, "reply": f"model unreachable: {e}",
                    "notes": []}

        notes = []
        blob = _first_json_object(raw)
        if blob is None:
            # it just talked -- that is speech, never an instruction
            return {"command": None, "reply": raw.strip()[:500],
                    "notes": ["planner replied in prose; nothing executed"]}
        try:
            obj = json.loads(blob)
        except Exception:
            return {"command": None, "reply": "I got confused -- say that again?",
                    "notes": ["planner JSON did not parse; nothing executed"]}

        say = str(obj.get("say") or "").strip()
        plan = obj.get("plan") or []
        if isinstance(plan, dict):
            plan = [plan]
        if not isinstance(plan, list):
            plan = []

        if question and plan:
            notes.append("it was a question, so the plan was discarded -- "
                         "asking never moves the robot")
            plan = []

        steps = []
        for st in plan:
            if not isinstance(st, dict):
                return {"command": None, "reply": say or "I couldn't plan that.",
                        "notes": notes + ["malformed step; nothing executed"]}
            zones, zn = resolve_zone_refs(st.get("zones") or [], robot)
            notes += zn
            cleaned_st, cn = _coerce({**st, "zones": zones})
            notes += cn
            try:
                cmd = MissionCommand(**cleaned_st)
            except Exception as e:
                # one bad step sinks the whole plan: half a plan is worse
                # than none, which is the lesson of "stop and clean aisle_01"
                first = str(e).splitlines()
                detail = next((l.strip() for l in first[1:] if l.strip()), str(e))
                # The model's "say" describes the plan it proposed. That plan
                # is not happening, so the model's words must not be passed on
                # -- "Doing both." while doing nothing is the worst reply this
                # robot can give.
                bad = [n for n in notes + cn if "unrecognised zone" in n]
                why = (bad[0].split(":", 1)[-1].strip() + " isn't a zone here"
                       if bad else "part of that plan was invalid")
                return {"command": None,
                        "reply": f"I didn't do that -- {why}. Nothing was sent "
                                 f"to the robot.",
                        "notes": notes + [f"step rejected: {detail}"]}
            if cmd.action.value == "noop":
                continue
            steps.append({"action": cmd.action.value,
                          "zones": [z.value for z in cmd.zones],
                          "reason": cmd.reason or "planned"})

        if not steps:
            return {"command": None, "reply": say or "OK.", "notes": notes}
        return {"command": {"steps": steps} if len(steps) > 1 else steps[0],
                "reply": confirm(steps, robot),
                "notes": notes + [f"planned {len(steps)} step(s)",
                                  f'model said: "{say}"']}

    g = StateGraph(AgentState)
    g.add_node("classify", classify)
    g.add_node("fastpath", fastpath)
    g.add_node("planner", planner)
    g.add_edge(START, "classify")
    g.add_conditional_edges(
        "classify", lambda s: s["route"],
        {"command": "fastpath", "question": "planner", "open": "planner"})
    g.add_conditional_edges(
        "fastpath", lambda s: "done" if s.get("handled") else "plan",
        {"done": END, "plan": "planner"})
    g.add_edge("planner", END)
    return g.compile()