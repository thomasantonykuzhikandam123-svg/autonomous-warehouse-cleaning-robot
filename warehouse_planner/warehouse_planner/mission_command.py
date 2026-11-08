#!/usr/bin/env python3
"""
The boundary between the LLM and the robot.

Nothing the language model produces reaches Nav2 directly. It produces text;
this module turns that text into either a validated MissionCommand or a
rejection with a reason. There is no third outcome.

Two design rules, both deliberate:

  1. The LLM chooses from a FIXED SET OF NAMED ZONES. It can never supply
     coordinates. A hallucinated zone name is rejected by the enum; a
     hallucinated coordinate would have been executed. This single choice
     removes an entire class of failure.

  2. Coercion happens BEFORE validation, not instead of it. Small models make
     predictable mistakes -- llama3.2 in particular returns a list where a
     string belongs, which you have already hit. Those known shapes are
     repaired. Anything still wrong after repair is rejected, loudly.

No ROS and no network here on purpose: this file is testable on its own.
"""

from __future__ import annotations

import json
import re
from enum import Enum
from typing import List, Optional, Tuple

from pydantic import BaseModel, Field, ValidationError, field_validator


# --------------------------------------------------------------------------
# Vocabulary the LLM is allowed to use
# --------------------------------------------------------------------------

class Zone(str, Enum):
    AISLE_1 = "aisle_1"
    AISLE_2 = "aisle_2"
    AISLE_3 = "aisle_3"
    FRONT = "front"
    BACK = "back"
    ALL = "all"
    UNCLEANED = "uncleaned"


class Action(str, Enum):
    CLEAN = "clean"
    STOP = "stop"
    RESUME = "resume"
    SKIP = "skip"          # used by the obstacle agent: drop a zone, continue
    HOME = "home"          # return to the dock
    RESET = "reset"        # forget which zones are recorded as cleaned
    NOOP = "noop"          # "this is not a cleaning command" -- see note below


class MissionCommand(BaseModel):
    """The only object that is allowed to reach the robot.

    NOOP deserves a word. It is NOT the same as a rejection:

      rejected -> the model produced something malformed. A fault.
      noop     -> the model correctly understood the input was not a
                  cleaning command, and said so. Working as intended.

    Without NOOP every input is forced into an action that DOES something,
    and a small model will pick whichever is structurally legal -- which is
    always 'stop'. That turned "aisle 2 is blocked" into a full mission halt.
    Giving the model an honest way out is what stops that.
    """
    action: Action
    zones: List[Zone] = Field(default_factory=list)
    reason: Optional[str] = Field(default=None, max_length=200)

    @field_validator("zones")
    @classmethod
    def zones_make_sense(cls, v: List[Zone]) -> List[Zone]:
        if Zone.ALL in v and len(v) > 1:
            return [Zone.ALL]                      # "all" absorbs the rest
        seen, out = set(), []
        for z in v:                                # dedupe, keep order
            if z not in seen:
                seen.add(z)
                out.append(z)
        return out

    def model_post_init(self, __context) -> None:
        if self.action in (Action.CLEAN, Action.SKIP) and not self.zones:
            raise ValueError(f"action '{self.action.value}' requires at least one zone")
        if self.action in (Action.STOP, Action.RESUME, Action.NOOP,
                           Action.HOME, Action.RESET) and self.zones:
            raise ValueError(f"action '{self.action.value}' takes no zones")


# --------------------------------------------------------------------------
# Repairing the mistakes small models actually make
# --------------------------------------------------------------------------

_ACTION_SYNONYMS = {
    "clean": "clean", "start": "clean", "go": "clean", "begin": "clean",
    "move": "clean", "head": "clean", "proceed": "clean", "navigate": "clean",
    "drive": "clean", "reroute": "clean", "switch": "clean",
    "sweep": "clean", "vacuum": "clean", "run": "clean", "do": "clean",
    "stop": "stop", "halt": "stop", "pause": "stop", "cancel": "stop",
    "abort": "stop", "emergency_stop": "stop",
    "resume": "resume", "continue": "resume", "restart": "resume",
    "skip": "skip", "avoid": "skip", "ignore": "skip", "exclude": "skip",
    "home": "home", "dock": "home", "return": "home", "park": "home",
    "reset": "reset", "forget": "reset", "clear": "reset",
    "noop": "noop", "none": "noop", "unknown": "noop", "nothing": "noop",
    "unclear": "noop", "n_a": "noop", "no_action": "noop", "invalid": "noop",
}

_ZONE_SYNONYMS = {
    "uncleaned": "uncleaned", "unclean": "uncleaned", "remaining": "uncleaned",
    "rest": "uncleaned", "leftover": "uncleaned", "whats_left": "uncleaned",
    "not_done": "uncleaned", "unfinished": "uncleaned", "missed": "uncleaned",
    "all": "all", "everything": "all", "whole_warehouse": "all",
    "entire_warehouse": "all", "warehouse": "all", "full": "all",
    "everywhere": "all", "complete": "all", "whole_floor": "all",
    "front": "front", "front_area": "front", "front_zone": "front",
    "entrance": "front", "loading_area": "front", "packing_area": "front",
    "back": "back", "back_area": "back", "back_zone": "back",
    "rear": "back", "rear_area": "back",
}


def _normalise_token(tok: str) -> str:
    """'Aisle 3' / 'aisle-3' / 'AISLE3' / 'the third aisle' -> 'aisle_3'."""
    t = str(tok).strip().lower()
    # Word numbers FIRST, while separators are still spaces. Doing this after
    # underscoring silently fails: '_' is a word character, so \bthree\b can
    # never match inside 'aisle_three'.
    words = {"one": "1", "two": "2", "three": "3",
             "first": "1", "second": "2", "third": "3"}
    for w, d in words.items():
        t = re.sub(rf"\b{w}\b", d, t)
    t = re.sub(r"[^a-z0-9]+", "_", t).strip("_")
    t = re.sub(r"^(the|a|an)_", "", t)
    # int() strips leading zeros: "aisle_01" must become "aisle_1", not
    # "aisle_01". It did not, so the enum rejected it, parse_direct failed,
    # and inside a chain that silently truncated the whole instruction.
    m = re.search(r"(?:aisle|isle|aile|row|lane)[_\- ]?0*(\d+)", t)
    if m:
        return f"aisle_{int(m.group(1))}"
    m = re.search(r"^0*(\d+)[_\- ]?(?:aisle|isle)$", t)
    if m:
        return f"aisle_{int(m.group(1))}"
    return t


def _first_json_object(raw: str) -> Optional[str]:
    """Pull the first balanced {...} out of whatever the model wrapped it in:
    markdown fences, a chatty preamble, trailing commentary."""
    s = re.sub(r"```(?:json)?", "", raw)
    start = s.find("{")
    if start < 0:
        return None
    depth, in_str, esc = 0, False, False
    for i in range(start, len(s)):
        c = s[i]
        if in_str:
            if esc:
                esc = False
            elif c == "\\":
                esc = True
            elif c == '"':
                in_str = False
            continue
        if c == '"':
            in_str = True
        elif c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return s[start:i + 1]
    return None


def _coerce(obj: dict) -> Tuple[dict, List[str]]:
    """Repair the known-bad shapes. Returns (cleaned, notes)."""
    notes: List[str] = []
    out: dict = {}

    low = {str(k).strip().lower(): v for k, v in obj.items()}

    # --- action ---
    act = low.get("action", low.get("command", low.get("intent")))
    if isinstance(act, list):                       # llama3.2's favourite bug
        notes.append("action arrived as a list; took the first element")
        act = act[0] if act else None
    if act is not None:
        key = _normalise_token(act)
        if key in _ACTION_SYNONYMS:
            if _ACTION_SYNONYMS[key] != key:
                notes.append(f"action '{act}' mapped to '{_ACTION_SYNONYMS[key]}'")
            out["action"] = _ACTION_SYNONYMS[key]
        else:
            out["action"] = key                     # let the enum reject it

    # --- zones ---
    z = low.get("zones", low.get("zone", low.get("targets", low.get("area"))))
    if z is None:
        z = []
    if isinstance(z, str):
        notes.append("zones arrived as a string; wrapped in a list")
        z = re.split(r",| and ", z)
    if isinstance(z, dict):
        notes.append("zones arrived as a dict; took its values")
        z = list(z.values())
    if not isinstance(z, list):
        z = [z]

    cleaned, dropped = [], []
    for item in z:
        if isinstance(item, dict):                  # [{"zone": "aisle_1"}]
            item = next(iter(item.values()), "")
        key = _normalise_token(item)
        key = _ZONE_SYNONYMS.get(key, key)
        if key in {e.value for e in Zone}:
            cleaned.append(key)
        else:
            dropped.append(str(item))
    if dropped:
        notes.append(f"dropped unrecognised zone(s): {', '.join(dropped)}")
    out["zones"] = cleaned

    # --- reason ---
    r = low.get("reason", low.get("explanation", low.get("why")))
    if isinstance(r, list):
        r = " ".join(str(x) for x in r)
    if r is not None:
        out["reason"] = str(r)[:200]

    return out, notes


_FILLER = re.compile(
    r"^\s*(please|now|ok|okay|so|then|can you|could you|would you|will you|"
    r"i want you to|i need you to|go ahead and|just)\b[\s,]*", re.I)


def strip_fillers(text: str) -> str:
    t = text or ""
    for _ in range(4):
        t2 = _FILLER.sub("", t)
        if t2 == t:
            break
        t = t2
    return t.strip()


_VERB_RE = re.compile(
    r"^\s*(?P<verb>clean|sweep|vacuum|start|do|stop|halt|pause|cancel|abort|"
    r"resume|continue|skip|avoid|ignore|home|dock|return|park|reset|forget|"
    r"move|head|proceed|navigate|drive|reroute|switch)\b"
    r"(?P<rest>.*)$", re.I)
_BLOCKED_RE = re.compile(
    r"^(?P<what>.+?)\s+(?:is|are|has been|was|seems|looks)?\s*"
    r"(?:blocked|obstructed|occupied|unavailable|closed)\b", re.I)


_AISLE_REF = re.compile(
    r"(?:aisle|isle|aile|row|lane)[\s_\-]*0*(\d+)"
    r"|\b0*(\d+)[\s_\-]*(?:aisle|isle)\b"
    r"|\b(first|second|third)\s+(?:aisle|isle)\b", re.I)
_WORD = re.compile(r"[A-Za-z][A-Za-z_]*", re.I)
_ORDINAL = {"first": "1", "second": "2", "third": "3"}


def _zones_in(text: str) -> List[str]:
    """Zone names in a piece of free text, IN ORDER OF APPEARANCE.

    Done as one position-ordered pass so "clean the back then aisle 1" keeps
    that order. The previous version used a non-greedy token regex, which
    matched "aisle_01" as "aisle_0" -- it grabbed only the first digit, the
    enum rejected it, and the whole instruction fell apart from there.
    """
    t = text or ""
    hits = []                                   # (position, zone)

    for m in _AISLE_REF.finditer(t):
        n = m.group(1) or m.group(2)
        if n is None:
            n = _ORDINAL[m.group(3).lower()]
        hits.append((m.start(), f"aisle_{int(n)}"))

    consumed = [(m.start(), m.end()) for m in _AISLE_REF.finditer(t)]
    for m in _WORD.finditer(t):
        if any(a <= m.start() < b for a, b in consumed):
            continue                            # part of an aisle reference
        key = _normalise_token(m.group(0))
        key = _ZONE_SYNONYMS.get(key, key)
        if key in {e.value for e in Zone} and not key.startswith("aisle_"):
            hits.append((m.start(), key))

    out = []
    for _, z in sorted(hits, key=lambda h: h[0]):
        if z not in out:
            out.append(z)
    return out





_CHAIN = re.compile(
    r"\s*,\s*(?:then\s+|and\s+then\s+|and\s+|next\s+)?"      # ", then ..."
    r"|\s+(?:and\s+then|after\s+that|then|next|and)\s+",        # " and ..."
    re.I)


def has_chain(text: str) -> bool:
    """Does this instruction contain more than one clause?

    If it does and parse_chain cannot handle all of it, the caller must NOT
    fall back to parse_direct -- that matches only the first clause and
    silently discards the rest. "stop and clean aisle_01" became a bare stop
    that way.
    """
    t = strip_fillers(text or "")
    if not t or _EXCEPT.search(t):
        return False
    return len([p for p in _CHAIN.split(t) if p.strip()]) > 1


def parse_chain(text: str) -> Optional[List[MissionCommand]]:
    """Split "stop and clean aisle 1" / "clean aisle 3 then aisle 1 then back"
    into an ordered list of commands.

    A single MissionCommand cannot express two different actions, which is why
    "stop and clean aisle 1" was rejected outright. Chains are a list.

    Bare zone fragments inherit the previous verb, so "clean aisle 3 then
    aisle 1" means clean both, in that order -- not clean one and guess at the
    other.
    """
    t = strip_fillers(text)
    if not t:
        return None
    if _EXCEPT.search(t):
        return None      # "except A and B" owns its own 'and'; splitting
                         # the chain there tore the exclusion list in half
                         # and let an excluded zone back into the mission
    parts = [p.strip() for p in _CHAIN.split(t) if p.strip()]
    if len(parts) < 2:
        return None

    out: List[MissionCommand] = []
    last_verb: Optional[str] = None
    for part in parts:
        c = parse_direct(part)
        if c is not None:
            out.append(c)
            last_verb = c.action.value
            continue
        z = _zones_in(part)               # bare fragment, e.g. "aisle 1"
        if z and last_verb in ("clean", "skip"):
            try:
                out.append(MissionCommand(action=Action(last_verb), zones=z,
                                          reason="chained"))
                continue
            except Exception:
                pass
        return None                        # something in the chain is unclear

    # merge consecutive cleans so the ORDER the user asked for is preserved
    merged: List[MissionCommand] = []
    for c in out:
        if (merged and c.action == Action.CLEAN
                and merged[-1].action == Action.CLEAN):
            zs = [z for z in merged[-1].zones]
            for z in c.zones:
                if z not in zs:
                    zs.append(z)
            merged[-1] = MissionCommand(action=Action.CLEAN, zones=zs,
                                        reason="chained")
        else:
            merged.append(c)
    return merged or None


_EXCEPT = re.compile(r"\b(?:except|excluding|apart from|other than|but not|"
                     r"except for|besides)\b", re.I)

# Qualifiers this parser cannot evaluate. Their presence means the instruction
# says more than the zone names do, so guessing from the zone names alone is
# how "clean everything except aisle 2" became "clean everything".
_UNHANDLED = re.compile(
    r"\b(nearest|closest|furthest|farthest|dirtiest|cleanest|quickest|fastest|"
    r"shortest|longest|biggest|largest|smallest|worst|best|only|just the|"
    r"whichever|whatever is|if |unless|before |after |while |"
    r"from where|where you (?:left|stopped)|pick up|carry on from|"
    r"continue from|half.?way|part.?way)\b", re.I)


def parse_direct(text: str) -> Optional[MissionCommand]:
    """Deterministic parse for plainly-worded instructions, tried BEFORE the model.

    This exists because of a real failure: "clean aisle_2" came back as
    skip aisle_2. The few-shot prompt pairs the literal token "aisle_2" with
    the skip example, and a small model will follow that surface pattern over
    the actual verb. Zone names must not be bound to actions by example.

    Same principle as keyword-first routing: where the wording is
    unambiguous, do not ask a probabilistic system.
    """
    t = strip_fillers(text)
    if not t:
        return None

    m = _BLOCKED_RE.match(t)          # "aisle 2 is blocked"
    if m:
        z = _zones_in(m.group("what"))
        if z:
            try:
                return MissionCommand(action=Action.SKIP, zones=z,
                                      reason="reported blocked")
            except Exception:
                return None

    if re.search(r"\b(home|dock)\b", t, re.I) and not _zones_in(t):
        try:
            return MissionCommand(action=Action.HOME, zones=[], reason="direct")
        except Exception:
            return None

    m = _VERB_RE.match(t)
    if not m:
        return None
    verb = _ACTION_SYNONYMS.get(_normalise_token(m.group("verb")))
    rest = m.group("rest")
    if verb is None:
        return None
    try:
        if verb in ("stop", "resume", "home", "reset"):
            if _zones_in(rest):       # "stop cleaning aisle 2" is not a plain stop
                return None
            return MissionCommand(action=Action(verb), zones=[], reason="direct")
        # "clean A except B": subtract, do not ignore the word.
        if _UNHANDLED.search(rest):
            return None    # says more than the zone names do -> model decides

        mx = _EXCEPT.search(rest)
        if mx:
            keep = _zones_in(rest[:mx.start()])
            drop = set(_zones_in(rest[mx.end():]))
            if not keep or not drop:
                return None                      # unclear -> model
            if "all" in keep or "uncleaned" in keep:
                keep = ["front", "aisle_1", "aisle_2", "aisle_3", "back"]
            zones = [z for z in keep if z not in drop]
            if not zones:
                return None
            return MissionCommand(action=Action(verb), zones=zones,
                                  reason=f"all except {', '.join(sorted(drop))}")

        zones = _zones_in(rest)
        if not zones:
            return None               # "clean" with no zone -> let the model try
        return MissionCommand(action=Action(verb), zones=zones, reason="direct")
    except Exception:
        return None


def parse_llm_output(raw: str) -> Tuple[Optional[MissionCommand], List[str]]:
    """text -> (command, notes) or (None, [reasons it was rejected]).

    A None return is a SAFE outcome: the caller does nothing and says why.
    """
    if not raw or not raw.strip():
        return None, ["model returned nothing"]

    blob = _first_json_object(raw)
    if blob is None:
        return None, ["no JSON object found in model output"]

    try:
        obj = json.loads(blob)
    except json.JSONDecodeError as e:
        return None, [f"JSON did not parse: {e}"]

    if not isinstance(obj, dict):
        return None, ["top-level JSON was not an object"]

    cleaned, notes = _coerce(obj)

    try:
        cmd = MissionCommand(**cleaned)
    except (ValidationError, ValueError) as e:
        first = str(e).splitlines()
        detail = next((l.strip() for l in first[1:] if l.strip()), str(e))
        return None, notes + [f"rejected: {detail}"]

    return cmd, notes


# --------------------------------------------------------------------------
# The prompt. Short on purpose: long prompts make small models drift.
# --------------------------------------------------------------------------

SYSTEM_PROMPT = """You convert warehouse cleaning requests into JSON.

Reply with ONE JSON object and nothing else. No markdown, no explanation.

Schema:
  action: one of "clean", "stop", "resume", "skip", "noop"
  zones:  a LIST of strings from: "aisle_1", "aisle_2", "aisle_3", "front", "back", "all"
  reason: a short string

Rules:
  - zones is ALWAYS a list, even for one zone.
  - "clean" and "skip" need at least one zone.
  - "stop", "resume" and "noop" take an empty zones list.
  - If a zone is reported blocked or obstructed, use "skip" with that zone.
  - Use "stop" ONLY when asked to halt the robot.
  - If the request is not about cleaning this warehouse, use "noop".
  - The VERB decides the action. A zone name never implies an action.
  - Never invent zone names. Never output coordinates.

Examples:
user: clean aisle 3
{"action": "clean", "zones": ["aisle_3"], "reason": "asked for aisle 3"}

user: clean aisle_2
{"action": "clean", "zones": ["aisle_2"], "reason": "asked for aisle 2"}

user: aisle 1 is blocked
{"action": "skip", "zones": ["aisle_1"], "reason": "aisle 1 reported blocked"}

user: stop the robot
{"action": "stop", "zones": [], "reason": "asked to halt"}

user: what is the weather
{"action": "noop", "zones": [], "reason": "not a cleaning request"}
"""