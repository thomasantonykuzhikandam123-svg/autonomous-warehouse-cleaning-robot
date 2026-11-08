#!/usr/bin/env python3
"""Test the NEW robot_agent logic against the exact turns of baseline run 1.

strip_fillers / _zones_in are copied verbatim from mission_command.py so the
behaviour matches; everything else is the patched code.
"""
import math
import re

# ---------------------------------------------------------------- verbatim --
_FILLER = re.compile(
    r"^\s*(please|now|ok|okay|so|then|can you|could you|would you|will you|"
    r"i want you to|i need you to|go ahead and|just)\b[\s,]*", re.I)


def strip_fillers(text):
    t = text or ""
    for _ in range(4):
        t2 = _FILLER.sub("", t)
        if t2 == t:
            break
        t = t2
    return t.strip()


_AISLE_REF = re.compile(
    r"(?:aisle|isle|aile|row|lane)[\s_\-]*0*(\d+)"
    r"|\b0*(\d+)[\s_\-]*(?:aisle|isle)\b"
    r"|\b(first|second|third)\s+(?:aisle|isle)\b", re.I)
_WORD = re.compile(r"[A-Za-z][A-Za-z_]*", re.I)
_ORDINAL = {"first": "1", "second": "2", "third": "3"}
_ZONE_SYNONYMS = {"front": "front", "back": "back", "all": "all",
                  "everything": "all", "uncleaned": "uncleaned"}
_ZONE_VALUES = {"aisle_1", "aisle_2", "aisle_3", "front", "back", "all", "uncleaned"}


def _zones_in(text):
    t = text or ""
    hits = []
    for m in _AISLE_REF.finditer(t):
        n = m.group(1) or m.group(2)
        if n is None:
            n = _ORDINAL[m.group(3).lower()]
        hits.append((m.start(), f"aisle_{int(n)}"))
    consumed = [(m.start(), m.end()) for m in _AISLE_REF.finditer(t)]
    for m in _WORD.finditer(t):
        if any(a <= m.start() < b for a, b in consumed):
            continue
        key = re.sub(r"[^a-z0-9]+", "_", m.group(0).lower()).strip("_")
        key = _ZONE_SYNONYMS.get(key, key)
        if key in _ZONE_VALUES and not key.startswith("aisle_"):
            hits.append((m.start(), key))
    out = []
    for _, z in sorted(hits, key=lambda h: h[0]):
        if z not in out:
            out.append(z)
    return out


ZONE_NAMES = {"front": "the front area", "aisle_1": "aisle 1", "aisle_2": "aisle 2",
              "aisle_3": "aisle 3", "back": "the back area", "home": "the dock",
              "start": "the dock", "done": "nothing"}
ZONE_CENTRE = {"front": (3.50, -1.10), "aisle_1": (-0.49, 7.50),
               "aisle_2": (3.51, 7.50), "aisle_3": (7.53, 7.50),
               "back": (3.50, 14.70)}
ALL_ZONES = ["front", "aisle_1", "aisle_2", "aisle_3", "back"]

# ------------------------------------------------------------------- new ----
ZONE_BOX = {                          # map frame (x0, x1, y0, y1)
    "front":   (-3.60, 10.60, -3.20,  1.00),
    "aisle_1": (-1.94,  0.96,  0.80, 14.40),
    "aisle_2": (2.06,   4.96,  0.80, 14.40),
    "aisle_3": (6.06,   9.01,  0.80, 14.40),
    "back":    (-3.60, 10.60, 14.00, 15.30),
}


def _zone_gap(z, x, y, here=None):
    """0 when the robot is in the zone -- by its own report first, box second."""
    if here and z == here:
        return 0.0
    b = ZONE_BOX.get(z)
    if b is not None and b[0] <= x <= b[1] and b[2] <= y <= b[3]:
        return 0.0
    cx, cy = ZONE_CENTRE[z]
    return math.hypot(cx - x, cy - y)


def _ranked(robot, aisles_only, dirty_only):
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


_CONTINUE = re.compile(
    r"^(carry on|continue|keep going|keep cleaning|keep at it|go on|go ahead|"
    r"proceed|resume|get going|start cleaning|start|finish it|finish up)\b", re.I)
_RESTART = re.compile(r"\b(over|from the start|from scratch|again)\b", re.I)
_WHAT_DONE = re.compile(
    r"^(what|how much|how many)\b.*\b(clean|cleaned|done|finished|swept|covered)\b", re.I)
_ABOUT_LEFT = re.compile(r"\b(left|remaining|still|to go|next|yet to)\b", re.I)


def new_head(text, robot):
    """The two new blocks that go at the TOP of resolve_with_state."""
    t = strip_fillers(text or "")
    robot = robot or {}
    name = lambda z: ZONE_NAMES.get(z, z)

    if robot.get("paused") and _CONTINUE.match(t) and not _zones_in(t) \
            and not _RESTART.search(t):
        zone = name(robot.get("zone") or "")
        g, tot = robot.get("goal"), robot.get("total")
        where = f" ({g} of {tot}, in {zone})" if g and tot and zone else ""
        return True, {"action": "resume", "zones": [],
                      "reason": "told to continue while paused"}, \
            f"Resuming from where I stopped{where}."

    if _WHAT_DONE.match(t) and not _ABOUT_LEFT.search(t):
        cleaned = [z for z in (robot.get("cleaned") or []) if z in ALL_ZONES]
        part = {z: p for z, p in (robot.get("partial") or {}).items()
                if z in ALL_ZONES and z not in cleaned}
        bits = ["Finished: " + ", ".join(name(z) for z in cleaned) + "."
                if cleaned else "Nothing finished yet."]
        if part:
            bits.append("Part-way through " + ", ".join(
                f"{name(z)} at about {p} percent" for z, p in part.items()) + ".")
        pct = robot.get("coverage_pct")
        if pct is not None:
            bits.append(f"{pct} percent of the floor overall.")
        return True, None, " ".join(bits)
    return False, None, ""


def nearest_reply(r, near, dirty, aisles_only, robot, is_order):
    """The patched tail of resolve_with_state, for testing the wording."""
    name = lambda z: ZONE_NAMES.get(z, z)
    dist, pick = r[0] if near else r[-1]
    kind = (f"{'nearest' if near else 'farthest'} "
            f"{'uncleaned ' if dirty else ''}"
            f"{'aisle' if aisles_only else 'zone'}")
    where = "I'm in it already" if dist == 0.0 else f"about {dist:.1f} m away"
    if not is_order:
        return None, f"The {kind} is {name(pick)}, {where}."
    part = (robot.get("partial") or {}).get(pick)
    extra = (f" Restarting it from the beginning; it was about {part} percent "
             f"done." if part else "")
    return {"action": "clean", "zones": [pick], "reason": f"{kind}, {dist:.1f} m"}, \
        f"The {kind} is {name(pick)} ({where}). Going there.{extra}"


def confirm(steps, robot):
    """What the robot says after a command: built from the VALIDATED steps."""
    name = lambda z: ZONE_NAMES.get(z, z)
    part = (robot or {}).get("partial") or {}
    out = []
    for st in steps:
        a, zs = st.get("action"), st.get("zones") or []
        if a == "clean":
            bits = []
            for z in zs:
                p = part.get(z)
                bits.append(name(z) + (f" (restarting from the beginning; it was "
                                       f"about {p} percent done)" if p else ""))
            out.append("cleaning " + ", then ".join(bits))
        elif a == "skip":
            out.append("skipping " + ", ".join(name(z) for z in zs))
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
    return s[0].upper() + s[1:] + "."


# ------------------------------------------------------------------ tests ---
PAUSED_IN_AISLE_1 = {"zone": "aisle_1", "goal": 5, "total": 24, "paused": True,
                     "queue": ["aisle_1", "aisle_3"], "cleaned": [],
                     "partial": {"aisle_1": 36}, "coverage_pct": 3.3,
                     "x": -1.01, "y": 12.71, "blocked": []}
IN_AISLE_3 = {"zone": "aisle_3", "goal": 8, "total": 13, "paused": False,
              "queue": ["aisle_3"], "cleaned": [], "partial": {"aisle_1": 36, "aisle_3": 45},
              "coverage_pct": 10.2, "x": 7.54, "y": 2.70, "blocked": []}


def test_carry_on_while_paused_is_resume():
    ok, cmd, reply = new_head("ok carry on", PAUSED_IN_AISLE_1)
    assert ok and cmd == {"action": "resume", "zones": [],
                          "reason": "told to continue while paused"}, (ok, cmd)
    assert reply == "Resuming from where I stopped (5 of 24, in aisle 1).", reply


def test_continue_variants():
    for s in ("continue", "keep going", "go ahead", "so can you start cleaning",
              "carry on then", "resume", "proceed", "keep cleaning"):
        ok, cmd, _ = new_head(s, PAUSED_IN_AISLE_1)
        assert ok and cmd["action"] == "resume", (s, ok, cmd)


def test_continue_does_not_hijack_real_orders():
    # a named zone, a restart request, or not being paused all fall through
    for s, st in (("carry on with aisle 2", PAUSED_IN_AISLE_1),
                  ("start aisle 1 over", PAUSED_IN_AISLE_1),
                  ("start again", PAUSED_IN_AISLE_1),
                  ("carry on", IN_AISLE_3),
                  ("clean aisle 3", PAUSED_IN_AISLE_1),
                  ("stop", PAUSED_IN_AISLE_1)):
        ok, cmd, _ = new_head(s, st)
        assert not (ok and cmd and cmd.get("action") == "resume"), (s, ok, cmd)


def test_what_have_you_cleaned_is_answered_from_state():
    ok, cmd, reply = new_head("what have you cleaned so far", IN_AISLE_3)
    assert ok and cmd is None
    assert "Nothing finished yet." in reply and "aisle 1 at about 36 percent" in reply \
        and "aisle 3 at about 45 percent" in reply and "10.2 percent" in reply, reply
    ok, _, reply = new_head("what have you actaully cleaned", PAUSED_IN_AISLE_1)
    assert ok and "36 percent" in reply, reply


def test_how_much_is_left_is_not_swallowed():
    for s in ("how much is left?", "what's left to clean", "how much is remaining"):
        ok, _, _ = new_head(s, IN_AISLE_3)
        assert not ok, s


def test_standing_in_a_zone_is_zero_metres():
    r = _ranked(IN_AISLE_3, True, True)
    assert r[0] == (0.0, "aisle_3"), r                 # was (4.8, 'aisle_3')
    assert [z for _, z in r] == ["aisle_3", "aisle_2", "aisle_1"], r


def test_distances_outside_a_zone_are_unchanged():
    at_dock = {"x": 0.0, "y": 0.0, "cleaned": [], "blocked": []}
    r = dict((z, round(d, 2)) for d, z in _ranked(at_dock, False, True))
    assert r["aisle_1"] == round(math.hypot(-0.49, 7.50), 2), r
    assert r["front"] == 0.0, r                        # the dock IS in the front zone


def test_blocked_zone_still_excluded():
    st = dict(IN_AISLE_3, blocked=["aisle_3"])
    assert [z for _, z in _ranked(st, True, True)] == ["aisle_2", "aisle_1"]


def test_confirmations_say_what_was_actually_sent():
    assert confirm([{"action": "clean", "zones": ["aisle_3"]}], IN_AISLE_3) == \
        "Cleaning aisle 3 (restarting from the beginning; it was about 45 percent done)."
    assert confirm([{"action": "resume", "zones": []}], PAUSED_IN_AISLE_1) == \
        "Resuming from where I stopped."
    assert confirm([{"action": "stop", "zones": []},
                    {"action": "clean", "zones": ["front", "back"]}], {}) == \
        "Stopping, then cleaning the front area, then the back area."
    assert confirm([{"action": "skip", "zones": ["aisle_3"]}], {}) == "Skipping aisle 3."


def test_lane_end_turn_is_still_inside_the_aisle():
    """fixed_1 turned at (0.01, 14.04); the old box ended at 13.90."""
    assert _zone_gap("aisle_1", 0.01, 14.04) == 0.0
    assert _zone_gap("aisle_1", -1.00, 0.95) == 0.0


def test_the_robots_own_zone_wins_over_geometry():
    far_north = dict(IN_AISLE_3, zone="aisle_3", x=7.51, y=14.80)
    assert _zone_gap("aisle_3", 7.51, 14.80, "aisle_3") == 0.0
    assert _ranked(far_north, True, True)[0] == (0.0, "aisle_3")
    # with no zone reported, geometry still decides
    assert _zone_gap("aisle_1", 7.51, 14.80) > 5.0


def test_reply_never_says_zero_metres():
    r = _ranked(IN_AISLE_3, True, True)
    cmd, reply = nearest_reply(r, True, True, True, IN_AISLE_3, True)
    assert "0.0 m" not in reply and "I'm in it already" in reply, reply
    assert "Restarting it from the beginning; it was about 45 percent done." in reply, reply
    assert cmd["zones"] == ["aisle_3"]
    _, ask = nearest_reply(r, True, True, True, IN_AISLE_3, False)
    assert ask == "The nearest uncleaned aisle is aisle 3, I'm in it already.", ask


def test_reply_keeps_metres_when_it_is_somewhere_else():
    at_dock = {"x": 0.0, "y": 0.0, "cleaned": [], "blocked": [], "zone": "start",
               "partial": {}}
    r = _ranked(at_dock, True, True)
    _, reply = nearest_reply(r, True, True, True, at_dock, True)
    assert "about 7.5 m away" in reply and "I'm in it already" not in reply, reply
    assert "Restarting" not in reply, reply          # nothing partial at the dock


def test_run1_turn3_no_longer_lies():
    """The actual failure: 'ok carry on' -> clean aisle_3, said 'the back area'."""
    ok, cmd, reply = new_head("ok carry on", PAUSED_IN_AISLE_1)
    assert cmd["action"] == "resume"
    assert "back" not in reply and "aisle 3" not in reply, reply


if __name__ == "__main__":
    import sys
    bad = 0
    for n, f in sorted(globals().items()):
        if n.startswith("test_") and callable(f):
            try:
                f()
                print("PASS", n)
            except AssertionError as e:
                bad += 1
                print("FAIL", n, "::", e)
    print("\n" + ("ALL GREEN" if not bad else f"{bad} FAILED"))
    sys.exit(1 if bad else 0)
