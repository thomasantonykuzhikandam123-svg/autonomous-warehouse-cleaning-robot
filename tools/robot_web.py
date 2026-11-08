#!/usr/bin/env python3
"""robot_web.py (v10.10, 14 Oct) - Mission Control: the robot live in a web page, with a talk button.

v10.13 (2 Nov): the wet-floor guardian (wet_floor.py), live and in replay: where it has wet the floor and how
long until dry, the water off where people walk (learned from its own tracks, only where it could see) or
someone is about to, warnings when a person it tracks heads onto wet floor; /api/wet and 'wet' in the state;
'where is the floor wet?' answered in code. It decides only the water and what to say: never the motion.
v10.12 (2 Nov): /twin, the robot's view in 3D (robot_twin.html, three.js r160 from tools/vendor or,
failing that, the CDN): the same /api/state drawn as a warehouse you can fly round, with the guard's
predictions replayed as 'imagined futures'. Presentation only: nothing the robot does changes.
v10.10b (15 Oct, the rehearsal's microphone): 'Please Swan', 'Clean Awan', 'Play Selvan', 'Please iron'
(clean aisle one, as Safari wrote it) -> clean aisle 1: an order word, then one word that says or sounds
like the number; 'first / second / third aisle', 'row / lane N' too. An order whose zone was lost
('Clean Hazel', 'Clean') is ASKED about, with buttons, and the answer ('the second', 'yes') taken -- the
model is never left to guess a zone (it sent 'Clean Awan' to the front). A clean the model plans for
words that name no zone is not sent; broken JSON from the model is never shown. Why the model is down
(refused / timed out, and where) is said.
v10.10 (after the rehearsal): what a microphone makes of this vocabulary ('Hazel three', 'Al two',
'L1', 'bacteria') is put right, and of the recogniser's top 5 guesses the first that is a command
wins; the model gets the live situation with every question; how long, what do you see and how
fast are answered in code (they work with the model down); the page says when the model is down.

    python3 ~/ros2_ws/src/tools/robot_web.py           (shift.sh demo starts it in a 'web' tab)
    python3 ~/ros2_ws/src/tools/robot_web.py --replay ~/runs shift_v108a      (no ROS: a recorded shift)

Open the address it prints (http://<VM address>:8080) in a browser on the Mac. The microphone only
works when the page comes from localhost, so for voice run  python3 mac_web.py  on the Mac (the page
offers it at /mac.py) and open http://localhost:8080 instead. Typing always works, and so does the
Mac's own dictation (press the dictation key, or Fn twice) into the talk box.

The page shows, from the topics the stack already publishes (nothing about the robot is decided here):
  the map (/map) and the floor cleaned so far (/coverage_passes from coverage_recorder; painted here
  from the robot's pose until that arrives), the robot (TF map -> base_link) with its trail and the
  path it is following (/plan), what the lidar sees (/scan), what the people guard sees and predicts
  (/people_markers: each person, where they may walk next, the walkway, the stop line) and its
  decision (/people_guard/state), the job (/mission_state), what the robot says (narrator.py's
  Narrator on /mission_state), the conversation (/dialogue: the talk tab's turns and this page's), the
  camera with the detector's boxes (/eye/image_annotated), and, if ticked, where the simulator really
  has everyone (/traffic/truth).

It can:
  talk  typed or spoken words go through the same robot agent as the talk tab (robot_agent.py:
        commands checked by mission_command, questions answered from /mission_state, llama3.2 on the
        Mac only for what no rule covers); the reply comes back as text and, if switched on, voice.
  STOP  the emergency stop: True on /emergency_stop (guard_gate.py holds every velocity command at
        zero, between Nav2 and the Collision Monitor, until released) and, with it, {"action": "stop"}
        on /mission_command (coverage_nav2 pauses the job and cancels the goal, so nothing times out
        while the robot is held). No model involved. Release lets commands through again; the robot
        stays paused until told 'carry on'.
  a click on a zone of the map is the same as saying 'clean <zone>'.
Every turn goes on /dialogue (with "via": "web: ..."), so coverage_recorder files it like the talk
tab's. Replay mode re-runs the real people guard (people_guard.GuardCore) on the recorded scans, so
the recorded shift shows the same predictions and decisions; nothing is sent to a robot then.
"""
import argparse
import base64
import collections
import csv
import gzip
import importlib.util
import itertools
import json
import math
import os
import queue
import re
import socket
import subprocess
import sys
import threading
import time
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

VERSION = 'v10.14'
HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, 'robot_web.html')
TWIN = os.path.join(HERE, 'robot_twin.html')    # v10.12: the robot's view in 3D
THREE_JS = os.path.join(HERE, 'vendor', 'three.module.min.js')   # three.js r160, put there by the installer
MAC = os.path.join(HERE, 'mac_web.py')
EAR = os.path.join(HERE, 'ear_server.py')       # v10.11: Whisper on the Mac, served for download
PORT = 8080
BRUSH = 0.32             # m: half the brush width coverage_recorder paints with (clean_width 0.64)
TRAIL_STEP = 0.05        # m between two trail points
TRAIL_MAX = 6000         # points kept (300 m of driving)
FROZEN_S = 10.0          # s of real time without sim time moving on: the page says the sim is frozen

# zones as robot_agent.py has them (map frame: x0, x1, y0, y1); a click inside one asks to clean it
ZONES = {
    'front':   (-3.60, 10.60, -3.20, 1.00),
    'aisle_1': (-1.94, 0.96, 0.80, 14.40),
    'aisle_2': (2.06, 4.96, 0.80, 14.40),
    'aisle_3': (6.06, 9.01, 0.80, 14.40),
    'back':    (-3.60, 10.60, 14.00, 15.30),
}
ZONE_SAY = {'front': 'the front area', 'aisle_1': 'aisle 1', 'aisle_2': 'aisle 2',
            'aisle_3': 'aisle 3', 'back': 'the back area'}
DOCK = (0.0, 0.0)
HABITS = os.path.expanduser('~/runs/wet_habits.npz')   # v10.13: where people walk, kept between shifts (live)


def log(msg):
    print(f'[{time.strftime("%H:%M:%S")}] web: {msg}', flush=True)


_WF = []


def wet_module():
    """v10.13: wet_floor.py next to this file (None, and the page shows no wet layer, if it is missing)."""
    if not _WF:
        try:
            spec = importlib.util.spec_from_file_location('wet_floor', os.path.join(HERE, 'wet_floor.py'))
            m = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(m)
            _WF.append(m)
        except Exception as e:
            log(f'no wet_floor.py ({type(e).__name__}: {e}): no wet-floor layer')
            _WF.append(None)
    return _WF[0]


_RW, _PG = [], []


def rewind_module():
    """v10.14: rewind.py next to this file (None, and the page has no Rewind, if it is missing)."""
    if not _RW:
        try:
            spec = importlib.util.spec_from_file_location('rewind', os.path.join(HERE, 'rewind.py'))
            m = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(m)
            _RW.append(m)
        except Exception as e:
            log(f'no rewind.py ({type(e).__name__}: {e}): no Rewind')
            _RW.append(None)
    return _RW[0]


def guard_module():
    """v10.14: people_guard.py next to this file, for Rewind's model of going on (live mode)."""
    if not _PG:
        try:
            spec = importlib.util.spec_from_file_location('pg_rewind', os.path.join(HERE, 'people_guard.py'))
            m = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(m)
            _PG.append(m)
        except Exception as e:
            log(f'no people_guard.py for Rewind ({type(e).__name__}: {e})')
            _PG.append(None)
    return _PG[0]


# ------------------------------------------------------------------ words heard by a microphone
_NUM = {'one': '1', 'won': '1', 'want': '1', 'two': '2', 'to': '2', 'too': '2', 'tu': '2', 'three': '3',
        'tree': '3', 'free': '3', 'thri': '3', 'sri': '3', '1': '1', '2': '2', '3': '3'}
# what a microphone makes of "aisle" (Safari and Chrome, an Indian-English speaker, 14 Oct): isle,
# I'll, Al, L, Hazel, air, ail, ale ... always followed by the aisle's number
_AISLE_HEARD = re.compile(r"\b(?:aisles?|isles?|i'?ll|ill|ails?|aile|ales?|al|el|l|hazel|hazels|eye\s?l+|air)"
                          r"\s*-?\s*(one|won|want|two|to|too|tu|three|tree|free|thri|sri|[123])\b", re.I)
_HEARD = [
    (re.compile(r"\b(bacteria|back\s?aria|back\s?idea|bag\s?area|back\s?error|backarea|black\s?area)\b", re.I),
     'back area'),
    (re.compile(r"\b(front\s?idea|front\s?aria|friend\s?area|frontier|front\s?error|fruit\s?area)\b", re.I),
     'front area'),
    (re.compile(r"\b(are you|you)\s+(playing|planing|claiming)\b", re.I), r'\1 cleaning'),
    (re.compile(r"\b(veedu|veetu|vidu)\b", re.I), 'go home'),
]
_ROW = re.compile(r"\b(?:row|lane|line|number)\s*(one|won|wan|two|to|too|three|tree|free|[123])\b", re.I)
_LEAD = re.compile(r"^\s*(please|plees|lean|clin|kleen|clear|plain|green)\b(?=\s+(the\s+)?(aisle|front|back))", re.I)

# v10.10b (15 Oct): "clean aisle one" as the recogniser wrote it on the day: 'Please Swan', 'Please Awan',
# 'Clean Awan', 'Play Selvan', 'Play Awan', 'Please iron' -- and 'Clean Hazel' (the number lost). An order
# word, then one word that says or sounds like the number, is that aisle; an order with the zone lost is
# asked about (with buttons), never guessed by the model.
_ORD = {'first': '1', '1st': '1', 'second': '2', '2nd': '2', 'third': '3', '3rd': '3'}
_EXACT = dict(_NUM, wan='1', van='1', shree='3')
_ENDS = (('1', re.compile(r"(?:w|v|r|l)[aou]n$")),                  # swan awan selvan iron ivan won
         ('2', re.compile(r"t(?:o|oo|u|ew|wo|ue)$")),                # altu itu atoo
         ('3', re.compile(r"(?:t|th|f|s|sh)r(?:ee|i|y|ea|ie)$")))     # eltree sri free
_SOUNDS = {'1': ('one', 'won', 'wan', 'van', 'awan', 'swan', 'iwan', 'ivan', 'evan', 'iwon'),
           '2': ('two', 'too', 'tu', 'atu', 'itu', 'eltu', 'altu'),
           '3': ('three', 'tree', 'free', 'thri', 'sri', 'athree', 'ithree', 'isletree')}
_CLEANISH = {'clean', 'clin', 'kleen', 'klin', 'cleen', 'lean', 'glean', 'green', 'queen', 'clear', 'plain',
             'plane', 'play', 'played', 'pray', 'screen', 'scream', 'cream', 'keen', 'clint', 'quinn', 'klein',
             'cline', 'kline', 'colleen', 'sweep', 'mop', 'wash', 'scrub'}
_LEADS = _CLEANISH | {'please', 'plees', 'pleas', 'pls', 'go', 'goto', 'start', 'do'}
_FILL = {'the', 'a', 'an', 'up', 'to', 'into', 'number', 'no', 'now', 'and', 'then', 'of', 'can', 'could',
         'would', 'will', 'you', 'kindly', 'just', 'zone'}
_AISLEISH = re.compile(r"^(?:aisles?|isles?|i'?ll|ill|ails?|aile|ales?|al|el|l|hazels?|eye|air|i)$")
_OTHER = {'stop', 'carry', 'resume', 'continue', 'home', 'dock', 'why', 'what', 'how', 'who', 'when', 'where',
          'introduce', 'yourself', 'see', 'long', 'fast', 'pause', 'wait', 'front', 'back', 'everything', 'all',
          'done', 'left', 'hello', 'hi', 'hey', 'thank', 'thanks', 'me', 'it', 'that', 'this', 'them', 'on',
          'ahead', 'again', 'explain', 'tell', 'cancel', 'sorry', 'yes', 'battery', 'status', 'help', 'is', 'are',
          'was', 'were', 'did', 'does', 'have', 'has', 'rest', 'remaining', 'nearest', 'next', 'dirty', 'area',
          'whole', 'floor', 'warehouse', 'same', 'last', 'there', 'here'}
_YES = re.compile(r"^\s*(yes|yeah|yep|yup|ya|yah|correct|right|sure|ok|okay|please do|do it|go ahead|that one|"
                  r"haa?n|exactly)\b", re.I)
_SAYN = {'1': 'first', '2': 'second', '3': 'third'}
ASK_ZONE = ("I didn't catch which zone. Say the first, second or third aisle, or the front or back area, "
            "or tap one below.")
ZONE_CHOICES = [('Aisle 1', 'clean aisle 1'), ('Aisle 2', 'clean aisle 2'), ('Aisle 3', 'clean aisle 3'),
                ('Front area', 'clean the front area'), ('Back area', 'clean the back area')]


def _words(t):
    return re.findall(r"[a-z0-9']+", (t or '').lower())


def sounds_like_aisle(w):
    """The number a word sounds most like ('wonn' -> '1'), if close enough and clearly closer than the
    others. None otherwise."""
    import difflib
    w = re.sub(r'[^a-z]', '', (w or '').lower())
    if not 3 <= len(w) <= 9:
        return None
    best = sorted(((max(difflib.SequenceMatcher(None, w, v).ratio() for v in vs), n) for n, vs in _SOUNDS.items()),
                  reverse=True)
    return best[0][1] if best[0][0] >= 0.75 and best[0][0] - best[1][0] >= 0.2 else None


def number_heard(words):
    """The aisle number in the words after the order: 'first', 'one', '2', 'tree' (one number only); or a
    single word that ends the way a number sounds ('Swan', 'Awan', 'Selvan', 'iron' -> '1'). Else None."""
    nums = {_ORD.get(w) or _EXACT.get(w) for w in words} - {None}
    if len(nums) == 1:
        return nums.pop()
    if nums or len(words) != 1:
        return None
    hits = {n for n, rx in _ENDS if rx.search(words[0])}
    if len(hits) == 1:
        return hits.pop()
    return sounds_like_aisle(words[0])


def _target(t):
    """(the first word is an order word, what follows it without the filler words)."""
    w = _words(t)
    lead = bool(w) and w[0] in _LEADS
    rest = [x for x in (w[1:] if lead else w) if x not in _FILL and x not in _LEADS]
    return w, lead, rest


def guess_command(t):
    """'Please Swan' -> 'clean aisle 1': an order word, then (besides 'aisle' as heard) one word that says
    or sounds like an aisle's number. Or 'first aisle' / 'aisle 2' on its own. None otherwise -- and
    never when the text already names a zone."""
    if re.search(r"\baisle\s*[123]\b|\bfront\b|\bback\b", t or '', re.I):
        return None
    w, lead, rest = _target(t)
    if not w or len(w) > 6 or any(x in _OTHER for x in rest):
        return None
    named = [x for x in rest if not _AISLEISH.match(x)]
    if not lead and len(named) == len(rest):          # no order word: only with 'aisle' as heard
        return None
    n = number_heard(named) if 1 <= len(named) <= 2 else None
    return f'clean aisle {n}' if n else None


def zone_question(t):
    """(question, guess) when it is an order to clean with the zone lost ('Clean Hazel', 'Clean'), or a
    bare word that sounds like a number ('Swan' -> did you mean the first aisle?). None otherwise."""
    w, lead, rest = _target(t)
    if not w or len(w) > 5 or any(x in _OTHER for x in rest):
        return None
    if lead and (w[0] in _CLEANISH or any(_AISLEISH.match(x) for x in rest)) and len(rest) <= 2:
        return ASK_ZONE, None
    if not lead and len(w) <= 2:
        n = number_heard([x for x in rest if not _AISLEISH.match(x)])
        if n:
            return f"Did you mean the {_SAYN[n]} aisle? Say yes, or tap the zone below.", f'clean aisle {n}'
    return None


def zone_answer(t, guess=None):
    """The answer to 'which zone?': 'yes' (to a guess), 'the second', 'aisle three', 'Swan', 'front'."""
    if guess and _YES.match(t or ''):
        return guess
    w = [x for x in _words(t) if x not in _FILL and x not in _LEADS and not _AISLEISH.match(x)]
    if not w or len(w) > 3:
        return None
    if 'front' in w or 'back' in w:
        return None if ('front' in w) == ('back' in w) else f"clean the {'front' if 'front' in w else 'back'} area"
    n = number_heard(w)
    return f'clean aisle {n}' if n else None


def zone_choices(guess=None):
    return sorted(ZONE_CHOICES, key=lambda c: c[1] != guess) if guess else list(ZONE_CHOICES)


def voice_fix(text):
    """What speech recognition makes of this vocabulary, put right: 'Hazel three', 'Al two', 'L1'
    -> 'aisle 3/2/1'; 'bacteria' -> 'back area'; 'please aisle 1' -> 'clean aisle 1'; v10.10b: 'Please
    Swan', 'Play Selvan', 'Please iron' -> 'clean aisle 1'; 'first aisle', 'aisle 2' alone -> clean it.
    Only these patterns are touched; everything else is passed on as heard."""
    t = _AISLE_HEARD.sub(lambda m: 'aisle ' + _NUM[m.group(1).lower()], text or '')
    for rx, to in _HEARD:
        t = rx.sub(to, t)
    t = _ROW.sub(lambda m: 'aisle ' + _NUM[m.group(1).lower()], t)
    t = _LEAD.sub('clean', t)
    t = re.sub(r"^\s*(?:the\s+)?(aisle [123]|front area|back area)\s*[.!?]*\s*$", r'clean \1', t, flags=re.I)
    g = guess_command(t)
    return g if g else re.sub(r'\s+', ' ', t).strip()


_ZONE_SAID = re.compile(r"\b(aisles?|front|back|everything|all|whole|rest|remaining|dirty|nearest|next|zones?|"
                        r"areas?|first|second|third|one|two|three|[123]|it|there|same|again|that|last|both)\b", re.I)
_JSONISH = re.compile(r'^\s*[\[{]|"plan"\s*:|"say"\s*:|"say:"')
_ZONEISH = re.compile(r"\b(aisle [123]|front area|back area|the front|the back|everything|home|dock)\b", re.I)


def pick_heard(cands, parse=None):
    """Of the recogniser's guesses (best first), each put right: the first that is a command,
    else the first that names a zone, else the best."""
    fixed = [voice_fix(c) for c in cands if c and c.strip()]
    if not fixed:
        return ''
    if parse is not None:
        for f in fixed:
            try:
                if parse(f) is not None:
                    return f
            except Exception:
                pass
    for f in fixed:
        if _ZONEISH.search(f):
            return f
    return fixed[0]


def world_worker(t):
    """v10.11: the world's worker (an actor in the world file, not one of traffic.py's agents, so not in
    /traffic/truth): across aisle 1 at y 4.0 on a fixed 6 s loop of sim time (as tourrep.worker; it
    matched the recordings' camera truth to the millimetre). For the 'simulator truth' layer only."""
    wp = [(0.0, -1.6), (2.2, 0.6), (3.0, 0.6), (5.2, -1.6), (6.0, -1.6)]
    p = t % 6.0
    x = -1.6
    for (t0, x0), (t1, x1) in zip(wp, wp[1:]):
        if t0 <= p <= t1:
            x = x0 + (p - t0) / (t1 - t0) * (x1 - x0)
            break
    return {'name': 'worker', 'kind': 'person', 'x': round(x, 3), 'y': 4.0}


def view_polygon(ranges, amin, ainc, rmin, rmax, lx, ly, lyaw, step=2):
    """v10.11: what the lidar sees right now, as a polygon in the map: each beam out to where it
    stopped (no return = clear to its range; under range_min = blocked at once). Everything outside
    it the robot cannot see - a person there is unknown to it unless it remembers them."""
    r = np.asarray(ranges, dtype=float)[::step]
    a = amin + ainc * step * np.arange(len(r)) + lyaw
    r = np.where(np.isfinite(r), r, rmax)
    r = np.where(r <= rmin, rmin, np.minimum(r, rmax))
    return {'o': [round(float(lx), 2), round(float(ly), 2)], 'r': round(float(rmax), 1),
            'pts': [[round(float(x), 2), round(float(y), 2)] for x, y in zip(lx + r * np.cos(a), ly + r * np.sin(a))]}


# ------------------------------------------------------------------ markers -> shapes for the page
def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _rgba(c):
    return [round(float(c.r), 3), round(float(c.g), 3), round(float(c.b), 3), round(float(c.a), 3)]


def marker_shape(m, to_map=None):
    """One visualization_msgs/Marker as a small dict the page draws in 2D (map frame), or None.
    to_map(frame) -> (x, y, yaw) of that frame in the map, for markers not in 'map'."""
    fx, fy, fyaw = 0.0, 0.0, 0.0
    frame = m.header.frame_id or 'map'
    if frame.lstrip('/') != 'map':
        if to_map is None:
            return None
        tf = to_map(frame)
        if tf is None:
            return None
        fx, fy, fyaw = tf
    px, py = m.pose.position.x, m.pose.position.y
    pyaw = _yaw(m.pose.orientation)
    c, s = math.cos(fyaw), math.sin(fyaw)
    ox, oy = fx + c * px - s * py, fy + s * px + c * py
    oyaw = fyaw + pyaw
    co, so = math.cos(oyaw), math.sin(oyaw)

    def pt(p):
        return [round(ox + co * p.x - so * p.y, 3), round(oy + so * p.x + co * p.y, 3)]

    base = {'ns': m.ns, 'id': int(m.id), 'rgba': _rgba(m.color)}
    t = m.type
    if t in (2, 3):                                   # SPHERE, CYLINDER
        base.update(k='c', x=round(ox, 3), y=round(oy, 3), r=round(abs(m.scale.x) / 2.0, 3),
                    h=round(float(m.scale.z), 3))
    elif t == 1:                                      # CUBE
        base.update(k='b', x=round(ox, 3), y=round(oy, 3), w=round(abs(m.scale.x), 3),
                    l=round(abs(m.scale.y), 3), yaw=round(oyaw, 3))
    elif t in (4, 5):                                 # LINE_STRIP, LINE_LIST
        if len(m.points) < 2:
            return None
        base.update(k='l' if t == 4 else 'll', pts=[pt(p) for p in m.points[:2000]],
                    w=round(abs(m.scale.x), 3))
    elif t in (6, 7, 8):                              # CUBE_LIST, SPHERE_LIST, POINTS
        if not m.points:
            return None
        base.update(k='p', pts=[pt(p) for p in m.points[:3000]], s=round(abs(m.scale.x), 3))
        if m.colors and len(m.colors) == len(m.points):
            base['rgba'] = _rgba(m.colors[0])
    elif t == 9:                                      # TEXT_VIEW_FACING
        base.update(k='t', x=round(ox, 3), y=round(oy, 3), text=str(m.text)[:160],
                    h=round(abs(m.scale.z), 3))
    elif t == 0:                                      # ARROW
        if len(m.points) >= 2:
            a, b = pt(m.points[0]), pt(m.points[1])
        else:
            a = [round(ox, 3), round(oy, 3)]
            b = [round(ox + co * abs(m.scale.x), 3), round(oy + so * abs(m.scale.x), 3)]
        base.update(k='a', pts=[a, b], w=round(max(abs(m.scale.y), 0.03), 3))
    else:
        return None
    return base


class MarkerSet:
    """The markers of one topic, kept by (ns, id) with ADD / DELETE / DELETEALL as RViz does."""

    def __init__(self):
        self.items = collections.OrderedDict()

    def apply(self, markers, to_map=None):
        for m in markers:
            if m.action == 3:                         # DELETEALL
                self.items.clear()
            elif m.action == 2:                       # DELETE
                self.items.pop((m.ns, m.id), None)
            else:
                sh = marker_shape(m, to_map)
                if sh is None:
                    self.items.pop((m.ns, m.id), None)
                else:
                    self.items[(m.ns, m.id)] = sh

    def shapes(self):
        return list(self.items.values())


# ------------------------------------------------------------------ the shared state
class Painter:
    """The floor swept by the brush, from the robot's poses (used until coverage_recorder's
    /coverage_passes arrives, and in replay)."""

    def __init__(self, info, radius=BRUSH):
        self.w, self.h, self.res, self.ox, self.oy = info['w'], info['h'], info['res'], info['ox'], info['oy']
        self.grid = np.zeros((self.h, self.w), np.int8)
        r = int(math.ceil(radius / self.res))
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
        keep = (xx * xx + yy * yy) * self.res * self.res <= radius * radius
        self.dy, self.dx = yy[keep], xx[keep]
        self.last = None

    def paint(self, x, y):
        if self.last is None:
            self.last = (x, y)
        lx, ly = self.last
        d = math.hypot(x - lx, y - ly)
        if d > 2.0:                                   # a jump (relocalised, replay seek): no smear
            lx, ly, d = x, y, 0.0
        n = max(1, int(d / (2 * self.res)) + 1)
        for k in range(n + 1):
            f = k / n
            cx = int((lx + (x - lx) * f - self.ox) / self.res)
            cy = int((ly + (y - ly) * f - self.oy) / self.res)
            ix, iy = self.dx + cx, self.dy + cy
            ok = (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
            cells = self.grid[iy[ok], ix[ok]]
            self.grid[iy[ok], ix[ok]] = np.maximum(cells, 30)
        self.last = (x, y)


class Hub:
    """Everything the page shows, written by the ROS (or replay) side, read by the web side."""

    def __init__(self, mode):
        self.lock = threading.Lock()
        self.mode = mode
        self.started = time.time()
        self.feed_wall = {}
        self.wet = None                 # v10.13: the wet-floor guardian (wet_floor.WetFloor), once the map is in
        self.wet_v, self.wet_v_wall, self.wet_cache = 0, 0.0, None
        self.wet_said = {}              # kind -> wall time it last spoke
        self.wet_done_said = False
        self.wet_saved_wall = time.time()
        self.rew = None                 # v10.14: Rewind (rewind.Rewind): every stop for a person, and the future it avoided
        self.rew_said = 0.0             # wall time a stop's verdict was last said
        self.reset()
        self.map_info, self.map_bytes, self.map_v, self.map_cache = None, None, 0, None
        self.events = collections.deque(maxlen=800)
        self.seq = 0
        self.agent = {'ok': False, 'model': None, 'model_up': None, 'why': 'starting', 'busy': False}
        self.replay = None              # replay status dict (replay mode)
        self.estop = {'on': False, 'since': None, 'by': None}   # the emergency stop, as last seen
        self.camera_raw = None          # (stamp, h, w, encoding, bytes)
        self.camera_jpg = None          # (stamp, jpeg bytes)
        self.cam_lock = threading.Lock()
        self.cam_tried = False
        self.cv2 = None

    FEEDS = {'map': '/map', 'pose': 'TF map -> base_link', 'mission': '/mission_state',
             'guard': '/people_guard/state', 'markers': '/people_markers', 'scan': '/scan',
             'plan': '/plan', 'coverage': '/coverage_passes', 'truth': '/traffic/truth',
             'camera': '/eye/image_annotated', 'dialogue': '/dialogue'}

    def seen(self, feed):
        """Called with the lock held, or for counters only: the wall time each feed last arrived."""
        now = time.time()
        if feed not in self.feed_wall and self.mode == 'live':
            log(f'first {self.FEEDS.get(feed, feed)}')
        self.feed_wall[feed] = now

    def reset(self):
        """Robot, coverage and history (a new replay lap starts from a clean floor)."""
        self.robot = None               # dict x y yaw t
        self.trail = collections.deque(maxlen=TRAIL_MAX)
        self.trail_base = 0             # index of trail[0] since the last reset
        self.trail_epoch = getattr(self, 'trail_epoch', 0) + 1
        self.dist = 0.0
        self.speed = 0.0
        self.plan = None
        self.scan = None
        self.view = None                # v10.11: what the lidar sees now (view_polygon)
        self.markers = MarkerSet()
        self.guard = None
        self.guard_wall = None
        self.mission = None
        self.mission_wall = None
        self.truth = None
        self.truth_wall = None
        self.holds = []                 # [x, y, sim t0, sim t1 or None, say]
        self.hold_open = None
        self.painter = None
        self.cov_rec = None             # coverage_recorder's grid (int8 bytes)
        self.cov_rec_wall = None
        self.cov_v = getattr(self, 'cov_v', 0) + 1
        self.cov_cache = None
        self.cov_dirty_wall = 0.0
        self.sim_t = None
        self.sim_wall = None            # wall time at which sim time last moved on
        self.rtf_ref = None
        self.rtf = None
        if getattr(self, 'wet', None) is not None:     # v10.13: a clean, dry floor again (habits kept)
            self.wet.reset_floor()
            self.wet_done_said = False

    # -------- writers (ROS / replay side)
    def set_map(self, w, h, res, ox, oy, data):
        with self.lock:
            self.seen('map')
            info = {'w': int(w), 'h': int(h), 'res': float(res), 'ox': float(ox), 'oy': float(oy)}
            raw = np.asarray(data, dtype=np.int8).tobytes()
            if self.map_info == info and self.map_bytes == raw:
                return
            self.map_info, self.map_bytes = info, raw
            self.map_v += 1
            self.map_cache = None
            self.painter = Painter(info)
            self.cov_v += 1
            self.cov_cache = None
            WF = wet_module()
            if WF is not None:
                old = self.wet
                self.wet = WF.WetFloor(info['w'], info['h'], info['res'], info['ox'], info['oy'])
                if old is not None and old.P.shape == self.wet.P.shape:
                    self.wet.P, self.wet.O = old.P, old.O
                elif self.mode == 'live' and os.path.exists(HABITS):
                    if self.wet.load(HABITS):
                        log(f'wet floor: where people walk, from earlier shifts ({HABITS})')
                self.wet_cache = None

    def set_coverage(self, w, h, data):
        with self.lock:
            self.seen('coverage')
            if self.map_info is None or (int(w), int(h)) != (self.map_info['w'], self.map_info['h']):
                return
            raw = np.asarray(data, dtype=np.int8).tobytes()
            self.cov_rec_wall = time.time()
            if raw != self.cov_rec:
                self.cov_rec = raw
                self.cov_v += 1
                self.cov_cache = None

    def set_clock(self, t):
        if t is None or t <= 0.0:
            return
        now = time.time()
        if self.sim_t is None or t > self.sim_t + 1e-6:
            self.sim_wall = now
        if t < (self.sim_t or 0.0) - 5.0:            # sim restarted / replay jumped back
            self.rtf_ref = None
        self.sim_t = t
        if self.rtf_ref is None or now - self.rtf_ref[0] > 30.0:
            self.rtf_ref = (now, t)
        elif now - self.rtf_ref[0] >= 3.0:
            self.rtf = max(0.0, (t - self.rtf_ref[1]) / (now - self.rtf_ref[0]))
            if now - self.rtf_ref[0] >= 10.0:
                self.rtf_ref = (now, t)

    def set_pose(self, x, y, yaw, t):
        with self.lock:
            self.seen('pose')
            self.set_clock(t)
            r = self.robot
            if r is not None and t is not None and r['t'] is not None and t > r['t']:
                d = math.hypot(x - r['x'], y - r['y'])
                if d < 2.0:
                    v = d / (t - r['t'])
                    self.speed = 0.6 * self.speed + 0.4 * v if t - r['t'] < 1.0 else v
            if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) >= TRAIL_STEP:
                if self.trail:
                    step = math.hypot(x - self.trail[-1][0], y - self.trail[-1][1])
                    if step < 2.0:
                        self.dist += step
                if len(self.trail) == self.trail.maxlen:
                    self.trail_base += 1
                self.trail.append((round(x, 3), round(y, 3)))
            self.robot = {'x': x, 'y': y, 'yaw': yaw, 't': t}
            if self.painter is not None:
                self.painter.paint(x, y)
                now = time.time()
                if self.cov_rec is None and now - self.cov_dirty_wall >= 1.0:
                    self.cov_dirty_wall = now
                    self.cov_v += 1
                    self.cov_cache = None

    def set_plan(self, pts):
        with self.lock:
            self.seen('plan')
            if pts is None or len(pts) < 2:
                self.plan = None
                return
            pts = np.asarray(pts, dtype=float)
            if len(pts) > 300:
                pts = pts[np.linspace(0, len(pts) - 1, 300).astype(int)]
            self.plan = [[round(float(a), 3), round(float(b), 3)] for a, b in pts]

    def set_scan(self, xy, view=None):
        with self.lock:
            self.seen('scan')
            self.scan = None if xy is None else [[round(float(a), 2), round(float(b), 2)] for a, b in xy]
            self.view = view

    def set_markers(self, markers, to_map=None):
        with self.lock:
            self.seen('markers')
            self.markers.apply(markers, to_map)

    def set_shapes(self, shapes):
        with self.lock:
            self.markers.items = collections.OrderedDict(((s['ns'], s['id']), s) for s in shapes)

    def set_guard(self, g):
        with self.lock:
            self.seen('guard')
            self.guard = g
            self.guard_wall = time.time()
            busy = self.mission is not None and not (self.mission.get('done') or self.mission.get('paused')
                                                       or self.mission.get('awaiting_command'))
            t = g.get('t') or self.sim_t or 0.0
            if g.get('state') == 'wait' and busy and self.robot is not None:
                if self.hold_open is None:
                    self.hold_open = [round(self.robot['x'], 2), round(self.robot['y'], 2), t, None,
                                      g.get('say') or '']
                    self.holds.append(self.hold_open)
                    self.holds = self.holds[-200:]
                else:
                    self.hold_open[4] = g.get('say') or self.hold_open[4]
            elif self.hold_open is not None:
                self.hold_open[3] = t
                self.hold_open = None

    def set_mission(self, s):
        with self.lock:
            self.seen('mission')
            asked = len(s.get('asked') or [])
            j = getattr(self, 'job', None)
            if (s.get('total') or 0) > 0 and (j is None or asked != j['asked'] or (s.get('goal') or 0) < j['goal0']
                                             or (s.get('total') or 0) != j['total']):
                self.job = {'t0': self.sim_t or 0.0, 'goal0': s.get('goal') or 1, 'asked': asked,
                            'total': s.get('total') or 0}
            self.mission = s
            self.mission_wall = time.time()

    def set_truth(self, agents):
        with self.lock:
            self.seen('truth')
            if self.sim_t is not None and not any(a.get('name') == 'worker' for a in agents):
                agents = list(agents) + [world_worker(self.sim_t)]      # v10.11: the world's own actor
            self.truth = agents
            self.truth_wall = time.time()
            if self.wet is not None and self.sim_t is not None:
                self.wet.measure(self.sim_t, agents)   # v10.13: who really stepped on wet floor (counting only)

    def set_camera(self, stamp, h, w, enc, data):
        with self.lock:
            self.seen('camera')
        with self.cam_lock:
            self.camera_raw = (stamp, h, w, enc, data)

    def add_event(self, kind, **kw):
        with self.lock:
            self.seq += 1
            ev = {'seq': self.seq, 'kind': kind, 'wall': round(time.time(), 2),
                  'sim': None if self.sim_t is None else round(self.sim_t, 1)}
            ev.update(kw)
            self.events.append(ev)
            return self.seq

    def set_estop(self, on, by):
        with self.lock:
            if bool(on) != self.estop['on']:
                self.estop = {'on': bool(on), 'since': time.time(), 'by': by}
                return True
            return False

    def mission_copy(self):
        with self.lock:
            return json.loads(json.dumps(self.mission)) if self.mission else {}

    # -------- v10.13: the wet-floor guardian
    WET_SAY_GAP = {'water_off': 30.0, 'warn': 4.0}

    def wet_update(self, t, mission=None, shapes=None, view=None):
        """One step of the wet-floor guardian (live: the 10 Hz tick; replay: every scan). What it says goes
        on the page as 'say' events; a warning carries where the person is (the 3D view marks them)."""
        WF = wet_module()
        if WF is None or t is None:
            return
        with self.lock:
            w, r = self.wet, self.robot
            if w is None or r is None:
                return
            m = self.mission if mission is None else mission
            sh = self.markers.shapes() if shapes is None else shapes
            vw = self.view if view is None else view
            w.step(t, r['x'], r['y'], WF.cleaning_now(m, ZONES, r['x'], r['y'], self.speed),
                   WF.people_from_shapes(sh), vw)
            events = w.take_events()
            now = time.time()
            if now - self.wet_v_wall >= 1.0:
                self.wet_v += 1
                self.wet_v_wall = now
                self.wet_cache = None
            finish = None
            if m and m.get('done'):
                if not self.wet_done_said:
                    self.wet_done_said = True
                    finish = w.finish_text()
            elif m and m.get('zone') in ZONES:
                self.wet_done_said = False
            if self.mode == 'live' and now - self.wet_saved_wall > 60.0:
                self.wet_saved_wall = now
                try:
                    w.save(HABITS)
                except Exception as e:
                    log(f'wet floor: could not keep where people walk ({e})')
        for kind, text, extra in events:
            if now - self.wet_said.get(kind, 0.0) < self.WET_SAY_GAP.get(kind, 10.0):
                continue
            self.wet_said[kind] = now
            if kind == 'water_off':
                self.add_event('say', text=f'Water off here: {text}. I clean this bit dry.', wet='off', **extra)
            else:
                self.add_event('say', text=text, wet='warn', **extra)
        if finish:
            self.add_event('say', text=finish, wet='done')

    def wet_json(self):
        with self.lock:
            WF = wet_module()
            if self.wet is None or WF is None:
                return None
            if self.wet_cache is None:
                w = self.wet
                wet, hab = w.grids(self.sim_t or 0.0)
                self.wet_cache = json.dumps({
                    'v': self.wet_v, 'w': w.W, 'h': w.H, 'res': WF.RES, 'ox': w.ox, 'oy': w.oy, 't_dry': w.t_dry,
                    'wet': base64.b64encode(wet.tobytes()).decode(), 'hw': w.HW, 'hh': w.HH, 'hres': WF.HRES,
                    'habit': base64.b64encode(hab.tobytes()).decode()}).encode()
            return self.wet_cache

    # -------- v10.14: Rewind
    REW_SAY_GAP = 20.0

    def rewind_engine(self):
        """Live: made on first use (a replay sets hub.rew to the recorded shift's stops)."""
        if self.rew is None and self.mode == 'live':
            RW, pg = rewind_module(), guard_module()
            if RW is not None and pg is not None:
                self.rew = RW.Rewind(pg)
        return self.rew

    def rewind_update(self, t):
        """Live, every tick: a WAIT for a person starts a stop; 4 s later, the future it avoided."""
        rw = self.rewind_engine()
        RW = rewind_module()
        if rw is None or RW is None or t is None:
            return []
        with self.lock:
            r = self.robot
            if r is None:
                return []
            now = time.time()
            g = self.guard if (self.guard_wall and now - self.guard_wall < 2.0) else None
            tr = self.truth if (self.truth_wall and now - self.truth_wall < 2.0) else None
            rw.step(t, (r['x'], r['y'], r['yaw']), self.speed, self.plan, g, tr,
                    RW.people_from_shapes(self.markers.shapes()))
            done = rw.take_events()
        for st in done:
            self.rewind_announce(st)
        return done

    def rewind_announce(self, st):
        """A stop's verdict, on the page (and said, when it mattered)."""
        self.add_event('stop', id=st['id'], cls=st['cls'], text=st.get('text'), x=st['x'], y=st['y'],
                       who=st.get('who'), gap=st.get('gap'))
        now = time.time()
        if st['cls'] in ('hit', 'near') and now - self.rew_said >= self.REW_SAY_GAP:
            self.rew_said = now
            self.add_event('say', text='That stop mattered. ' + (st.get('text') or ''), stop=st['id'])

    def rewind_now(self):
        """The sim time up to which stops may be shown: everything (live), or what has been played (replay)."""
        return self.sim_t if self.mode == 'replay' else None

    def rewind_list(self):
        with self.lock:
            if self.rew is None:
                return None
            return json.dumps({'v': self.rew.v, 'stops': self.rew.listing(self.rewind_now())},
                              separators=(',', ':')).encode()

    def rewind_detail(self, i):
        with self.lock:
            if self.rew is None:
                return None
            d = self.rew.detail(i, self.rewind_now())
            return None if d is None else json.dumps(d, separators=(',', ':')).encode()

    # -------- readers (web side)
    def state(self, since=0, trail_from=0, epoch=0):
        with self.lock:
            now = time.time()
            r = self.robot
            first = self.trail_base
            if epoch != self.trail_epoch or trail_from < first:
                trail_from = first
            k = max(0, trail_from - first)
            pts = list(self.trail)[k:k + 3000]
            out = {
                'version': VERSION, 'mode': self.mode, 'wall': round(now, 2),
                'sim': None if self.sim_t is None else round(self.sim_t, 2),
                'rtf': None if self.rtf is None else round(self.rtf, 2),
                'frozen': bool(self.sim_wall is not None and now - self.sim_wall > FROZEN_S),
                'map_v': self.map_v, 'cov_v': self.cov_v,
                'robot': None if r is None else {
                    'x': round(r['x'], 3), 'y': round(r['y'], 3), 'yaw': round(r['yaw'], 3),
                    'v': round(self.speed, 2), 'dist': round(self.dist, 1)},
                'trail': None,
                'plan': self.plan,
                'scan': self.scan,
                'view': self.view,
                'shapes': self.markers.shapes(),
                'guard': self.guard,
                'guard_age': None if self.guard_wall is None else round(now - self.guard_wall, 1),
                'mission': self.mission,
                'mission_age': None if self.mission_wall is None else round(now - self.mission_wall, 1),
                'truth': self.truth if (self.truth_wall and now - self.truth_wall < 5.0) else None,
                'holds': [[h[0], h[1], round((h[3] if h[3] is not None else (self.sim_t or h[2])) - h[2], 1),
                           h[3] is None, h[4]] for h in self.holds],
                'events': [e for e in self.events if e['seq'] > since][-200:],
                'seq': self.seq,
                'agent': dict(self.agent),
                'camera': self.camera_raw is not None and self.camera_raw[0] is not None,
                'replay': self.replay,
                'estop': dict(self.estop, for_s=None if not self.estop['on'] or not self.estop['since']
                              else round(now - self.estop['since'], 1)),
                'feeds': {k: round(now - v, 1) for k, v in self.feed_wall.items()},
                'cov_src': 'recorder' if self.cov_rec is not None else 'painted' if self.painter is not None else None,
                'wet': None if self.wet is None else dict(
                    self.wet.summary(self.sim_t or 0.0, truth=bool(self.truth_wall and now - self.truth_wall < 5.0)),
                    v=self.wet_v),
                'rewind': None if self.rew is None else self.rew.summary(self.rewind_now()),
            }
            out['trail'] = {'epoch': self.trail_epoch, 'from': first + k, 'to': first + k + len(pts), 'pts': pts}
            return out

    def map_json(self):
        with self.lock:
            if self.map_info is None:
                return None
            if self.map_cache is None:
                self.map_cache = json.dumps(dict(self.map_info, v=self.map_v, zones=ZONES, dock=DOCK,
                                                 data=base64.b64encode(self.map_bytes).decode())).encode()
            return self.map_cache

    def coverage_json(self):
        with self.lock:
            if self.map_info is None:
                return None
            if self.cov_cache is None:
                if self.cov_rec is not None:
                    raw, src = self.cov_rec, 'recorder'
                elif self.painter is not None:
                    raw, src = self.painter.grid.tobytes(), 'painted'
                else:
                    return None
                self.cov_cache = json.dumps({'v': self.cov_v, 'src': src, 'w': self.map_info['w'],
                                             'h': self.map_info['h'],
                                             'data': base64.b64encode(raw).decode()}).encode()
            return self.cov_cache

    def camera_jpeg(self):
        with self.cam_lock:
            raw = self.camera_raw
            if raw is None:
                return None
            if self.camera_jpg is not None and self.camera_jpg[0] == raw[0]:
                return self.camera_jpg[1]
            if not self.cam_tried:
                self.cam_tried = True
                try:
                    import cv2
                    self.cv2 = cv2
                except Exception as e:
                    log(f'no camera on the page (cv2: {e})')
            if self.cv2 is None:
                return None
            stamp, h, w, enc, data = raw
            try:
                a = np.frombuffer(data, dtype=np.uint8)
                if enc in ('rgb8', 'bgr8'):
                    img = a[:h * w * 3].reshape(h, w, 3)
                    if enc == 'rgb8':
                        img = img[:, :, ::-1]
                elif enc == 'mono8':
                    img = a[:h * w].reshape(h, w)
                elif enc in ('rgba8', 'bgra8'):
                    img = a[:h * w * 4].reshape(h, w, 4)[:, :, :3]
                    if enc == 'rgba8':
                        img = img[:, :, ::-1]
                else:
                    return None
                ok, jpg = self.cv2.imencode('.jpg', np.ascontiguousarray(img),
                                            [self.cv2.IMWRITE_JPEG_QUALITY, 70])
                if not ok:
                    return None
                self.camera_jpg = (stamp, jpg.tobytes())
                return self.camera_jpg[1]
            except Exception:
                return None


# ------------------------------------------------------------------ talking
def brief(s):
    """Where the job is, in one sentence, from /mission_state (no model)."""
    if not s:
        return "I have no report from the cleaning yet."
    pct = s.get('coverage_pct') or 0
    if s.get('awaiting_command'):
        return "I'm at the dock, waiting for a job."
    if s.get('paused'):
        return f"I'm paused; {pct:.0f} percent of the floor is done."
    if s.get('done'):
        return f"I've finished my job; {pct:.0f} percent of the whole floor is clean."
    z = s.get('zone')
    where = 'heading back to the dock' if z == 'home' else f"cleaning {ZONE_SAY.get(z, z)}"
    out = f"I'm {where}, waypoint {s.get('goal')} of {s.get('total')}; {pct:.0f} percent of the whole floor is done."
    q = [ZONE_SAY.get(x, x) for x in (s.get('queue') or []) if x != z]
    if q:
        out += ' After this: ' + ', then '.join(q) + '.'
    return out


_INTRO = re.compile(r"^\s*(please\s+|can you\s+|could you\s+)?(introduce yourself( to [\w ]+)?|who are you|what are you|"
                    r"tell (us|me) about yourself|what can you do|say hello( to [\w ]+)?|(hello|hi),? robot)"
                    r"\s*[?.!]*\s*$", re.I)


def canned(text, s):
    """An introduction, from what the robot really is (no model: it must be right in front of an audience)."""
    if not _INTRO.search(text or ''):
        return None
    return ("Hello! I'm an autonomous floor-cleaning robot for this warehouse. I sweep each zone lane by lane, "
            "and my safety layer watches everyone around me with my lidar: it predicts where each person may "
            "walk next, and stops me at a line before their walkway, so I never cut across someone. "
            "You can ask me why I stopped, how much is left, or send me to clean any zone. " + brief(s))


def default_ollama():
    """The Mac as the VM sees it: the default gateway (as stack.sh works it out), port 11434."""
    env = os.environ.get('OLLAMA')
    if env:
        return env
    try:
        with open('/proc/net/route') as f:
            for line in f.readlines()[1:]:
                p = line.split()
                if len(p) > 2 and p[1] == '00000000':
                    gw = socket.inet_ntoa(int(p[2], 16).to_bytes(4, 'little'))
                    return f'http://{gw}:11434'
    except Exception:
        pass
    return 'http://10.0.2.2:11434'


class Talker:
    """The talk tab's agent, for the page: one turn at a time, in order."""

    replay = False

    def __init__(self, hub, send, dialogue, ollama, model, timeout=90.0, agent=True, planner_dir=None,
                 estop=None):
        self.hub, self.send, self.dialogue = hub, send, dialogue
        self.estop = estop or (lambda on: False)        # publishes /emergency_stop; True if anyone listens
        self.host, self.model, self.timeout = ollama.rstrip('/'), model, timeout
        self.model_tag = None           # v10.11: the exact name Ollama has it under ('llama3.2:latest', ':3b')
        self.q = queue.Queue()
        self.pending = None             # v10.10b: asked 'which zone?' {'t', 'guess'}
        self.history = []
        self.seen = set()               # web_id of the turns this page sent (not shown twice)
        self.ids = itertools.count(1)
        self.alts = {}                  # turn id -> the recogniser's other guesses (voice)
        self.run = f'{int(time.time()) % 100000:05d}'   # tells this run's turns from an earlier one's
        self.graph = self.confirm = self.parse_direct = None
        why = ''
        if planner_dir:
            sys.path.insert(0, planner_dir)
        if agent:
            try:
                from warehouse_planner.robot_agent import build_graph, confirm
                self.graph = build_graph(self.ask)
                self.confirm = confirm
            except Exception as e:
                why = f'robot agent not available ({type(e).__name__}: {e}); commands only'
        else:
            why = 'agent off (--no-agent); commands only'
        try:
            from warehouse_planner.mission_command import parse_direct
            self.parse_direct = parse_direct
        except Exception as e:
            if self.graph is None:
                why += f'; no mission_command either ({e})'
        hub.agent.update(ok=self.graph is not None, model=model, why=why or 'ok')
        if why:
            log(why)
        threading.Thread(target=self.loop, daemon=True).start()
        threading.Thread(target=self.watch_model, daemon=True).start()

    def context(self):
        """v10.10: the live situation, added to every prompt (llama3.2 is small: it answers far
        better from facts in front of it than from nothing)."""
        h = self.hub
        with h.lock:
            g, m, r = dict(h.guard or {}), dict(h.mission or {}), dict(h.robot or {})
            rtf, holds = h.rtf, len(h.holds)
            says = [e['text'] for e in h.events if e['kind'] == 'say'][-3:]
        L = ['LIVE FACTS, measured just now (the output rules above still apply; never invent numbers):']
        if g:
            L.append(f"- My safety layer: {g.get('say') or 'clear'}. In view: {g.get('people', 0)} people, "
                     f"{g.get('vehicles', 0)} vehicles" + (f", the nearest person {g['person_m']:.1f} m away"
                                                          if g.get('person_m') is not None else '') + '.')
        if r:
            L.append(f"- Speed {h.speed:.2f} m/s; driven {h.dist:.0f} m since this page started.")
        if m.get('holds'):
            L.append(f"- Stops for people or vehicles on this job: {m['holds']}, {m.get('held_s') or 0:.0f} s in all.")
        eta = self.eta()
        if eta:
            L.append(f'- {eta}')
        if rtf:
            L.append(f'- The simulation runs at {rtf:.2f} times real time.')
        if says:
            L.append('- What I said lately: ' + ' | '.join(says))
        with h.lock:
            card = h.rew.scorecard(h.rewind_now()) if h.rew is not None else None
        if card:
            L.append('- My stops, checked afterwards against what really happened (v10.14 Rewind): ' + card)
        L.append('- About me: a 0.6 x 0.4 m cleaning robot with a 0.64 m brush, a 360-degree lidar and a camera '
                 '(a person detector on the Mac). My people guard decides 10 times a second: it predicts where '
                 'each person may walk and waits at a stop line outside their walkway. The warehouse: the front '
                 'area with my dock, three aisles between four racks, the back area; pickers in the aisles, a '
                 'worker crossing aisle 1, a visitor, a tugger in the back corridor and a forklift at the front.')
        return '\n'.join(L)

    def eta(self):
        """A rough time left for the job, from its pace so far (waypoints per second of robot time)."""
        h = self.hub
        with h.lock:
            m, t, j = dict(h.mission or {}), h.sim_t, getattr(h, 'job', None)
            rtf = h.rtf
        if not m or m.get('done') or m.get('awaiting_command') or not j or t is None:
            return None
        g, tot = m.get('goal') or 0, m.get('total') or 0
        done = g - j['goal0']
        if tot <= 0 or done < 2 or t - j['t0'] < 20.0:
            return None
        left = (t - j['t0']) / done * max(tot - g + 1, 0)
        wall = f' (about {left / rtf / 60.0:.0f} min on the clock at this simulation speed)' if rtf and rtf < 0.9 else ''
        return f'Estimated time left for this job: about {max(left / 60.0, 0.5):.0f} min of robot time{wall}, from my pace so far.'

    _ASK_ETA = re.compile(r"\b(how long|estimated time|eta|how much time|time (left|remaining)|when will you (be )?"
                          r"(done|finish(ed)?))\b", re.I)
    _ASK_SEE = re.compile(r"\b(what (do|can) you see|how many (people|persons|pickers|humans)|anyone (around|near)|"
                          r"who('s| is) (around|near))\b", re.I)
    _ASK_FAST = re.compile(r"\b(how fast|your speed|what speed)\b", re.I)
    _ASK_WET = re.compile(r"\b(wet|damp|slippery)\b|\bwater (on|off)\b|\byour water\b", re.I)      # v10.13
    # v10.14: Rewind
    _ASK_CLOSEST = re.compile(r"\b(closest|close|nearest|scariest|worst|narrowest) (call|shave|miss|escape|moment|stop)\b|"
                              r"\bnear[- ]?miss(es)?\b|\bshow me (the |your )?(closest|worst|scariest)\b", re.I)
    _ASK_LAST = re.compile(r"\b(show me|replay|rewind|play back|what about) (the |your |that )?(last |latest )?stop\b|"
                           r"\bwhat would (have )?happen(ed)?\b|\b(if|what if) you (had ?n.?t|had not|did ?n.?t|did not) "
                           r"(stop(ped)?|wait(ed)?)\b|\brewind\b", re.I)
    _ASK_STOPS = re.compile(r"\bhow many (times )?(did|have) you (stop|stopped|wait|waited)\b|\bhow many stops\b|"
                            r"\b(were|was) (all )?(your |those |the |that )?stops? (needed|necessary|worth it|real|justified)\b|"
                            r"\bdid (your |those |that )?stops? (matter|help)\b|\bstops? (that )?mattered\b|"
                            r"\bstop(ping)? so (much|often)\b|\bscorecard\b|\bunnecessary stops?\b|\bover[- ]?cautious\b", re.I)

    def code_answer(self, text):
        """v10.10: questions answered from the live state in code (they work with the model down)."""
        t = text or ''
        h = self.hub
        rew = self.rewind_answer(t)                  # v10.14: before the rest ('show me the last stop' is no command)
        if rew is not None:
            return rew
        if self._ASK_ETA.search(t):
            eta = self.eta()
            with h.lock:
                m = dict(h.mission or {})
            if eta:
                return eta.replace('Estimated time left for this job:', 'I need') .replace('from my pace so far',
                                                                                          'judging by my pace so far')
            if m.get('awaiting_command') or not m:
                return "I haven't got a job yet, so there's nothing to time. Give me one and ask again after a minute."
            if m.get('done'):
                return 'My job is finished, so no time left.'
            return 'Ask me again in a minute: I need a little of the job behind me to judge my pace.'
        if self._ASK_SEE.search(t):
            with h.lock:
                g = dict(h.guard or {})
            if not g:
                return "My safety layer isn't reporting just now, so I can't tell you."
            p, v = g.get('people', 0), g.get('vehicles', 0)
            out = (f"My lidar sees {p} {'person' if p == 1 else 'people'}" if p else 'I see nobody near me')
            out += f" and {v} moving vehicle{'s' if v != 1 else ''}" if v else ''
            if g.get('person_m') is not None:
                out += f"; the nearest person is {g['person_m']:.1f} m from my centre"
            out += '. ' + (g.get('say') or 'clear').capitalize() + '.'
            return out + (" I can't see behind the racks: anyone there is unknown to me until they step out, "
                          'and someone who just stepped out of view I remember for a few seconds.')
        if self._ASK_WET.search(t):
            return self.wet_answer()
        if self._ASK_FAST.search(t):
            with h.lock:
                g, v = dict(h.guard or {}), h.speed
            lim = g.get('v_limit')
            return (f"{v:.2f} metres a second right now" + (f", held to {lim:.1f} by my safety layer ({g.get('say')})"
                                                            if lim is not None else '') + '.')
        return None

    def rewind_answer(self, t):
        """v10.14: 'show me your closest call', 'what would have happened?', 'were your stops needed?'."""
        which = ('closest' if self._ASK_CLOSEST.search(t) else 'last' if self._ASK_LAST.search(t)
                 else 'card' if self._ASK_STOPS.search(t) else None)
        if which is None:
            return None
        h = self.hub
        with h.lock:
            rw, now, sim = h.rew, h.rewind_now(), h.sim_t
            if rw is None:
                return "I can't look back at my stops here: rewind.py isn't installed next to me."
            card = rw.scorecard(now)
            vis = rw.visible(now)
            st = None if not vis else (rw.closest(now) if which == 'closest' else vis[-1])
            st = None if st is None else dict(st)
        if which == 'card':
            return card + (" Ask me to show you my closest call." if any(s['cls'] == 'hit' for s in vis) else '')
        if st is None:
            return ("I haven't stopped for anyone yet, so there's nothing to rewind." if not rw.stops else
                    "I'm still watching how my last stop turns out: ask me again in a few seconds.")
        ago = max(0.0, (sim or st['t']) - st['t'])
        when = ('just now' if ago < 20 else f'{ago:.0f} seconds ago' if ago < 90 else
                f'about {ago / 60:.0f} minutes ago')
        where = next((ZONE_SAY[z] for z, (x0, x1, y0, y1) in ZONES.items()
                      if x0 <= st['x'] <= x1 and y0 <= st['y'] <= y1), None)
        head = ('My closest call so far' if which == 'closest' else 'The last time I stopped for someone')
        out = f"{head} was {when}" + (f" in {where}" if where else '') + f". {st.get('text') or ''}"
        rg, rv = st.get('real_gap', 9.0), st.get('real_v') or 0.0
        if st.get('waited', 0) >= 1.0:
            out += f" Instead I waited {st['waited']:.0f} seconds"
            out += ((" and they walked into me; I wasn't moving towards them." if rv < 0.05 else
                     f" but still touched them, moving towards them at {rv:.1f} metres a second.") if rg <= 0.0 else
                    f" and nobody came closer than {rg:.1f} metres." if rg < 8.9 else '.')
        elif rg <= 0.0 and rv >= 0.05:
            out += f" I still touched them, moving towards them at {rv:.1f} metres a second."
        self.rewind_pick = st['id']
        return out + " I'm replaying it in the 3D view."

    def wet_answer(self):
        """v10.13: 'where is the floor wet?', from the wet-floor guardian."""
        h = self.hub
        with h.lock:
            w = h.wet
            if w is None:
                return "I'm not keeping a wet-floor map just now."
            t = h.sim_t or 0.0
            a, pump, why = w.area(t), w.pump, w.why
            wet = w.wetness(t)
            ys, xs = np.nonzero(wet > 0)
            where = None
            if len(xs):
                cx, cy = w.ox + (xs.mean() + 0.5) * 0.1, w.oy + (ys.mean() + 0.5) * 0.1
                where = next((ZONE_SAY[z] for z, (x0, x1, y0, y1) in ZONES.items() if x0 <= cx <= x1 and y0 <= cy <= y1),
                             None)
                left = float(w.t_dry * wet[wet > 0].max())
        if a['wet_m2'] <= 0.0:
            out = 'The floor is dry: nothing I cleaned is still wet.'
        else:
            out = (f"About {a['wet_m2']:.0f} square metres are still wet" + (f', in {where}' if where else '')
                   + f"; the last of it is dry in about {max(1, round(left / 60))} minute{'s' if left >= 90 else ''}.")
        if pump == 'off':
            out += f' My water is off right now: {why}.'
        elif pump == 'on':
            out += ' My water is on.'
        if a['dry_m2'] >= 0.5:
            out += (f" I cleaned {a['dry_m2']:.0f} square metres dry where people walk; that needs a wet pass when "
                    "it's quiet.")
        return out

    def ask(self, system, text):
        system = system + '\n\n' + self.context()
        payload = {'model': self.model_tag or self.model, 'keep_alive': '30m',      # v10.11: stays loaded
                   'messages': [{'role': 'system', 'content': system}, {'role': 'user', 'content': text}],
                   'stream': False, 'options': {'temperature': 0.0}}
        req = urllib.request.Request(f'{self.host}/api/chat', data=json.dumps(payload).encode(),
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.loads(r.read())['message']['content']

    def warm(self):
        """v10.11: load the model now (an empty prompt loads it) and keep it 30 min, so the first question
        doesn't wait the 5-15 s Ollama takes to load llama3.2 (it unloads it after 5 idle minutes)."""
        try:
            req = urllib.request.Request(f'{self.host}/api/generate', data=json.dumps(
                {'model': self.model_tag or self.model, 'prompt': '', 'keep_alive': '30m'}).encode(),
                headers={'Content-Type': 'application/json'})
            t0 = time.time()
            urllib.request.urlopen(req, timeout=120).read()
            log(f'the model is loaded and kept warm ({self.model_tag or self.model}, {time.time() - t0:.1f} s)')
        except Exception as e:
            log(f'could not load the model ahead of the first question: {e}')

    def watch_model(self):
        was_up = False
        while True:
            up = False
            try:
                with urllib.request.urlopen(f'{self.host}/api/tags', timeout=4.0) as r:
                    names = [m['name'] for m in json.loads(r.read()).get('models', [])]
                tags = [n for n in names if n.split(':')[0] == self.model]
                up = bool(tags)
                if up:
                    self.model_tag = next((n for n in tags if n.endswith(':latest')), tags[0])
                why = '' if up else f"Ollama is up but '{self.model}' is not pulled"
            except Exception as e:
                r = getattr(e, 'reason', e)
                why = f'no Ollama at {self.host} (' + (
                    'refused: Ollama listens only on the Mac itself; quit the Ollama app, then '
                    'OLLAMA_HOST=0.0.0.0 ollama serve' if isinstance(r, ConnectionRefusedError) else
                    "timed out: the Mac's firewall, or not the Mac's address" if isinstance(r, (socket.timeout, TimeoutError))
                    else type(r).__name__) + ')'
            with self.hub.lock:
                self.hub.agent.update(model_up=up, model_why=why, host=self.host)
            if up and not was_up:
                threading.Thread(target=self.warm, daemon=True).start()
            was_up = up
            time.sleep(15.0 if up else 6.0)

    def submit(self, text, via, alts=None):
        text = (text or '').strip()[:300]
        if not text:
            return None
        tid = f'w{self.run}-{next(self.ids)}'
        self.seen.add(tid)
        self.alts[tid] = [str(a)[:300] for a in (alts or [])[:5]]
        self.q.put((text, via, tid))
        return tid

    def loop(self):
        while True:
            text, via, tid = self.q.get()
            with self.hub.lock:
                self.hub.agent['busy'] = True
            try:
                self.turn(text, via, tid)
            except Exception as e:
                self.hub.add_event('robot', text=f'Sorry, something went wrong on my side: {e}', id=tid)
            finally:
                with self.hub.lock:
                    self.hub.agent['busy'] = not self.q.empty()

    def zone_check(self, heard):
        """v10.10b: (what to act on, a question to ask instead, its buttons). The answer to the last
        'which zone?' (within 90 s) becomes the command; an order with its zone lost gets the question."""
        pend, self.pending = self.pending, None
        try:
            if self.parse_direct is not None and self.parse_direct(heard) is not None:
                return heard, None, None
        except Exception:
            pass
        if pend and time.time() - pend['t'] < 90.0:
            a = zone_answer(heard, pend.get('guess'))
            if a:
                return a, None, None
        q = zone_question(heard)
        if q:
            self.pending = {'t': time.time(), 'guess': q[1]}
            return heard, q[0], zone_choices(q[1])
        return heard, None, None

    def turn(self, text, via, tid):
        self.rewind_pick = None
        heard = (pick_heard([text] + self.alts.pop(tid, []), self.parse_direct) if via in ('voice', 'whisper')
                 else text)       # v10.11: 'whisper' = heard by ear_server.py on the Mac
        heard, ask, choices = self.zone_check(heard)
        self.hub.add_event('you', text=text, via=via, id=tid, heard=heard if heard != text else None)
        state = self.hub.mission_copy()
        out, err = {}, None
        intro = canned(heard, state)
        if ask is not None:
            out = {'command': None, 'reply': ask, 'route': 'question', 'route_why': 'which zone? (code)',
                   'notes': ['the zone was not clear: asked, no model call'], 'choices': choices}
        elif intro is not None:
            out = {'command': None, 'reply': intro, 'route': 'question', 'route_why': 'introduction (scripted)',
                   'notes': ['scripted introduction, no model call']}
        elif self.graph is not None and self.code_answer(heard) is not None:
            out = {'command': None, 'reply': self.code_answer(heard), 'route': 'question',
                   'route_why': 'answered from the live state (code)', 'notes': ['no model call']}
        elif self.graph is not None:
            try:
                out = self.graph.invoke({'text': heard, 'history': self.history, 'robot': state}) or {}
            except Exception as e:
                err = str(e)
        elif self.code_answer(heard) is not None:
            out = {'command': None, 'reply': self.code_answer(heard), 'route': 'question',
                   'route_why': 'answered from the live state (code)', 'notes': ['no model call']}
        elif self.parse_direct is not None:
            d = self.parse_direct(heard)
            if d is not None:
                out = {'command': {'action': d.action.value, 'zones': [z.value for z in d.zones],
                                   'reason': d.reason or 'web page'},
                       'reply': '', 'route': 'command', 'route_why': 'parse_direct (no agent)',
                       'notes': ['exact command, no model call']}
            else:
                out = {'command': None, 'route': 'none', 'notes': [],
                       'reply': "I can only take plain commands right now (the robot agent isn't loaded), "
                                "for example 'clean aisle 2', 'stop', 'carry on' or 'go home'."}
        else:
            err = 'no agent and no mission_command'
        if err is not None:
            reply = f"Sorry, I couldn't work that out ({err})."
            self.hub.add_event('robot', text=reply, id=tid, error=err)
            self.dialogue({'text': heard, 'state': state, 'error': err, 'via': f'web: {via}', 'web_id': tid})
            return
        cmd = out.get('command')
        reply = (out.get('reply') or '').strip()
        notes = out.get('notes') or []
        if cmd is None and reply.startswith('model unreachable'):
            log(f'the model did not answer: {reply[:160]}  (Ollama at {self.host}, model {self.model})')
            notes = notes + [reply]
            with self.hub.lock:
                mw = self.hub.agent.get('model_why') or reply
            reply = ("My language model on the Mac isn't answering, so I can't take that one. "
                     + brief(state) + " Plain commands still work: clean aisle 2, stop, carry on, go home. "
                     f"(Model: {mw}.)")
        steps0 = (cmd.get('steps') or [cmd]) if cmd is not None else []
        if (cmd is not None and any(str(n).startswith('planned ') for n in notes)
                and any(st.get('action') == 'clean' for st in steps0) and not _ZONE_SAID.search(heard)):
            notes = notes + [f'the model planned {steps0} for words that name no zone: not sent, asked']
            cmd, reply, out['choices'] = None, ASK_ZONE, zone_choices()
            self.pending = {'t': time.time(), 'guess': None}
        elif cmd is None and _JSONISH.search(reply):
            notes = notes + ['the model answered in broken JSON: not shown, nothing sent']
            reply, out['choices'] = "Sorry, I got muddled and did nothing. " + ASK_ZONE, zone_choices()
            self.pending = {'t': time.time(), 'guess': None}
        sent = None
        if cmd is not None:
            sent = self.send(cmd)
            steps = cmd.get('steps') or [cmd]
            if self.confirm is not None and (not reply or any('exact command' in n for n in notes)):
                reply = self.confirm(steps, state)
            elif not reply:
                reply = 'OK: ' + ', then '.join(
                    f"{st.get('action')} {' '.join(ZONE_SAY.get(z, z) for z in st.get('zones') or [])}".strip()
                    for st in steps) + '.'
        if not reply:
            reply = 'OK.'
        self.hub.add_event('robot', text=reply, id=tid, sent=cmd, delivered=sent,
                           route=out.get('route'), why=out.get('route_why'), notes=notes[-3:],
                           **({'choices': out['choices']} if out.get('choices') else {}))
        if getattr(self, 'rewind_pick', None) is not None and out.get('route') == 'question':
            self.hub.add_event('rewind', id=self.rewind_pick)          # v10.14: the 3D view acts it out
        self.rewind_pick = None
        self.dialogue({'text': heard, 'route': out.get('route'), 'route_why': out.get('route_why'),
                       'notes': notes, 'reply': reply, 'sent': cmd, 'state': state,
                       'via': f'web: {via}', 'web_id': tid, 'heard_raw': text if heard != text else None})
        self.history.append(('user', heard))
        self.history.append(('assistant', reply))
        self.history = self.history[-8:]

    def stop_now(self):
        """The STOP button: the emergency stop at the gate, and the job paused. No agent, no queue."""
        cmd = {'action': 'stop', 'zones': [], 'reason': 'STOP button on the web page'}
        tid = f'w{self.run}-{next(self.ids)}'
        self.seen.add(tid)
        self.hub.set_estop(True, 'this page')
        gate = self.estop(True)                       # first: the gate holds the wheels at zero
        self.hub.add_event('you', text='STOP', via='button', id=tid)
        sent = self.send(cmd)                         # then: the job pauses, its goal is cancelled
        if self.replay:
            reply = "Emergency stop. (This is a replay, so nothing was sent to a robot.)"
        elif gate or sent:
            reply = ("Emergency stop: I'm held at zero speed" + (" by my safety gate" if gate else "")
                     + (", and my job is paused" if sent else "") + ". Release the stop, then say "
                     "'carry on' when you want me to continue.")
        else:
            reply = "That didn't reach the robot: nothing is listening on /emergency_stop or /mission_command."
        self.hub.add_event('robot', text=reply, id=tid, sent=cmd, delivered=bool(gate or sent), route='command',
                           why='STOP button', estop=True)
        self.dialogue({'text': 'STOP (button)', 'route': 'command', 'route_why': 'STOP button: /emergency_stop '
                       'true and a stop', 'notes': [], 'reply': reply, 'sent': cmd,
                       'state': self.hub.mission_copy(), 'via': 'web: button', 'web_id': tid})
        return bool(gate or sent)

    def release(self):
        """Release the emergency stop. The job stays paused until 'carry on'."""
        tid = f'w{self.run}-{next(self.ids)}'
        self.seen.add(tid)
        self.hub.set_estop(False, 'this page')
        gate = self.estop(False)
        self.hub.add_event('you', text='Release the emergency stop', via='button', id=tid)
        st = self.hub.mission_copy()
        reply = ("Released: my safety gate lets commands through again. "
                 + ("I'm still paused; say 'carry on' to continue." if st.get('paused') else "")
                 if gate or self.replay else "The release didn't reach the robot (nothing on /emergency_stop).")
        self.hub.add_event('robot', text=reply.strip(), id=tid, route='command', why='release')
        self.dialogue({'text': 'release the emergency stop (button)', 'route': 'command',
                       'route_why': '/emergency_stop false', 'notes': [], 'reply': reply, 'sent': None,
                       'state': st, 'via': 'web: button', 'web_id': tid})
        return bool(gate)

    def from_dialogue(self, rec):
        """A /dialogue record: the talk tab's turns are shown; this page's own are already there."""
        if not isinstance(rec, dict) or rec.get('web_id') in self.seen:
            return
        via = 'web' if str(rec.get('via') or '').startswith('web') else 'terminal'
        if rec.get('text'):
            self.hub.add_event('you', text=str(rec.get('text')), via=via)
        reply = rec.get('reply') or (f"(error: {rec.get('error')})" if rec.get('error') else '')
        if reply:
            self.hub.add_event('robot', text=str(reply), sent=rec.get('sent'), via=via,
                               route=rec.get('route'))


# ------------------------------------------------------------------ live: ROS 2
def run_ros(hub, args):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import OccupancyGrid, Path
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import MarkerArray
    try:
        from sensor_msgs.msg import Image
    except ImportError:
        Image = None
    sys.path.insert(0, HERE)
    try:
        from narrator import Narrator
    except Exception as e:
        Narrator = None
        log(f'no narrator ({e}): the "robot says" panel stays empty')

    class WebNode(Node):
        def __init__(self):
            super().__init__('robot_web', parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.buf = Buffer()
            self.tfl = TransformListener(self.buf, self)
            self.narrator = Narrator() if Narrator else None
            self.cmd_pub = self.create_publisher(String, '/mission_command', 10)
            self.dlg_pub = self.create_publisher(String, '/dialogue', 10)
            self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
            self.wet_pub = self.create_publisher(String, '/wet_floor/state', 10)   # v10.13: for the record (1 Hz)
            self.rew_pub = self.create_publisher(String, '/rewind/stop', 10)      # v10.14: each stop's verdict
            self.wet_n = 0
            self.create_subscription(Bool, '/emergency_stop', self.on_estop, 10)
            self.estop_wall = 0.0
            self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
            self.create_subscription(OccupancyGrid, '/coverage_passes', self.on_cov, latched)
            self.create_subscription(String, '/mission_state', self.on_mission, 10)
            self.create_subscription(String, '/people_guard/state', self.on_guard, 10)
            self.create_subscription(MarkerArray, '/people_markers', self.on_markers, 5)
            self.create_subscription(Path, '/plan', self.on_plan, 1)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(String, '/traffic/truth', self.on_truth, 5)
            self.create_subscription(String, '/dialogue', self.on_dialogue, 10)
            if Image is not None and not args.no_camera:
                self.create_subscription(Image, '/eye/image_annotated', self.on_image, qos_profile_sensor_data)
            self.scan_n = 0
            self.create_timer(0.1, self.tick)
            self.talker = Talker(hub, self.send_command, self.send_dialogue, args.ollama, args.model,
                                 agent=not args.no_agent, estop=self.send_estop)

        def sim_now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def to_map(self, frame):
            try:
                tf = self.buf.lookup_transform('map', frame, Time())
            except Exception:
                return None
            tr = tf.transform
            return tr.translation.x, tr.translation.y, _yaw(tr.rotation)

        def tick(self):
            if hub.estop['on'] and hub.estop['by'] == 'this page' and time.time() - self.estop_wall > 2.0:
                self.send_estop(True, quiet=True)    # held: say so again (a restarted gate hears it)
            t = self.sim_now()
            p = self.to_map('base_link')
            if p is not None:
                hub.set_pose(p[0], p[1], p[2], t)
                hub.wet_update(t)                    # v10.13: the wet-floor guardian
                for st in hub.rewind_update(t):     # v10.14: Rewind: a stop's verdict, for the record
                    self.rew_pub.publish(String(data=json.dumps(
                        {k: v for k, v in st.items() if not k.startswith('_')}, default=str)))
                self.wet_n += 1
                if self.wet_n % 10 == 0 and hub.wet is not None:
                    with hub.lock:
                        d = hub.wet.summary(t)
                    self.wet_pub.publish(String(data=json.dumps(d)))
            else:
                with hub.lock:
                    hub.set_clock(t)

        def on_map(self, msg):
            i = msg.info
            hub.set_map(i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y, msg.data)
            log(f'map {i.width} x {i.height} at {i.resolution:.2f} m')

        def on_cov(self, msg):
            hub.set_coverage(msg.info.width, msg.info.height, msg.data)

        def on_mission(self, msg):
            try:
                s = json.loads(msg.data)
            except Exception:
                return
            hub.set_mission(s)
            if self.narrator is not None:
                # before the first job the robot sits at the dock: its traffic is not news
                fed = dict(s, traffic=None) if s.get('awaiting_command') else s
                for line in self.narrator.feed(fed, self.sim_now()):
                    hub.add_event('say', text=line)

        def on_guard(self, msg):
            try:
                hub.set_guard(json.loads(msg.data))
            except Exception:
                pass

        def on_markers(self, msg):
            hub.set_markers(msg.markers, self.to_map)

        def on_plan(self, msg):
            hub.set_plan([(p.pose.position.x, p.pose.position.y) for p in msg.poses]
                         if (msg.header.frame_id or 'map').lstrip('/') == 'map' else None)

        def on_scan(self, msg):
            self.scan_n += 1
            if self.scan_n % 2:                       # 5 Hz is plenty for the page
                return
            lp = self.to_map(msg.header.frame_id)
            if lp is None:
                return
            r = np.asarray(msg.ranges, dtype=float)
            a = msg.angle_min + msg.angle_increment * np.arange(len(r)) + lp[2]
            ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max * 0.995)
            idx = np.nonzero(ok)[0][::2]
            hub.set_scan(np.column_stack((lp[0] + r[idx] * np.cos(a[idx]), lp[1] + r[idx] * np.sin(a[idx]))),
                         view_polygon(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                                      msg.range_max, lp[0], lp[1], lp[2]))

        def on_truth(self, msg):
            try:
                d = json.loads(msg.data)
                hub.set_truth([{k: a.get(k) for k in ('name', 'kind', 'x', 'y', 'yaw', 'v', 'L', 'W', 'fork', 'state')
                                if k in a} for a in d.get('agents', [])])
            except Exception:
                pass

        def on_dialogue(self, msg):
            with hub.lock:
                hub.seen('dialogue')
            try:
                self.talker.from_dialogue(json.loads(msg.data))
            except Exception:
                pass

        def on_image(self, msg):
            st = msg.header.stamp
            hub.set_camera((st.sec, st.nanosec), msg.height, msg.width, msg.encoding, bytes(msg.data))

        def send_command(self, cmd):
            n = self.count_subscribers('/mission_command')
            with hub.lock:                            # coverage_recorder listens too: is the cleaning alive?
                alive = hub.mission_wall is not None and time.time() - hub.mission_wall < 5.0
            m = String()
            m.data = json.dumps(cmd)
            self.cmd_pub.publish(m)
            log(f'sent /mission_command {m.data}  ({n} listening'
                + ('' if alive else '; but no /mission_state for 5 s: is the cleaning tab running?') + ')')
            return n > 0 and alive

        def send_estop(self, on, quiet=False):
            n = self.count_subscribers('/emergency_stop') - 1   # not counting this node's own
            self.estop_wall = time.time()
            self.estop_pub.publish(Bool(data=bool(on)))
            if not quiet:
                log(f'/emergency_stop {str(bool(on)).lower()}  ({n} listening: guard_gate)')
            return n > 0

        def on_estop(self, msg):
            if hub.set_estop(bool(msg.data), 'someone else' if time.time() - self.estop_wall > 0.5 else 'this page'):
                if time.time() - self.estop_wall > 0.5:
                    hub.add_event('say', text=('Emergency stop pressed (from outside this page).' if msg.data
                                               else 'Emergency stop released (from outside this page).'))

        def send_dialogue(self, rec):
            m = String()
            m.data = json.dumps(rec, default=str)
            self.dlg_pub.publish(m)

    def spin(node):
        try:
            rclpy.spin(node)
        except Exception:                            # shutdown while spinning
            pass

    rclpy.init()
    node = WebNode()
    threading.Thread(target=spin, args=(node,), daemon=True).start()
    return node.talker, lambda: (node.destroy_node(), rclpy.shutdown() if rclpy.ok() else None)


# ------------------------------------------------------------------ replay: a recorded shift
def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def guard_shapes(pg, core, lk, state, robot, t):
    """What people_guard's publish_markers draws, as the page's shapes (replay has no ROS)."""
    out = []

    def add(ns, i, **kw):
        kw.update(ns=ns, id=int(i))
        out.append(kw)

    def rnd(xy):
        return [[round(float(a), 3), round(float(b), 3)] for a, b in xy]
    waited_for = lk.who if state == 'wait' else None
    for tr in core.people:
        stale = tr.lost or (t - tr.t) > pg.STALE
        rgba = ([0.9, 0.1, 0.1, 0.7] if tr.id == waited_for else [0.6, 0.6, 0.6, 0.4] if stale else
                [0.2, 0.5, 1.0, 0.5] if tr.id in core.along else [1.0, 0.55, 0.1, 0.5])
        add('person', tr.id, k='c', x=round(tr.px, 3), y=round(tr.py, 3), r=pg.BODY_R, h=1.7, rgba=rgba)
        for k, f in enumerate(lk.futs.get(tr.id, [])):
            if len(f) > 1 and np.ptp(f[:26, 0]) + np.ptp(f[:26, 1]) > 0.05:
                add('predicted_path', tr.id * 10 + k, k='l', pts=rnd(f[:26]), w=0.05,
                    rgba=[1.0, 0.25, 0.25, 0.9])
        if tr.id in core.crossers and tr.line is not None and len(tr.raw) > 2:
            cx, cy, ux, uy = tr.line
            s = [(x - cx) * ux + (y - cy) * uy for _, x, y in tr.raw]
            a, b = min(s), max(s)
            add('walkway', tr.id, k='l', pts=rnd([(cx + ux * a, cy + uy * a), (cx + ux * b, cy + uy * b)]),
                w=0.5, rgba=[1.0, 0.55, 0.1, 0.35])
        add('label', tr.id, k='t', x=round(tr.px, 3), y=round(tr.py, 3), h=0.28, rgba=[1, 1, 1, 1],
            text=(f'person {tr.id}  {tr.speed:.1f} m/s' + ('  (along the path)' if tr.id in core.along else '')
                  + ('  (not seen just now)' if stale else '') + ('  (robot waits)' if tr.id == waited_for else '')))
    z = lk.zone
    if z is not None and lk.path_xy is not None and len(lk.path_xy) > 1:
        P = lk.path_xy
        i0 = int(round(z.s_hold / 0.05))
        i1 = min(int(round(z.s_out / 0.05)), len(P) - 1)
        if i1 > i0:
            add('crossing_zone', 0, k='l', pts=rnd(P[i0:i1 + 1]), w=0.7, rgba=[1.0, 0.75, 0.0, 0.25])
        k = min(max(i0, 0), len(P) - 2)
        dx, dy = P[k + 1, 0] - P[k, 0], P[k + 1, 1] - P[k, 1]
        L = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / L * 0.45, dx / L * 0.45
        add('stop_line', 0, k='l', pts=rnd([(P[k, 0] - nx, P[k, 1] - ny), (P[k, 0] + nx, P[k, 1] + ny)]),
            w=0.08, rgba=[1.0, 0.15, 0.15, 1.0] if state == 'wait' else [1.0, 1.0, 1.0, 0.9])
    if lk.go_path is not None:
        add('robot_path', 0, k='l', pts=rnd(lk.go_path[:31]), w=0.04, rgba=[0.2, 0.9, 0.3, 0.8])
    for v in core.vehicles:
        if not (v.moving or v.recently_moved(t)):
            continue
        (x0, y0), (x1, y1) = v.pts.min(axis=0), v.pts.max(axis=0)
        hold = core.veh_hold and v.id in (core.vlook.who, core.vlook.keep_who)
        add('vehicle', v.id, k='b', x=round(float(x0 + x1) / 2, 3), y=round(float(y0 + y1) / 2, 3),
            w=round(max(float(x1 - x0), 0.2), 3), l=round(max(float(y1 - y0), 0.2), 3), yaw=0.0,
            rgba=[1.0, 0.2, 0.1, 0.55] if hold else [1.0, 0.6, 0.0, 0.45])
        if v.speed > pg.VEH_STOPPED:
            u = np.array([v.vx, v.vy]) / v.speed
            add('vehicle_path', v.id, k='l', w=0.06, rgba=[1.0, 0.45, 0.0, 0.9],
                pts=rnd([(v.cx, v.cy), (v.cx + v.vx * pg.VEH_HORIZON + u[0] * pg.VEH_LEAD,
                                        v.cy + v.vy * pg.VEH_HORIZON + u[1] * pg.VEH_LEAD)]))
        add('vehicle_label', v.id, k='t', x=round(float(v.cx), 3), y=round(float(v.cy), 3), h=0.28,
            rgba=[1, 1, 1, 1], text=(f'VEHICLE {v.speed:.1f} m/s' if v.moving else 'VEHICLE standing (forks?)')
            + ('  (giving way)' if hold else ''))
    for k, (sx, sy, sr, sid) in enumerate(core.standing):
        add('personal_space', k, k='c', x=round(sx, 3), y=round(sy, 3), r=round(max(sr, pg.PERSONAL_MIN), 3),
            h=0.03, rgba=[0.15, 0.85, 0.35, 0.35])
        add('standing', k, k='c', x=round(sx, 3), y=round(sy, 3), r=pg.BODY_R, h=1.7, rgba=[0.15, 0.85, 0.35, 0.6])
    if state == 'wait':
        why = core.guard.why
        text = ('WAIT: someone right in front' if getattr(core, 'touch_wait', False) else
                'EMERGENCY STOP' if why == 'emergency' else
                'WAIT: setting off would close in on someone' if why == 'start' else
                ('WAIT: vehicle standing ahead' if core.vlook.keep_out else 'WAIT: giving way to a vehicle')
                if core.veh_hold else
                f'WAIT at stop line (person {lk.who})' if z is not None else f'WAIT for person {lk.who}')
        rgba = [1.0, 0.2, 0.2, 1.0]
    else:
        why = core.guard.why
        text = {'approach': 'GO (to stop line)', 'in_way': 'GO (clearing)', 'clearing': 'GO (leaving walkway)',
                'crossing': 'CROSSING', 'committed': 'CROSSING (committed)'}.get(why, 'GO')
        rgba = [0.2, 1.0, 0.3, 1.0]
    add('robot_decision', 0, k='t', x=round(robot[0], 3), y=round(robot[1], 3), h=0.35, rgba=rgba, text=text)
    return out


class Replay:
    """A recorded shift (~/runs layout: rec/<L>.npz, traffic/<L>.csv, tour/<L>.csv, cov_<L>/events.jsonl,
    logs/say_<L>.log) played back through the real people guard (open loop, as decide_why does: the
    robot moves as it did), at the speed asked. Seeking starts the guard 20 s before the moment."""

    def __init__(self, hub, runs, label, speed=1.0, t_from=None, guard=None, loop=True):
        self.hub, self.label, self.loop = hub, label, loop
        runs = os.path.expanduser(runs)
        R = np.load(os.path.join(runs, 'rec', f'{label}.npz'), allow_pickle=False)
        self.meta = json.loads(str(R['meta']))
        mi = json.loads(str(R['map_info']))
        self.mi, self.map_data = mi, R['map_data']
        self.T, self.RG, self.LS, self.RB = R['t'], R['ranges'], R['laser'], R['robot']
        self.OV, self.OA, self.WALL = R['odom_v'], R['odom_age'], R['wall']
        off = np.concatenate(([0], np.cumsum(R['plan_n'])))
        self.plans = [R['plan_xy'][off[k]:off[k + 1]] for k in range(len(R['plan_t']))]
        self.plan_ts = np.asarray(R['plan_t'], dtype=float)
        self.pg = load_module(guard or os.path.join(HERE, 'people_guard.py'), 'pg_replay')
        self.truth_ts, self.truth = self.load_truth(os.path.join(runs, 'traffic', f'{label}.csv'))
        self.tour_ts, self.tour = self.load_tour(os.path.join(runs, 'tour', f'{label}.csv'))
        self.load_events(os.path.join(runs, f'cov_{label}', 'events.jsonl'))
        self.says = self.load_says(os.path.join(runs, 'logs', f'say_{label}.log'))
        self.speed = float(speed)
        self.paused = False
        self.seek_to = float(t_from) if t_from is not None else None
        self.t0, self.t1 = float(self.T[0]), float(self.T[-1])
        RW = rewind_module()                     # v10.14: every stop of the recorded shift, against the truth
        if RW is not None:
            rw = RW.Rewind(self.pg)
            try:
                n = rw.from_record(os.path.join(runs, 'guard', f'{label}.csv'), self.plan_at, self.truth_ts,
                                   self.truth, world_worker, self.T, self.RB)
                hub.rew = rw
                log(f'rewind: {n} stops for people in {label}')
            except Exception as e:
                log(f'rewind: could not read the stops ({type(e).__name__}: {e})')
        hub.set_map(mi['width'], mi['height'], mi['resolution'], mi['origin_x'], mi['origin_y'], self.map_data)
        log(f'replay {label}: {len(self.T)} scans, sim {self.t0:.1f}-{self.t1:.1f} s, guard {self.pg.VERSION} '
            f'(recorded with {self.meta.get("guard")}), {len(self.truth)} truth frames, {len(self.says)} lines said')

    # -------- the recording
    @staticmethod
    def load_truth(path):
        rows = collections.OrderedDict()
        try:
            for r in csv.DictReader(open(path)):
                try:
                    t = float(r['t'])
                    a = {'name': r['name'], 'kind': r['kind'], 'x': float(r['x']), 'y': float(r['y']),
                         'yaw': float(r['yaw']), 'v': float(r['v']), 'state': r.get('state') or ''}
                except (TypeError, ValueError, KeyError):
                    continue
                rows.setdefault(t, []).append(a)
        except OSError:
            pass
        return np.array(list(rows.keys()), dtype=float), list(rows.values())

    @staticmethod
    def load_tour(path):
        ts, rows = [], []
        try:
            for r in csv.DictReader(open(path)):
                try:
                    ts.append(float(r['t']))
                    rows.append((r.get('zone') or '', float(r.get('coverage_pct') or 0.0),
                                 r.get('mission_done') == '1'))
                except (TypeError, ValueError, KeyError):
                    continue
        except OSError:
            pass
        return np.array(ts, dtype=float), rows

    def load_events(self, path):
        self.states, self.commands = [], []
        try:
            for line in open(path):
                try:
                    e = json.loads(line)
                    t = float(np.interp(float(e['t']), self.WALL, self.T))
                except (ValueError, KeyError, TypeError):
                    continue
                if e.get('type') == 'state':
                    self.states.append((t, e['data']))
                elif e.get('type') == 'command':
                    self.commands.append((t, e['data']))
        except OSError:
            pass
        self.state_ts = np.array([x[0] for x in self.states], dtype=float)
        self.cmd_ts = np.array([x[0] for x in self.commands], dtype=float)

    @staticmethod
    def load_says(path):
        out = []
        try:
            for line in open(path, errors='replace'):
                m = re.match(r'^\[\s*([0-9.]+)\] robot: (.*)$', line.rstrip())
                if m:
                    out.append((float(m.group(1)), m.group(2)))
        except OSError:
            pass
        return out

    # -------- control from the page
    def control(self, c):
        cmd = c.get('cmd')
        try:
            if cmd == 'pause':
                self.paused = True
            elif cmd == 'play':
                self.paused = False
            elif cmd == 'speed':
                self.speed = float(min(max(float(c.get('value', 1.0)), 0.25), 8.0))
            elif cmd == 'seek':
                self.seek_to = float(min(max(float(c.get('value', self.t0)), self.t0), self.t1 - 1.0))
        except (TypeError, ValueError):
            pass
        self.publish_status(self.now)

    now = None

    def publish_status(self, t):
        with self.hub.lock:
            self.hub.replay = {'label': self.label, 't0': round(self.t0, 1), 't1': round(self.t1, 1),
                               't': None if t is None else round(t, 1), 'speed': self.speed,
                               'paused': self.paused, 'guard': self.pg.VERSION,
                               'recorded_with': self.meta.get('guard')}

    # -------- the run
    def start(self):
        threading.Thread(target=self.run, daemon=True).start()

    def new_core(self):
        pg, mi = self.pg, self.mi
        core = pg.GuardCore(pg.StaticMap(mi['width'], mi['height'], mi['resolution'], mi['origin_x'],
                                         mi['origin_y'], self.map_data))
        core.guard.prog_time = 25.0
        return core

    @staticmethod
    def at(ts, t):
        """index of the last time <= t, or -1"""
        return int(np.searchsorted(ts, t, side='right')) - 1 if len(ts) else -1

    def plan_at(self, t):
        i = self.at(self.plan_ts, t)
        if i >= 0 and t - self.plan_ts[i] <= self.pg.PLAN_AGE:
            return [tuple(p) for p in self.plans[i]]
        return None

    def mission_at(self, t, guard, for_s, holds, held):
        i = self.at(self.state_ts, t)
        st = self.states[i][1] if i >= 0 else {}
        j = self.at(self.tour_ts, t)
        tour = self.tour[j] if j >= 0 else None
        n_cmd = self.at(self.cmd_ts, t) + 1
        zone = st.get('zone', 'start')
        s = dict(st)
        s.update(goal=st.get('goal', 0), total=st.get('total', 0), zone=zone,
                 coverage_pct=tour[1] if tour else 0.0, awaiting_command=n_cmd == 0 and zone not in ZONES,
                 asked=[f"{c.get('action')} {', '.join(c.get('zones') or [])}".strip()
                        for _, c in self.commands[:n_cmd]],
                 guard=True, holds=holds, held_s=round(held, 1), x=None, y=None)
        if tour and tour[2]:
            s['done'] = True
        if guard is not None:
            s['traffic'] = {'state': guard['state'], 'why': guard['why'], 'say': guard['say'],
                            'v_limit': guard['v_limit'], 'limit': guard['limit'], 'person_m': guard['person_m'],
                            'vehicle_m': guard['vehicle_m'], 'for_s': round(for_s, 1)}
        return s

    def run(self):
        hub, pg = self.hub, self.pg
        amin, ainc = float(self.meta['angle_min']), float(self.meta['angle_increment'])
        rmin, rmax = float(self.meta['range_min']), float(self.meta['range_max'])
        info = {'w': self.mi['width'], 'h': self.mi['height'], 'res': self.mi['resolution'],
                'ox': self.mi['origin_x'], 'oy': self.mi['origin_y']}
        while True:
            fast_until, self.seek_to = self.seek_to, None
            with hub.lock:
                hub.reset()
                hub.painter = Painter(info)
            hub.add_event('replay', text=f'replay of {self.label}' +
                          (f' from {fast_until:.0f} s' if fast_until else ''))
            core = self.new_core()
            prev, v_est = None, 0.0
            k_say = 0
            rw = hub.rew
            k_stop = 0                                  # v10.14: the next recorded stop to announce
            wait_since, n_holds, held = None, 0, 0.0
            wall0 = sim0 = None
            for k in range(len(self.T)):
                if self.seek_to is not None:
                    break                                       # start again, fast to the new time
                t = float(self.T[k])
                self.now = t
                if fast_until is not None and t >= fast_until:
                    fast_until = None
                live = fast_until is None                       # shown as it happens (not seeking)
                warm = live or t >= fast_until - 20.0           # the guard runs (tracks need history)
                if live:
                    while self.paused and self.seek_to is None:
                        self.publish_status(t)
                        j = self.at(self.truth_ts, t)          # v10.12: the truth layer stays on while paused
                        if j >= 0 and t - self.truth_ts[j] < 1.0:
                            hub.set_truth(self.truth[j])
                        time.sleep(0.1)
                        wall0 = None
                    if self.seek_to is not None:
                        break
                    if wall0 is None:
                        wall0, sim0 = time.time(), t
                    lag = sim0 + (time.time() - wall0) * self.speed - t
                    if lag < 0:
                        time.sleep(min(-lag / self.speed, 1.0))
                    elif lag > 2.0:                             # fell behind: catch up, no pile-up
                        wall0, sim0 = time.time(), t
                robot, laser = tuple(float(v) for v in self.RB[k]), tuple(float(v) for v in self.LS[k])
                hub.set_pose(robot[0], robot[1], robot[2], t)
                with hub.lock:
                    hub.rtf = self.speed
                if self.OA[k] < 0.5:
                    v_est = float(self.OV[k])
                elif prev is not None and t > prev[0]:
                    v_est = 0.5 * v_est + 0.5 * math.hypot(robot[0] - prev[1], robot[1] - prev[2]) / (t - prev[0])
                prev = (t, robot[0], robot[1])
                guard = None
                shapes, view = [], None
                if warm:
                    plan = self.plan_at(t)
                    core.scan(self.RG[k], amin, ainc, rmin, rmax, laser, t, robot, plan)
                    state, lk, patient = core.decide(robot[:2], robot[2], v_est, 0.8, t, plan)
                    why = ('touch' if getattr(core, 'touch_wait', False) else
                           'vehicle' if state == 'wait' and getattr(core, 'veh_hold', False) else core.guard.why)
                    guard = pg.status(core, state, why, lk, robot, 0.8, t)
                    shapes = guard_shapes(pg, core, lk, state, robot, t)
                    view = view_polygon(self.RG[k], amin, ainc, rmin, rmax, *laser)
                    if live:
                        hub.set_shapes(shapes)
                        hub.set_plan(plan)
                        r = np.asarray(self.RG[k], dtype=float)
                        a = amin + ainc * np.arange(len(r)) + laser[2]
                        ok = np.isfinite(r) & (r > rmin) & (r < rmax * 0.995)
                        idx = np.nonzero(ok)[0][::2]
                        hub.set_scan(np.column_stack((laser[0] + r[idx] * np.cos(a[idx]),
                                                      laser[1] + r[idx] * np.sin(a[idx]))), view)
                for_s = 0.0
                if guard is not None:
                    busy = self.at(self.cmd_ts, t) >= 0
                    if guard['state'] == 'wait' and busy:
                        if wait_since is None:
                            wait_since, n_holds = t, n_holds + 1
                        for_s = t - wait_since
                    elif wait_since is not None:
                        held += t - wait_since
                        wait_since = None
                mission = self.mission_at(t, guard, for_s, n_holds,
                                          held + (t - wait_since if wait_since is not None else 0.0))
                mission['x'], mission['y'] = round(robot[0], 2), round(robot[1], 2)
                hub.set_mission(mission)
                if guard is not None:
                    hub.set_guard(guard)
                hub.wet_update(t, mission, shapes, view)      # v10.13: the wet-floor guardian, every scan
                j = self.at(self.truth_ts, t)
                if j >= 0 and t - self.truth_ts[j] < 1.0:
                    hub.set_truth(self.truth[j])
                while k_say < len(self.says) and self.says[k_say][0] <= t:
                    if live:
                        hub.add_event('say', text=self.says[k_say][1])
                    k_say += 1
                while rw is not None and k_stop < len(rw.stops) and rw.stops[k_stop]['t'] + rw.horizon <= t:
                    if live and rw.stops[k_stop].get('cls'):
                        hub.rewind_announce(rw.stops[k_stop])
                    k_stop += 1
                if live and k % 5 == 0:
                    self.publish_status(t)
            else:
                hub.add_event('replay', text=f'end of {self.label}' + (': starting again' if self.loop else ''))
                if not self.loop:
                    self.paused = True
                    while self.seek_to is None:
                        self.publish_status(self.t1)
                        time.sleep(0.2)
                else:
                    for _ in range(30):
                        if self.seek_to is not None:
                            break
                        time.sleep(0.1)


# ------------------------------------------------------------------ the web side
class Web(BaseHTTPRequestHandler):
    server_version = 'robot_web/' + VERSION
    hub = None
    talker = None
    replay = None

    def log_message(self, fmt, *a):
        pass

    def reply(self, code, body, ctype='application/json', gz=True, extra=None, cache='no-store'):
        if isinstance(body, str):
            body = body.encode()
        if gz and len(body) > 1400 and 'gzip' in (self.headers.get('Accept-Encoding') or ''):
            body = gzip.compress(body, 5)
            enc = 'gzip'
        else:
            enc = None
        try:
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', cache)
            if enc:
                self.send_header('Content-Encoding', enc)
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def body_json(self):
        n = int(self.headers.get('Content-Length') or 0)
        if n <= 0 or n > 20000:
            return {}
        try:
            return json.loads(self.rfile.read(n))
        except ValueError:
            return {}

    def do_GET(self):
        u = urllib.parse.urlsplit(self.path)
        q = urllib.parse.parse_qs(u.query)

        def qi(k, d=0):
            try:
                return int(q.get(k, [d])[0])
            except ValueError:
                return d
        if u.path in ('/', '/index.html'):
            try:
                self.reply(200, open(PAGE, 'rb').read(), 'text/html; charset=utf-8')
            except OSError:
                self.reply(500, f'robot_web.html is missing next to {__file__}', 'text/plain')
        elif u.path in ('/twin', '/twin.html', '/3d'):
            try:
                self.reply(200, open(TWIN, 'rb').read(), 'text/html; charset=utf-8')
            except OSError:
                self.reply(404, f'robot_twin.html is missing next to {__file__}: run the v10.12 installer',
                           'text/plain')
        elif u.path == '/vendor/three.module.min.js':
            try:
                src = open(THREE_JS, 'rb').read()
            except OSError:      # the page then loads it from the CDN
                return self.reply(404, 'three.module.min.js is not in tools/vendor', 'text/plain')
            self.reply(200, src, 'text/javascript; charset=utf-8', cache='public, max-age=86400')
        elif u.path == '/api/state':
            self.reply(200, json.dumps(self.hub.state(qi('since'), qi('trail'), qi('epoch')),
                                       separators=(',', ':')))
        elif u.path == '/api/map':
            b = self.hub.map_json()
            self.reply(200, b) if b else self.reply(204, b'')
        elif u.path == '/api/coverage':
            b = self.hub.coverage_json()
            self.reply(200, b) if b else self.reply(204, b'')
        elif u.path == '/api/wet':
            b = self.hub.wet_json()
            self.reply(200, b) if b else self.reply(204, b'')
        elif u.path == '/api/stops':
            b = self.hub.rewind_list()
            self.reply(200, b) if b else self.reply(204, b'')
        elif u.path == '/api/stop':
            b = self.hub.rewind_detail(qi('id', -1))
            self.reply(200, b) if b else self.reply(404, b'{"error": "no such stop (yet)"}')
        elif u.path == '/api/camera.jpg':
            b = self.hub.camera_jpeg()
            self.reply(200, b, 'image/jpeg', gz=False) if b else self.reply(204, b'')
        elif u.path == '/api/health':
            self.reply(200, json.dumps({'ok': True, 'version': VERSION, 'mode': self.hub.mode}))
        elif u.path == '/ear.py':
            try:
                src = open(EAR).read()
            except OSError:
                return self.reply(404, 'ear_server.py is missing', 'text/plain')
            self.reply(200, src, 'text/x-python; charset=utf-8', gz=False,
                       extra={'Content-Disposition': 'attachment; filename="ear_server.py"'})
        elif u.path == '/mac.py':
            host = (self.headers.get('Host') or '').split(':')[0] or vm_address()
            try:
                src = open(MAC).read().replace("VM = '192.168.64.4'", f"VM = '{host}'")
            except OSError:
                return self.reply(404, 'mac_web.py is missing', 'text/plain')
            self.reply(200, src, 'text/x-python; charset=utf-8', gz=False,
                       extra={'Content-Disposition': 'attachment; filename="mac_web.py"'})
        else:
            self.reply(404, 'not found', 'text/plain')

    def do_POST(self):
        u = urllib.parse.urlsplit(self.path)
        d = self.body_json()
        if u.path == '/api/talk':
            alts = d.get('alts') if isinstance(d.get('alts'), list) else None
            tid = self.talker.submit(str(d.get('text') or ''), str(d.get('via') or 'typed')[:12], alts)
            self.reply(200, json.dumps({'ok': tid is not None, 'id': tid}))
        elif u.path == '/api/stop':
            ok = self.talker.stop_now()
            self.reply(200, json.dumps({'ok': bool(ok)}))
        elif u.path == '/api/release':
            ok = self.talker.release()
            self.reply(200, json.dumps({'ok': bool(ok)}))
        elif u.path == '/api/replay' and self.replay is not None:
            self.replay.control(d)
            self.reply(200, json.dumps({'ok': True}))
        else:
            self.reply(404, json.dumps({'ok': False}))


def vm_address():
    try:
        out = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=3).stdout.split()
        out = [a for a in out if '.' in a and not a.startswith('127.')]
        if out:
            return out[0]
    except Exception:
        pass
    return 'localhost'


def addresses():
    try:
        out = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=3).stdout.split()
        return [a for a in out if '.' in a and not a.startswith('127.')] or ['localhost']
    except Exception:
        return ['localhost']


class ReplayTalker(Talker):
    """Replay: the agent answers from the recorded state; commands are shown, not sent."""
    replay = True

    def __init__(self, hub, args):
        super().__init__(hub, self.no_send, lambda rec: None, args.ollama, args.model, agent=not args.no_agent,
                         planner_dir=args.planner)

    def no_send(self, cmd):
        log(f'(replay) would send /mission_command {json.dumps(cmd)}')
        return False


def main(argv=None):
    ap = argparse.ArgumentParser(description='Mission Control: the robot live in a web page, with a talk button.')
    ap.add_argument('--port', type=int, default=PORT)
    ap.add_argument('--replay', nargs=2, metavar=('RUNS', 'LABEL'),
                    help='play a recorded shift (e.g. ~/runs shift_v108a) instead of the live robot; no ROS')
    ap.add_argument('--speed', type=float, default=1.0, help='replay speed (x real time)')
    ap.add_argument('--from', dest='t_from', type=float, default=None, help='replay from this sim time (s)')
    ap.add_argument('--once', action='store_true', help='replay once and hold at the end (default: loop)')
    ap.add_argument('--guard', default=None, help='replay: the people_guard.py to re-run (default: next to this file)')
    ap.add_argument('--planner', default=None,
                    help='a folder holding the warehouse_planner package, if it is not on the path')
    ap.add_argument('--ollama', default=default_ollama(), help='Ollama on the Mac (default: the VM gateway :11434)')
    ap.add_argument('--model', default='llama3.2')
    ap.add_argument('--no-agent', action='store_true', help='commands only, no robot agent / model')
    ap.add_argument('--no-camera', action='store_true', help='leave the camera off the page')
    args = ap.parse_args(argv)

    hub = Hub('replay' if args.replay else 'live')
    replay, shutdown = None, None
    if args.replay:
        replay = Replay(hub, args.replay[0], args.replay[1], speed=args.speed, t_from=args.t_from,
                        guard=args.guard, loop=not args.once)
        talker = ReplayTalker(hub, args)
        replay.publish_status(None)
        replay.start()
    else:
        if args.planner:
            sys.path.insert(0, args.planner)
        talker, shutdown = run_ros(hub, args)

    Web.hub, Web.talker, Web.replay = hub, talker, replay
    ThreadingHTTPServer.allow_reuse_address = True
    ThreadingHTTPServer.daemon_threads = True
    try:
        srv = ThreadingHTTPServer(('0.0.0.0', args.port), Web)
    except OSError as e:
        print(f'robot_web: port {args.port} is taken ({e}). Is it running already (another web tab)? '
              f'Stop that one, or use --port.', flush=True)
        if shutdown:
            shutdown()
        return 1
    print(f'robot_web {VERSION} ({hub.mode}{": " + args.replay[1] if args.replay else ""})', flush=True)
    for a in addresses():
        print(f'  on the Mac, open:   http://{a}:{args.port}     (3D view: /twin)', flush=True)
    print(f'  for the microphone: on the Mac   curl -o mac_web.py http://{addresses()[0]}:{args.port}/mac.py '
          f'&& python3 mac_web.py', flush=True)
    print(f'                      then open    http://localhost:{args.port}', flush=True)
    print(f'  talk: robot agent {"on" if hub.agent.get("ok") else "OFF (" + str(hub.agent.get("why")) + ")"}, '
          f'model {args.model} at {args.ollama}', flush=True)
    try:
        srv.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        if shutdown:
            try:
                shutdown()
            except Exception:
                pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
