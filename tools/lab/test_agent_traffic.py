"""robot_agent v10.4 (+ v10.5, v10.6): 'why did you stop?' answered from /mission_state's traffic fields
by code (the model is never called for it); the FACTS sheet carries the traffic lines.
v10.5: the lanes (swept % per aisle, the patch pass, blockages gone round) in the FACTS, and
"why did you stop" when something stands in the lane or the robot is going round it.

    python3 test_agent_traffic.py <folder holding warehouse_planner/robot_agent.py>
"""
import sys

sys.path.insert(0, sys.argv[1])
from warehouse_planner.robot_agent import build_graph, describe, explain_stop  # noqa: E402

FAILS = []
CALLS = []


def check(name, ok, detail=''):
    print(f'{"ok  " if ok else "FAIL"} {name}{"  (" + str(detail) + ")" if detail != "" else ""}')
    if not ok:
        FAILS.append(name)


def llm(system, text):
    CALLS.append(text)
    return '{"plan": [], "say": "model answer"}'


g = build_graph(llm)
base = {'zone': 'aisle_2', 'goal': 14, 'total': 23, 'coverage_pct': 21.0, 'paused': False,
        'done': False, 'cleaned': ['aisle_1'], 'queue': ['aisle_2'], 'partial': {'aisle_2': 40},
        'rescuing': False, 'blocked': [], 'asked': ['clean aisle_1, aisle_2'], 'x': 2.6, 'y': 9.0,
        'guard': True, 'holds': 7, 'held_s': 63.2, 'waiting_s': 6.0, 'deferred': [],
        'revisiting': False, 'revisited': 0, 'skipped': [], 'last_hold': None}
wait = dict(base, traffic={'state': 'wait', 'why': 'vehicle', 'say': 'giving way to a vehicle: going on '
                           'would pass 0.4 m from it', 'v_limit': 0.15, 'limit': None,
                           'person_m': 3.1, 'vehicle_m': 1.6, 'for_s': 6.2})


def ask(text, robot):
    CALLS.clear()
    out = g.invoke({'text': text, 'history': [], 'robot': robot})
    return out.get('reply', ''), out.get('command'), list(CALLS)


r, c, calls = ask('why did you stop?', wait)
check('waiting for a vehicle: the guard\'s reason, from state', r.startswith("I'm giving way to a vehicle")
      and '6 seconds' in r and not calls and c is None, r)
r, c, calls = ask('what are you waiting for', wait)
check('"what are you waiting for" too', r.startswith("I'm giving way") and not calls, r)
emerg = dict(base, traffic=dict(wait['traffic'], why='emergency', say='stopped: going on would pass 0.2 m '
                                'from a person, standing still keeps 1.4 m', for_s=1.0))
r, _, calls = ask('why did you stop', emerg)
check('emergency stop phrased "I\'ve stopped: ..."', r.startswith("I've stopped: going on would pass 0.2 m")
      and not calls, r)
moving = dict(base, traffic={'state': 'go', 'why': 'clear', 'say': 'clear', 'v_limit': None, 'limit': None,
                             'person_m': None, 'vehicle_m': None, 'for_s': 0.0},
              last_hold={'say': 'waiting at a walkway for a person to cross (going now would pass 0.4 m '
                         'from them)', 'for_s': 8.0, 'ago_s': 12.0})
r, _, calls = ask('why did you stop just now?', moving)
check('moving again: tells the last stop and how long ago', r.startswith('Nothing is holding me right now')
      and 'about 12 seconds ago' in r and 'for 8 seconds' in r and not calls, r)
slow = dict(moving, traffic=dict(moving['traffic'], say='slowed to 0.4 m/s: a vehicle is near', v_limit=0.4))
r, _, calls = ask('why are you so slow?', slow)
check('slow: the speed limit and its reason', r == 'Slowed to 0.4 m/s: a vehicle is near.' and not calls, r)
r, _, calls = ask("why aren't you moving", dict(base, paused=True, traffic=None))
check('paused: says it was told to stop', r.startswith("I'm paused") and not calls, r)
r, _, calls = ask('why is aisle 2 not finished?', wait)
check('a why-question about a zone goes to the planner (model)', bool(calls), r)
old = {k: v for k, v in base.items() if k not in ('guard', 'traffic', 'holds', 'held_s', 'waiting_s',
                                                  'deferred', 'revisiting', 'revisited', 'skipped',
                                                  'last_hold')}
r, _, calls = ask('why did you stop?', old)
check('older coverage_nav2 (no traffic fields): the planner answers as before', bool(calls), r)
facts = describe(wait)
check('FACTS: holding, nearest vehicle, stops on this job', 'safety layer is holding it: giving way to a '
      'vehicle' in facts and 'nearest vehicle it can see is 1.6 m' in facts and '7 times, 63 seconds' in facts,
      facts[-400:])
facts_old = describe(old)
check('FACTS for an older coverage_nav2 say nothing about traffic', 'safety layer' not in facts_old)
deferred = dict(moving, deferred=[{'zone': 'aisle_2', 'x': 2.48, 'y': 1.1, 'why': 'blocked'}],
                skipped=[{'zone': 'aisle_2', 'label': 'rv1a'}])
facts = describe(deferred)
check('FACTS: put off and given up', 'put off 1 stretch of floor' in facts and 'gave up on 1 stretch' in facts)
r, c, calls = ask('clean aisle 3', wait)
check('commands still work while held', c is not None and c.get('action') == 'clean'
      and c.get('zones') == ['aisle_3'], (c, r))
r, c, calls = ask('stop', wait)
check('"stop" is still a command', c is not None and c.get('action') == 'stop', c)
# ---- v10.5: lanes ----
lanes = dict(moving, swept={'aisle_1': 96.3, 'aisle_2': 48.7}, clean_at=90.0, patching=False,
             patches=0, bypasses=2, partial={'aisle_2': 48},
             lane={'phase': 'follow', 'going_round': False, 'left_m': 7.5, 'stood_s': 9.0, 'bypasses': 1})
facts = describe(lanes)
check('v10.5 FACTS: lanes swept per aisle and the 90 % rule', 'aisle 1 96 percent; aisle 2 49 percent' in facts
      and 'finished at 90 percent' in facts, facts[facts.find('Lanes swept'):][:160])
check('v10.5 FACTS: partial aisle given as lanes swept, not waypoints',
      'aisle 2 about 48 percent of its lanes swept' in facts, facts[facts.find('Partly'):][:120])
check('v10.5 FACTS: blockages gone round', 'gone round 2 blockages' in facts)
r, _, calls = ask('why did you stop?', lanes)
check('v10.5 standing behind something in the lane: says so, and what it will do',
      r.startswith('Something is standing in my lane') and '9 seconds' in r and not calls, r)
r, _, calls = ask('why did you stop?', dict(lanes, lane=dict(lanes['lane'], going_round=True, phase='enter')))
check('v10.5 going round: says so', r.startswith('Someone was standing in my lane, so I\'m going round')
      and not calls, r)
facts = describe(dict(lanes, patching=True, patches=3, revisiting=True))
check('v10.5 FACTS: the patch pass (and not also "going back over floor it had to leave")',
      'going back over stretches of lane it left dirty' in facts
      and 'floor it had to leave earlier' not in facts)
facts_old = describe(moving)
check('v10.5 FACTS for a v10.4 coverage_nav2 say nothing about lanes', 'Lanes swept' not in facts_old)
# ---- v10.6: the guard gone quiet, something stopped short of in the lane, lane ends left to a vehicle ----
hold = dict(lanes, safety_hold=True, lane={'phase': 'follow', 'going_round': False, 'left_m': 7.5,
                                            'stood_s': 4.0, 'bypasses': 0})
r, _, calls = ask('why did you stop?', hold)
check('v10.6 safety hold: says the people guard has gone quiet', r.startswith("I've stopped because my safety "
      "layer, the people guard, has gone quiet") and not calls, r)
r, _, calls = ask('why did you stop?', dict(hold, traffic=wait['traffic']))
check('v10.6 a live guard WAIT still wins over an old safety-hold flag', r.startswith("I'm giving way") and not calls, r)
facts = describe(hold)
check('v10.6 FACTS: holding because the guard stopped reporting', 'safety layer (the people guard) has stopped '
      'reporting' in facts, facts[-300:])
blk = dict(lanes, lane={'phase': 'follow', 'going_round': False, 'left_m': 7.5, 'stood_s': 1.0, 'bypasses': 0,
                         'blocked_ahead': True, 'clear_ahead_m': 0.42})
r, _, calls = ask('why are you standing still?', blk)
check('v10.6 stopped short of something in the lane: says so, with the gap', r.startswith(
      'Something is standing in my lane, 0.4 m ahead, so I\'ve stopped short of it') and not calls, r)
r, _, calls = ask('why did you stop?', dict(blk, lane=dict(blk['lane'], clear_ahead_m=None)))
check('v10.6 ... and without a gap figure', r.startswith("Something is standing in my lane, so I've stopped short")
      and not calls, r)
facts = describe(dict(blk, vehicle_skips=2))
check('v10.6 FACTS: lane ends left to a vehicle, and stopped short in the lane',
      'left 2 bits of lane to a vehicle' in facts and 'stopped short of something standing in its lane' in facts,
      facts[-300:])
check('v10.6 FACTS for a v10.5 coverage_nav2 say nothing new', 'people guard) has stopped' not in describe(lanes)
      and 'to a vehicle working there' not in describe(lanes))
down = dict(lanes, nav2_down=True)
r, _, calls = ask('why did you stop?', down)
check('v10.6 Nav2 down: says navigation is not taking goals, nothing skipped', r.startswith(
      "I've stopped because my navigation (Nav2) isn't taking goals") and 'nothing is being skipped' in r
      and not calls, r)
check('v10.6 FACTS: holding because Nav2 is not taking goals', 'navigation (Nav2) is not taking goals'
      in describe(down))
print('ALL OK' if not FAILS else f'{len(FAILS)} FAILED: {FAILS}')
sys.exit(1 if FAILS else 0)
