#!/usr/bin/env python3
"""test_ear.py (v10.11) - ear_server.py with a stand-in for faster-whisper (no model download):
health, the CORS preflight the page's fetch needs, a recording written down, the words the model is
primed with, empty / too big / undecodable audio, and it keeps serving after an error.

    python3 test_ear.py <ear_server.py>
"""
import importlib.util
import json
import os
import socket
import sys
import threading
import types
import urllib.request

FAILS = []


def check(ok, what):
    print(('  ok   ' if ok else '  FAIL ') + what, flush=True)
    if not ok:
        FAILS.append(what)


CALLS = []


class Seg:
    def __init__(self, text, lp=-0.2, ns=0.01):
        self.text, self.avg_logprob, self.no_speech_prob = text, lp, ns


class FakeAudio:
    """What the stand-in decode_audio returns: the bytes, as long as real audio of that size would be."""
    def __init__(self, data):
        self.data = data

    def __len__(self):
        return 0 if self.data.startswith(b'TINY') else 16000


def fake_decode(f):
    data = f.read()
    if data.startswith(b'BAD'):
        raise ValueError('Invalid data found when processing input')
    return FakeAudio(data)


class FakeModel:
    def __init__(self, name, device='cpu', compute_type='default', cpu_threads=0, **kw):
        self.args = (name, device, compute_type, cpu_threads)

    def transcribe(self, audio, **kw):
        data = audio.data
        CALLS.append(kw)
        if data.startswith(b'MUMBLE'):
            return iter([Seg(' clean isle wan', lp=-1.4, ns=0.2)]), types.SimpleNamespace(duration=1.2)
        if data.startswith(b'QUIET'):
            return iter([]), types.SimpleNamespace(duration=2.0)
        return iter([Seg(' Clean aisle 1.')]), types.SimpleNamespace(duration=1.6)


def free_port():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    p = s.getsockname()[1]
    s.close()
    return p


def http(port, path, data=None, method=None, headers=None):
    req = urllib.request.Request(f'http://127.0.0.1:{port}{path}', data=data, method=method, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, r.read(), r.headers
    except urllib.error.HTTPError as e:
        return e.code, e.read(), e.headers


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    sys.modules['faster_whisper'] = types.SimpleNamespace(WhisperModel=FakeModel, decode_audio=fake_decode)
    spec = importlib.util.spec_from_file_location('ear_t', os.path.abspath(argv[0]))
    E = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(E)
    print(f'test_ear: {argv[0]} ({E.VERSION})')
    ear = E.Ear('small.en', 2)
    check(ear.model.args == ('small.en', 'cpu', 'int8', 2), 'the model runs on the CPU in int8 (an M-series Mac: about a second)')
    port = free_port()
    srv = E.ThreadingHTTPServer(('127.0.0.1', port), E.make_handler(ear))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    c, b, h = http(port, '/health')
    j = json.loads(b)
    check(c == 200 and j['ok'] and j['model'] == 'small.en' and h.get('Access-Control-Allow-Origin') == '*',
          'GET /health: ok, the model, and CORS (the page on :8080 calls it on :8766)')
    c, b, h = http(port, '/hear', method='OPTIONS', headers={'Origin': 'http://localhost:8080',
                                                            'Access-Control-Request-Method': 'POST',
                                                            'Access-Control-Request-Headers': 'content-type'})
    check(c == 204 and 'POST' in (h.get('Access-Control-Allow-Methods') or '')
          and 'Content-Type' in (h.get('Access-Control-Allow-Headers') or ''),
          "the browser's preflight for POST with Content-Type audio/webm is allowed")
    c, b, h = http(port, '/hear', data=b'OK' + b'\x00' * 3000, headers={'Content-Type': 'audio/webm;codecs=opus'})
    j = json.loads(b)
    check(c == 200 and j['ok'] and j['text'] == 'Clean aisle 1.' and not j['unsure'] and j['audio_s'] == 1.6,
          f'a recording is written down: {j.get("text")!r}')
    kw = CALLS[-1]
    check(kw.get('language') == 'en' and 'aisle' in (kw.get('initial_prompt') or '') and 'front area' in
          (kw.get('hotwords') or '') and kw.get('vad_filter') is True and kw.get('condition_on_previous_text') is False
          and kw.get('temperature') == 0.0, "Whisper is primed with the robot's words (prompt + hot words), English, "
                                            'silence cut, nothing carried over between commands')
    real = os.environ.get('FW_SRC')
    if real:
        import inspect
        src = open(os.path.join(real, 'faster_whisper', 'transcribe.py')).read()
        sig = src[src.index('    def transcribe('):]
        sig = sig[:sig.index(') -> ')]
        check(all(f'{k}:' in sig for k in kw), f'every option is in faster-whisper\'s own transcribe(): {sorted(kw)}')
    c, b, h = http(port, '/hear', data=b'MUMBLE' + b'\x00' * 100, headers={'Content-Type': 'audio/mp4'})
    j = json.loads(b)
    check(c == 200 and j['unsure'] and j['text'] == 'clean isle wan', 'a doubtful hearing is marked unsure (the page says so)')
    c, b, h = http(port, '/hear', data=b'QUIET' + b'\x00' * 100)
    j = json.loads(b)
    check(c == 200 and j['text'] == '' and j['unsure'], 'silence: no words, unsure')
    n_calls = len(CALLS)
    c, b, h = http(port, '/hear', data=b'TINY' + b'\x00' * 100)
    j = json.loads(b)
    check(c == 200 and j['text'] == '' and len(CALLS) == n_calls, 'under 0.1 s of audio: no words, the model not even asked')
    c, b, h = http(port, '/hear', data=b'', method='POST')
    check(c == 400 and not json.loads(b)['ok'], 'no audio: 400')
    c, b, h = http(port, '/hear', data=b'\x00' * (E.MAX_BYTES + 1))
    check(c == 413, 'more than a minute of audio: 413')
    c, b, h = http(port, '/hear', data=b'BAD' + b'\x00' * 100)
    j = json.loads(b)
    check(c == 500 and 'Invalid data' in j['error'] and h.get('Access-Control-Allow-Origin') == '*',
          'undecodable audio: 500 with the reason (and CORS, so the page can read it)')
    c, b, h = http(port, '/hear', data=b'OK' + b'\x00' * 100)
    check(c == 200 and json.loads(b)['ok'], 'and it keeps serving')
    c, b, h = http(port, '/nothing')
    check(c == 404, 'anything else: 404')
    srv.shutdown()
    print('ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS))
    return 0 if not FAILS else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
