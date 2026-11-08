#!/usr/bin/env python3
"""ear_server.py (v10.11) - on the MAC: the robot page's ears. Speech to text with Whisper, on this Mac,
no cloud. The page (http://localhost:8080, through mac_web.py) records what you say after 🎤 and sends
it here; the words go back to the page, which sends them to the robot as if typed.

Once:   ~/eye/env/bin/python -m venv ~/ear && ~/ear/bin/pip install faster-whisper "av<19"
        (its own little environment, made from the detector's Python, so the detector's packages stay as they are)
Run:    ~/ear/bin/python ~/ear_server.py           (the first start downloads the model, ~250 MB)
        ~/ear/bin/python ~/ear_server.py base.en   (smaller and quicker; less sure of accents)
Then reload the page: the talk header says 'ear: Whisper'. Ctrl+C stops it; the page then goes back to
the browser's own speech recognition.

Why: the browser's recogniser (Safari's or Chrome's) does not know this robot's words and wrote "clean
aisle one" as "Please Swan" and "Play Selvan". Whisper is told the words first (a prompt and hot words:
aisle, front area, back area, carry on ...), so it hears them as they are meant, in any accent. It runs
on the CPU (int8): about a second for a short command on an M-series Mac. (PyAV 19 dropped an option
faster-whisper 1.2 passes when it decodes audio; "av<19" avoids that, and decode() below works either way.)

It listens on 127.0.0.1:8766 only: nothing outside this Mac can use it. The page reaches it because the
page itself is on this Mac (localhost). CORS headers let the page (port 8080) call it (port 8766).
"""
import argparse
import io
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

VERSION = 'v10.11'
PORT = 8766
PROMPT = ("Commands to a warehouse cleaning robot. Clean aisle 1. Clean aisle 2. Clean aisle 3. Clean the first "
          "aisle. Clean the second aisle. Clean the third aisle. Clean the front area. Clean the back area. Clean "
          "everything. Stop. Carry on. Go home. Why did you stop? What do you see? How long left? Introduce "
          "yourself.")
HOTWORDS = 'aisle, clean, front area, back area, carry on, go home, robot'
MAX_BYTES = 4_000_000                 # about a minute of compressed audio: commands are seconds


def decode(audio):
    """The recording (WebM/Opus from Chrome, MP4/AAC from Safari) as 16 kHz mono float32 for Whisper:
    faster-whisper's own decoder, or, if this PyAV no longer takes its metadata_errors option (PyAV 19),
    the same decoding without it."""
    from faster_whisper import decode_audio
    try:
        return decode_audio(io.BytesIO(audio))
    except TypeError as e:
        if 'metadata_errors' not in str(e):
            raise
    import av
    import numpy as np
    rs = av.audio.resampler.AudioResampler(format='s16', layout='mono', rate=16000)
    out = []
    with av.open(io.BytesIO(audio), mode='r') as c:
        for fr in c.decode(audio=0):
            out += [f.to_ndarray().reshape(-1) for f in rs.resample(fr)]
    out += [f.to_ndarray().reshape(-1) for f in rs.resample(None)]
    return np.concatenate(out).astype(np.float32) / 32768.0 if out else np.zeros(0, np.float32)


def tiny_wav(seconds=0.3):
    """A short silent WAV: the start-up check that audio decodes here."""
    import wave
    b = io.BytesIO()
    with wave.open(b, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(16000)
        w.writeframes(b'\x00\x00' * int(16000 * seconds))
    return b.getvalue()


class Ear:
    def __init__(self, model, threads):
        from faster_whisper import WhisperModel          # pip install faster-whisper
        t0 = time.time()
        self.name = model
        self.model = WhisperModel(model, device='cpu', compute_type='int8', cpu_threads=threads)
        self.lock = threading.Lock()                     # one at a time: the model is not re-entrant
        self.load_s = time.time() - t0

    def hear(self, audio):
        t0 = time.time()
        samples = decode(audio)
        if len(samples) < 1600:                          # under 0.1 s: nothing to write down
            return {'text': '', 'unsure': True, 'logprob': None, 'no_speech': None,
                    'audio_s': round(len(samples) / 16000, 2), 'ms': int((time.time() - t0) * 1000), 'model': self.name}
        with self.lock:
            segs, info = self.model.transcribe(
                samples, language='en', beam_size=5, temperature=0.0, initial_prompt=PROMPT,
                hotwords=HOTWORDS, vad_filter=True, without_timestamps=True, condition_on_previous_text=False)
            segs = list(segs)
        text = ' '.join(s.text.strip() for s in segs).strip()
        lp = min((s.avg_logprob for s in segs), default=None)
        ns = max((s.no_speech_prob for s in segs), default=None)
        unsure = not text or (lp is not None and lp < -1.0) or (ns is not None and ns > 0.6)
        return {'text': text, 'unsure': unsure, 'logprob': None if lp is None else round(lp, 2),
                'no_speech': None if ns is None else round(ns, 2), 'audio_s': round(info.duration, 2),
                'ms': int((time.time() - t0) * 1000), 'model': self.name}


def make_handler(ear):
    class H(BaseHTTPRequestHandler):
        server_version = 'ear_server/' + VERSION

        def log_message(self, fmt, *args):
            pass

        def cors(self):
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.send_header('Access-Control-Allow-Private-Network', 'true')

        def reply(self, code, obj):
            b = json.dumps(obj).encode()
            self.send_response(code)
            self.cors()
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_OPTIONS(self):
            self.send_response(204)
            self.cors()
            self.send_header('Content-Length', '0')
            self.end_headers()

        def do_GET(self):
            if self.path.startswith('/health'):
                return self.reply(200, {'ok': True, 'version': VERSION, 'model': ear.name,
                                        'load_s': round(ear.load_s, 1)})
            self.reply(404, {'ok': False, 'error': 'not found'})

        def do_POST(self):
            if not self.path.startswith('/hear'):
                return self.reply(404, {'ok': False, 'error': 'not found'})
            n = int(self.headers.get('Content-Length') or 0)
            if n <= 0 or n > MAX_BYTES:
                left = min(n, 16 * MAX_BYTES)
                while left > 0:                          # read it off, so the page gets the answer, not a reset
                    b = self.rfile.read(min(65536, left))
                    if not b:
                        break
                    left -= len(b)
                return self.reply(413 if n > MAX_BYTES else 400, {'ok': False, 'error': f'{n} bytes of audio'})
            audio = self.rfile.read(n)
            try:
                out = ear.hear(audio)
            except Exception as e:                       # undecodable audio, a model error: say so, keep serving
                return self.reply(500, {'ok': False, 'error': f'{type(e).__name__}: {e}'[:300]})
            print(f"[{time.strftime('%H:%M:%S')}] heard {out['text']!r} ({out['audio_s']} s of audio in "
                  f"{out['ms']} ms{', unsure' if out['unsure'] else ''})", flush=True)
            self.reply(200, dict(out, ok=True))
    return H


def main(argv=None):
    ap = argparse.ArgumentParser(description='Speech to text for the robot page, on this Mac (Whisper).')
    ap.add_argument('model', nargs='?', default='small.en',
                    help='faster-whisper model: small.en (default), base.en (quicker), distil-small.en, medium.en')
    ap.add_argument('--port', type=int, default=PORT)
    ap.add_argument('--threads', type=int, default=4)
    a = ap.parse_args(argv)
    try:
        import faster_whisper  # noqa: F401
    except ImportError:
        print('faster-whisper is not installed for this Python. Once:\n'
              '  ~/eye/env/bin/python -m venv ~/ear && ~/ear/bin/pip install faster-whisper "av<19"\n'
              'then run this with that Python:  ~/ear/bin/python ~/ear_server.py')
        return 1
    print(f'loading Whisper {a.model} (the first time it downloads it) ...', flush=True)
    try:
        ear = Ear(a.model, a.threads)
    except Exception as e:
        print(f'could not load the model: {type(e).__name__}: {e}')
        return 1
    try:
        decode(tiny_wav())
    except Exception as e:
        print(f'audio does not decode with this PyAV ({type(e).__name__}: {e}). Fix:  ~/ear/bin/pip install "av<19"')
        return 1
    try:
        srv = ThreadingHTTPServer(('127.0.0.1', a.port), make_handler(ear))
    except OSError as e:
        print(f'port {a.port} is taken on this Mac ({e}). Is ear_server.py running already?')
        return 1
    print(f'ear_server {VERSION}: Whisper {a.model} ready in {ear.load_s:.0f} s, on http://127.0.0.1:{a.port} '
          '(this Mac only). Reload the robot page; its talk header says "ear: Whisper". Ctrl+C to stop.', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\nstopped')
    return 0


if __name__ == '__main__':
    sys.exit(main())
