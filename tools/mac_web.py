#!/usr/bin/env python3
"""mac_web.py (v10.9) - on the MAC: the robot's web page at http://localhost:8080, so the microphone works.

    python3 mac_web.py            then open http://localhost:8080 in Chrome or Safari

Browsers only give a page the microphone when it comes from https or from localhost. The page lives
on the Ubuntu VM (robot_web.py, port 8080), so this passes localhost:8080 through to it, byte for
byte. Nothing is installed; Ctrl+C stops it. Get it from the VM's page (it fills in the VM's address):
    curl -o mac_web.py http://<VM address>:8080/mac.py
or give the address:   python3 mac_web.py 192.168.64.4
"""
import socket
import sys
import threading

VM = '192.168.64.4'
PORT = 8080


def pipe(src, dst):
    try:
        while True:
            b = src.recv(65536)
            if not b:
                break
            dst.sendall(b)
    except OSError:
        pass
    finally:
        for s in (src, dst):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                s.close()
            except OSError:
                pass


def serve(vm, port, local=None):
    local = local or port
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind(('127.0.0.1', local))
    except OSError as e:
        print(f'port {local} is taken on this Mac ({e}). Is mac_web.py running already?')
        return 1
    srv.listen(64)
    try:
        socket.create_connection((vm, port), timeout=3).close()
        print(f'the robot page on the VM answers ({vm}:{port})')
    except OSError as e:
        print(f'WARNING: nothing answers at {vm}:{port} yet ({e}). Start the demo on the VM '
              f'(shift.sh demo); this keeps trying.')
    print(f'Open http://localhost:{local} in Chrome or Safari (the microphone works there). Ctrl+C to stop.')
    while True:
        try:
            c, _ = srv.accept()
        except KeyboardInterrupt:
            print('\nstopped')
            return 0
        try:
            u = socket.create_connection((vm, port), timeout=5)
            u.settimeout(None)
        except OSError:
            c.sendall(b'HTTP/1.1 502 Bad Gateway\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n'
                      b'The robot page on the VM does not answer. Is the demo running (shift.sh demo)?\n')
            c.close()
            continue
        threading.Thread(target=pipe, args=(c, u), daemon=True).start()
        threading.Thread(target=pipe, args=(u, c), daemon=True).start()


if __name__ == '__main__':
    sys.exit(serve(sys.argv[1] if len(sys.argv) > 1 else VM, PORT))
