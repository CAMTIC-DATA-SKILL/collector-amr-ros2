"""호스트에서 컨테이너 포트로 TCP 바이트를 그대로 넘긴다.

ZeroMQ 프레임은 해석하지 않는다. Ubuntu 20.04 의 Python 3.8 에서 동작한다.
"""
import socket
import sys
import threading


def _pipe(src, dst):
    try:
        while True:
            data = src.recv(65536)
            if not data:
                break
            dst.sendall(data)
    except OSError:
        pass
    finally:
        for sock in (src, dst):
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


def _handle(client, target_host, target_port):
    try:
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        upstream = socket.create_connection((target_host, target_port), timeout=5)
        upstream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except OSError as exc:
        print("relay upstream failed: %s" % exc, flush=True)
        client.close()
        return
    left = threading.Thread(target=_pipe, args=(client, upstream))
    right = threading.Thread(target=_pipe, args=(upstream, client))
    left.daemon = True
    right.daemon = True
    left.start()
    right.start()
    left.join()
    right.join()
    client.close()
    upstream.close()


def main():
    if len(sys.argv) != 4:
        print("usage: tcp_relay.py LISTEN_PORT TARGET_HOST TARGET_PORT", file=sys.stderr)
        return 2
    listen_port = int(sys.argv[1])
    target_host = sys.argv[2]
    target_port = int(sys.argv[3])
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", listen_port))
    server.listen(8)
    print("relay 0.0.0.0:%s -> %s:%s" % (listen_port, target_host, target_port), flush=True)
    while True:
        client, _addr = server.accept()
        worker = threading.Thread(target=_handle, args=(client, target_host, target_port))
        worker.daemon = True
        worker.start()


if __name__ == "__main__":
    sys.exit(main() or 0)
