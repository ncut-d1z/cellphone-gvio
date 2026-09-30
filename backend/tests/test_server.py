"""Real TLS+ALPN+h2 integration; no generated credentials leave TemporaryDirectory."""
import concurrent.futures
import json
import os
from pathlib import Path
import socket
import ssl
import subprocess
import sys
import tempfile
import time
import httpx


def run(executable: str) -> None:
    with tempfile.TemporaryDirectory(prefix="gvio-tls-") as temporary:
        root = Path(temporary)
        cert, key = root / "server.crt", root / "server.key"
        subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "1",
                        "-subj", "/CN=localhost", "-addext", "subjectAltName=DNS:localhost,IP:127.0.0.1",
                        "-keyout", str(key), "-out", str(cert)], check=True, capture_output=True)
        (root / "index.html").write_text("gvio integration", encoding="utf-8")
        payload = bytes(range(256)) * 8192
        (root / "large.bin").write_bytes(payload)
        with socket.socket() as reserve:
            reserve.bind(("127.0.0.1", 0))
            port = reserve.getsockname()[1]
        process = subprocess.Popen([executable, str(cert), str(key), str(port), str(root)],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
                ready = pool.submit(process.stdout.readline).result(timeout=15)
            assert ready.strip() == "READY", "TLS listener did not start"
            context = ssl.create_default_context(cafile=str(cert))
            base = f"https://127.0.0.1:{port}"
            with httpx.Client(http2=True, verify=context, timeout=8, trust_env=False) as client:
                response = client.get(base + "/")
                assert response.http_version == "HTTP/2" and response.text == "gvio integration"
                assert client.get(base + "/large.bin").content == payload, "TLS partial-write/flow-control regression"
                assert client.get(base + "/missing").status_code == 404
                response = client.post(base + "/control", json={"cmd": "reset"})
                assert response.json()["ok"], "POST END_STREAM handling"
                # Two simultaneous streams on ONE h2 connection must not share offsets.
                def read_events():
                    result = []
                    with client.stream("GET", base + "/state") as stream:
                        assert stream.http_version == "HTTP/2"
                        for line in stream.iter_lines():
                            if line.startswith("data: "):
                                result.append(json.loads(line[6:]))
                                if len(result) == 3:
                                    return result
                    raise AssertionError("SSE stream ended early")
                with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
                    results = [pool.submit(read_events) for _ in range(2)]
                    for future in results:
                        assert len(future.result(timeout=12)) == 3
                for cmd in ("stop", "start", "reset", "magcal"):
                    assert client.post(base + "/control", json={"cmd": cmd}).json()["ok"], cmd
                assert client.get(base + "/").status_code == 200, "control stop must not join its server thread"
            for _ in range(24):
                with httpx.Client(http2=True, verify=context, timeout=8, trust_env=False) as client:
                    assert client.get(base + "/").status_code == 200
            process.stdin.write("quit\n")
            process.stdin.flush()
            assert process.wait(timeout=10) == 0, "clean shutdown"
            print("PASS TLS, ALPN, GET, POST, large body, concurrent SSE, control, reconnect, shutdown")
        finally:
            if process.poll() is None:
                process.kill()
            _, errors = process.communicate(timeout=10)
            if errors:
                print(errors, file=sys.stderr)


if __name__ == "__main__":
    run(sys.argv[1])
