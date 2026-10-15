"""Sign in to OpenBeam Cloud with the device-code flow and save the token.

  python3 -m openbeam_bridge.login --token-file ~/.config/openbeam/token
  python3 -m openbeam_bridge.login --token-file ~/.config/openbeam/token --sessions

Open the printed link, check the code matches, approve; the token is written
owner-only. `--sessions` lists your running sessions (pick OPENBEAM_SESSION_ID).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request


def _post(url: str, body=None, token: str = ""):
    data = json.dumps(body or {}).encode()
    request = urllib.request.Request(url, data=data, method="POST", headers={"Content-Type": "application/json"})
    if token:
        request.add_header("Authorization", f"Bearer {token}")
    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            return response.status, json.loads(response.read() or b"{}")
    except urllib.error.HTTPError as error:
        try:
            return error.code, json.loads(error.read() or b"{}")
        except ValueError:
            return error.code, {}


def _get(url: str, token: str):
    request = urllib.request.Request(url, headers={"Authorization": f"Bearer {token}"})
    with urllib.request.urlopen(request, timeout=20) as response:
        return json.loads(response.read())


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--api", default=os.environ.get("OPENBEAM_CLOUD_API_URL", "https://cloud.openbeam.world"))
    parser.add_argument("--token-file", required=True)
    parser.add_argument("--sessions", action="store_true", help="only list sessions with the saved token")
    args = parser.parse_args(argv)
    api = args.api.rstrip("/")
    path = os.path.expanduser(args.token_file)

    if not args.sessions:
        status, code = _post(f"{api}/v1/device/code")
        if status != 200:
            print(f"device code request failed: HTTP {status} {code}", file=sys.stderr)
            return 1
        print(f"Open {code['verificationUriComplete']}\nand approve the code {code['userCode']} "
              f"(expires in {code.get('expiresIn', 600)} s).")
        interval = int(code.get("interval", 5))
        deadline = time.monotonic() + int(code.get("expiresIn", 600))
        token = None
        while time.monotonic() < deadline:
            time.sleep(interval)
            status, answer = _post(f"{api}/v1/device/token", {"deviceCode": code["deviceCode"]})
            if status == 200 and answer.get("token"):
                token = answer["token"]
                break
            error = answer.get("error", "")
            if error == "slow_down":
                interval += 5
            elif error not in ("authorization_pending", ""):
                print(f"sign-in ended: {error}", file=sys.stderr)
                return 1
        if not token:
            print("the code expired", file=sys.stderr)
            return 1
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w") as handle:
            handle.write(token)
        print(f"signed in; token saved to {path}")

    with open(path, encoding="utf-8") as handle:
        token = handle.read().strip()
    sessions = _get(f"{api}/v1/sessions", token)
    rows = sessions if isinstance(sessions, list) else sessions.get("sessions", [])
    live = [s for s in rows if s.get("state") in ("ready", "streaming", "starting")]
    if not live:
        print("no running session: open the Cloud dashboard and press Play OpenBeam, then run with --sessions")
    for session in live:
        print(f"OPENBEAM_SESSION_ID={session.get('sessionId') or session.get('id')}  state={session.get('state')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
