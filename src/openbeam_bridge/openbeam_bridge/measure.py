"""Measure what the bridge can get from a game: per-call latency, sustained
poll rate, camera frame cost and command-to-motion latency.

No ROS needed:  python3 -m openbeam_bridge.measure --seconds 10 [--json out.json]
It reads the same OPENBEAM_* environment as the bridge. `--tool status`
measures the transport alone with a stock game tool, when the robot tools
are missing.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time

from .mcp_client import McpClient, endpoint_from_env
from .robot_api import McpBackend, parse_fixed, parse_joint_map


def percentiles(values):
    if not values:
        return {}
    ordered = sorted(values)

    def q(p):
        return ordered[min(len(ordered) - 1, int(p * len(ordered)))]

    return {"n": len(values), "min_ms": ordered[0] * 1e3, "p50_ms": q(0.5) * 1e3, "p95_ms": q(0.95) * 1e3,
            "max_ms": ordered[-1] * 1e3, "mean_ms": statistics.fmean(values) * 1e3}


def poll(fn, seconds):
    lat = []
    end = time.monotonic() + seconds
    started = time.monotonic()
    errors = 0
    while time.monotonic() < end:
        t0 = time.monotonic()
        try:
            fn()
            lat.append(time.monotonic() - t0)
        except Exception as error:  # keep going: we are measuring
            errors += 1
            if errors < 3:
                print(f"  error: {error}", file=sys.stderr)
    elapsed = time.monotonic() - started
    out = percentiles(lat)
    out["rate_hz"] = len(lat) / elapsed if elapsed else 0.0
    out["errors"] = errors
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--tool", default="", help="measure a plain tool (e.g. status) instead of the robot tools")
    parser.add_argument("--robot", default="")
    parser.add_argument("--state-tool", default="robot_state")
    parser.add_argument("--command-tool", default="robot_set_joints")
    parser.add_argument("--camera", type=int, default=5, help="screenshots to time (0 = skip)")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--step", type=float, default=0.3, help="rad step for the command-to-motion test")
    parser.add_argument("--joint-map", default="", help="game_name:ros_name,... (a stand-in robot)")
    parser.add_argument("--fixed-joints", default="", help="name=value,... SO-101 joints the robot lacks")
    parser.add_argument("--json", default="")
    args = parser.parse_args(argv)

    endpoint = endpoint_from_env()
    client = McpClient(endpoint)
    report = {"endpoint": endpoint.url.split("?")[0], "when": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    t0 = time.monotonic()
    init = client.initialize()
    report["initialize_ms"] = (time.monotonic() - t0) * 1e3
    report["server"] = init.get("serverInfo", {})
    tools = client.tools()
    print(f"connected to {report['server']} at {report['endpoint']}: {len(tools)} tools")

    if args.tool:
        print(f"polling {args.tool} for {args.seconds:.0f} s ...")
        report["tool"] = args.tool
        report["poll"] = poll(lambda: client.call(args.tool, {}), args.seconds)
    else:
        backend = McpBackend(client, robot=args.robot, tools={"state": args.state_tool, "command": args.command_tool},
                             joint_map=parse_joint_map(args.joint_map), fixed=parse_fixed(args.fixed_joints))
        backend.connect()
        print(f"polling {args.state_tool} for {args.seconds:.0f} s ...")
        sims = []

        def read():
            sims.append((time.monotonic(), backend.read_state().sim_time))

        report["state_poll"] = poll(read, args.seconds)
        if len(sims) > 2:
            wall = sims[-1][0] - sims[0][0]
            report["real_time_factor"] = (sims[-1][1] - sims[0][1]) / wall if wall else None
            distinct = len({round(s, 6) for _, s in sims})
            report["distinct_sim_times"] = distinct
            report["fresh_state_rate_hz"] = (distinct - 1) / wall if wall else None
        # command-to-motion: step one joint, time until the reported position moves
        state = backend.read_state()
        joint = "wrist_roll"
        start = state.positions[joint]
        target = start + (args.step if start < 0 else -args.step)
        t_cmd = time.monotonic()
        backend.command_positions({joint: target})
        report["command_call_ms"] = (time.monotonic() - t_cmd) * 1e3
        moved_at = None
        while time.monotonic() - t_cmd < 5.0:
            pos = backend.read_state().positions[joint]
            if abs(pos - start) > 0.002:  # ~0.1 degree: the joint has started to move
                moved_at = time.monotonic()
                break
        report["command_to_motion_ms"] = (moved_at - t_cmd) * 1e3 if moved_at else None
        report["command_poll"] = poll(lambda: backend.command_positions({joint: target}), min(args.seconds, 5.0))
        backend.command_positions({joint: start})

    if args.camera:
        sizes = []

        tool = "camera_frame" if "camera_frame" in tools else "screenshot"
        shot_args = {"max_width": args.camera_width, "format": "jpeg"} if tool == "camera_frame" else {}
        if tool == "screenshot" and "keep" in ((tools["screenshot"].get("inputSchema") or {}).get("properties") or {}):
            shot_args["keep"] = False
        report["camera_tool"] = tool

        def shot():
            _, data = client.image(tool, shot_args)
            sizes.append(len(data))

        print(f"timing {args.camera} {tool} captures ...")
        lat = []
        for _ in range(args.camera):
            t = time.monotonic()
            try:
                shot()
                lat.append(time.monotonic() - t)
            except Exception as error:
                report["camera_error"] = str(error)
                break
        report["camera"] = percentiles(lat)
        if sizes:
            report["camera"]["bytes_mean"] = statistics.fmean(sizes)
            report["camera"]["max_rate_hz"] = 1.0 / statistics.fmean(lat)

    print(json.dumps(report, indent=2))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
