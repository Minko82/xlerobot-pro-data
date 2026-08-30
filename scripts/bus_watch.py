#!/usr/bin/env python3
"""Find which servo link on the arm bus is dropping out.

Retrying makes a session survivable; it does not repair anything. This locates
the fault so it can be fixed properly.

The servos are a daisy chain -- adapter -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -- so a
bad cable or connector takes out that motor *and every motor after it*, while
the ones before it stay perfect. The pattern of which ids fail therefore names
the link:

    all six clean                 the bus is fine right now; keep flexing
    5 and 6 failing, 1-4 clean    the fault is in the 4 -> 5 link
    only 6 failing                the fault is in the 5 -> 6 link
    all six failing               adapter, its USB cable, or the 0 -> 1 link

Run it, then flex each joint and tug gently on each cable in turn. The counts
move the moment you disturb the bad one, which is what makes this quicker than
swapping parts and hoping.

    python scripts/bus_watch.py                  # left arm, ids 1-6
    python scripts/bus_watch.py --ids 7 8 9 10 11 12   # right arm

Ctrl-C to stop and print the totals.
"""

from __future__ import annotations

import argparse
import sys
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

LEFT = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", default="/dev/xle_head")
    p.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    p.add_argument("--hz", type=float, default=30.0, help="Poll rate; match recording.")
    p.add_argument("--limp", action="store_true",
                   help="Disable torque first, so the arm can be back-driven by hand. This is "
                        "the state recording and calibration run in, and every observed failure "
                        "has happened in it -- back-driving six servos injects back-EMF into the "
                        "shared supply. THE ARM WILL GO SLACK: support it before using this.")
    p.add_argument("--no-sync", action="store_true",
                   help="Skip the sync_read probe and only read motors individually.")
    args = p.parse_args()

    names = LEFT if len(args.ids) == 6 else [f"id{i}" for i in args.ids]
    motors = {n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
              for n, i in zip(names, args.ids)}
    bus = FeetechMotorsBus(port=args.port, motors=motors)
    bus.connect(handshake=False)

    # Read each id on its own. A sync_read of all six cannot say which one went
    # missing -- it just fails -- and "which one" is the entire question here.
    ok = dict.fromkeys(names, 0)
    err = dict.fromkeys(names, 0)
    sync_ok = sync_err = 0
    period = 1.0 / args.hz

    if args.limp:
        input("\n  Torque is about to be DISABLED and the arm will go slack.\n"
              "  Support it, then press ENTER...")
        bus.disable_torque()
        print("  arm is limp -- move it by hand while this runs\n")
    print(f"  polling {len(names)} motors on {args.port} at {args.hz:.0f} Hz")
    print("  flex each joint and tug each cable in turn. Ctrl-C to stop.\n")
    try:
        while True:
            t = time.perf_counter()
            # The broadcast read is the call that actually fails in recording and
            # calibration: six servos answer back-to-back on one half-duplex line,
            # so it breaks on timing that per-motor reads tolerate. Probe it first,
            # then read individually to say WHICH motor went missing.
            if not args.no_sync:
                try:
                    bus.sync_read("Present_Position", normalize=False, num_retry=0)
                    sync_ok += 1
                except Exception:
                    sync_err += 1
            for n in names:
                try:
                    bus.read("Present_Position", n, normalize=False, num_retry=0)
                    ok[n] += 1
                except Exception:
                    err[n] += 1
            total = sum(ok.values()) + sum(err.values())
            if total % (len(names) * 10) == 0:
                cells = " ".join(
                    f"{n.split('_')[0][:5]}:{'.' if not err[n] else str(err[n])}"
                    for n in names)
                sync = "" if args.no_sync else f"  sync_err:{sync_err}"
                sys.stdout.write(f"\r  {total // len(names):6d} polls   {cells}{sync}    ")
                sys.stdout.flush()
            time.sleep(max(0.0, period - (time.perf_counter() - t)))
    except KeyboardInterrupt:
        pass

    if not args.no_sync:
        tot = sync_ok + sync_err
        rate = 100 * sync_err / tot if tot else 0
        print(f"\n\n  sync_read of all six (the call that fails in recording/calibration)")
        print(f"    ok={sync_ok}  err={sync_err}   failure rate {rate:.2f}%"
              f"{'   <-- REPRODUCED' if sync_err else ''}")

    print("\n  id   motor              ok      err     rate")
    bad = []
    for n in names:
        tot = ok[n] + err[n]
        rate = 100 * err[n] / tot if tot else 0
        # Any drop at all is worth naming. The real fault on this arm sits near
        # 0.1%, so a "significant" threshold of half a percent reported a clean
        # bus while printing non-zero error counts directly above it.
        flag = "   <-- DROPPING" if err[n] else ""
        if err[n]:
            bad.append((motors[n].id, n))
        print(f"  {motors[n].id:<4} {n:<16} {ok[n]:6d}  {err[n]:6d}   {rate:5.2f}%{flag}")

    if not bad:
        print("\n  No drops on any motor. Either the bus is healthy right now, or you did "
              "not disturb\n  the bad connection -- it only shows up under the movement that "
              "provokes it.")
    else:
        first = min(b[0] for b in bad)
        others = [i for i in args.ids if i >= first]
        if [b[0] for b in bad] == others:
            prev = args.ids[args.ids.index(first) - 1] if args.ids.index(first) else "the adapter"
            print(f"\n  ids {others} failed and everything before them was clean.")
            print(f"  That is the signature of one bad link: {prev} -> {first}.")
            print(f"  Reseat or replace that cable first.")
        else:
            ids = [b[0] for b in bad]
            clean = [i for i in args.ids if i not in ids]
            print(f"\n  Drops on ids {ids}, but ids {clean} stayed clean.")
            print("  That is NOT a clean suffix of the chain, so it is not one bad cable --")
            print("  a broken link takes out its motor and everything after it.")
            if not args.no_sync:
                itot = sum(ok.values()) + sum(err.values())
                irate = 100 * sum(err.values()) / itot if itot else 0
                stot = sync_ok + sync_err
                srate = 100 * sync_err / stot if stot else 0
                if srate and irate:
                    print(f"\n  sync_read {srate:.3f}% vs individual reads {irate:.3f}% "
                          f"-- the broadcast is {srate / irate:.1f}x worse.")
                    print("  That is marginal timing on the shared line, not a broken "
                          "conductor.\n  Widen the retry window; do not replace cables.")
    bus.disconnect(disable_torque=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
