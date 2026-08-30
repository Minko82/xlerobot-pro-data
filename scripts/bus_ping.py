#!/usr/bin/env python3
"""Ask which servo ids answer on a port.

A USB-serial adapter appearing in /dev says only that its bridge chip
enumerated. It says nothing about whether the motor bus behind it is alive, and
the two have come apart here before: adapter 5B14031533 enumerates cleanly as
/dev/xle_arms and no id on it has ever replied. Enumeration is not the test.
This is.

    python scripts/bus_ping.py --port /dev/xle_head --ids 1 2 3 4 5 6
    python scripts/bus_ping.py --port /dev/xle_arms          # ids 1-20

Read-only by construction. It never enables torque and never writes a register,
so it is safe against a torqued arm and safe to run before calibration is
trusted. That is also why it closes the port by hand rather than calling
bus.disconnect(), which writes Torque_Enable=0 and would drop a held arm.

For a healthy-but-marginal bus, this is the wrong tool -- it answers "is
anything there", not "which link is dropping packets". Use bus_watch.py for
that.
"""

from __future__ import annotations

import argparse
import sys

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", required=True)
    p.add_argument("--ids", type=int, nargs="+", default=list(range(1, 21)),
                   help="Ids to try. Default 1-20 sweeps wide enough to find a bus "
                        "whose ids you do not already know.")
    args = p.parse_args()

    motors = {f"id{i}": Motor(i, "sts3215", MotorNormMode.RANGE_M100_100) for i in args.ids}
    bus = FeetechMotorsBus(port=args.port, motors=motors)
    try:
        bus.connect(handshake=False)
    except Exception as exc:
        # Opening the port is a different failure from the bus being silent, and
        # only this one implicates the adapter itself.
        print(f"OPEN FAILED {args.port}: {type(exc).__name__}: {exc}")
        return 1

    found = []
    for i in args.ids:
        name = f"id{i}"
        try:
            # Raw counts: normalising needs a calibration this deliberately does
            # not assume it has.
            pos = bus.read("Present_Position", name, normalize=False)
            found.append(i)
            print(f"  id {i:>3}  OK   raw_pos={pos}")
        except Exception as exc:
            print(f"  id {i:>3}  --   {type(exc).__name__}")

    # Not bus.disconnect(): it writes Torque_Enable=0 on the way out, which would
    # drop a torqued arm. Close the port and touch nothing.
    bus.port_handler.closePort()

    print(f"\n{args.port}: {len(found)} of {len(args.ids)} answered: {found}")
    return 0 if found else 1


if __name__ == "__main__":
    sys.exit(main())
