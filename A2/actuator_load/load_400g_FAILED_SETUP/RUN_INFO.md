# 400 g — FAILED SETUP, not a platform limit

Aborted after 16 s. Originally documented as shoulder saturation; that was
WRONG. A clean 400 g run (load_400g) reached the reference pose with the
shoulder at 102/450 torque (23%), current 8.

In this aborted attempt the arm never reached the pose — it settled 246 counts
short versus +43 in the good run — so it was working at a much longer moment
arm. current 84, load 410/450.

Cause: setup issue before the weight was attached, not a torque limit.
Retained only as an example of what a failed ramp looks like in telemetry.
