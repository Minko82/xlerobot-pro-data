from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
names=["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
b=FeetechMotorsBus(port="/dev/xle_head", motors={n: Motor(i+1,"sts3215",MotorNormMode.RANGE_M100_100) for i,n in enumerate(names)})
b.connect(handshake=False)
t=b.sync_read("Present_Temperature", normalize=False, num_retry=3)
b.disconnect(disable_torque=False)
print(" ".join(f"{n[:5]}={t[n]}" for n in names))
