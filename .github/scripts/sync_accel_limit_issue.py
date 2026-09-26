# Keeps the numbers in issue #19 (acceleration limiter robot tests) in sync with the code.
# Every synced number in the issue sits between <!--sync:NAME--> and <!--/sync-->.
import json, os, re, subprocess, sys

ISSUE = "19"
KINEMATICS = "src/main/java/frc/demacia/kinematics/KinematicsConstants.java"
LIMITER = "src/main/java/frc/demacia/kinematics/SwerveAccelerationLimiter.java"


def constant(path, name):
    match = re.search(r"\b" + name + r"\s*=\s*([0-9.]+)\s*;", open(path, encoding="utf-8").read())
    if not match:
        sys.exit(f"{name} not found in {path}")
    return float(match.group(1))


def number(x):
    return f"{x:.2f}".rstrip("0").rstrip(".")


max_module_velocity = constant(KINEMATICS, "MAX_ALLOWED_MODULE_VELOCITY")
max_accel = constant(LIMITER, "MAX_ACCEL")
free_speed = constant(LIMITER, "FREE_SPEED")
max_skid = constant(LIMITER, "MAX_SKID_ACCEL")
max_tilt_front = constant(LIMITER, "MAX_TILT_ACCEL_FRONT")
dt = constant(LIMITER, "MAX_DT")

# straight line from rest, the same limits the limiter uses (one loop at a time)
speed, time = 0.0, 0.0
while speed < max_module_velocity - 1e-9 and time < 30:
    accel = min(max_skid, max_tilt_front, max_accel * max(0.0, 1 - speed / free_speed))
    if accel <= 0:
        break
    speed = min(max_module_velocity, speed + accel * dt)
    time += dt
accel_time = number(time) if speed >= max_module_velocity - 1e-9 else "never (above FREE_SPEED)"
stop_time = number(max_module_velocity / min(max_skid, max_tilt_front))

values = {
    "max-module-velocity": number(max_module_velocity),
    "accel-time": accel_time,
    "stop-time": stop_time,
}
print(values)

body = json.loads(subprocess.check_output(["gh", "issue", "view", ISSUE, "--json", "body"]))["body"]
new_body = body
for name, value in values.items():
    new_body = re.sub(r"(<!--sync:" + name + r"-->).*?(<!--/sync-->)", lambda m: m.group(1) + value + m.group(2), new_body)

if new_body == body:
    print("issue already in sync")
elif "--dry-run" in sys.argv:
    print("would update the issue")
else:
    with open("issue_body.md", "w", encoding="utf-8") as f:
        f.write(new_body)
    subprocess.check_call(["gh", "issue", "edit", ISSUE, "--body-file", "issue_body.md"])
    print("issue updated")
