"""Offline behavioural check of GyroConditioner -- no ROS, no robot needed.

    python3 tools/test_gyro_conditioner.py src/perceptron_hardware

Guards the bug that cost a weekend: jetson_bridge_node published a raw, biased,
unscaled gyro with an all-zero covariance while every fix sat in stm32_bridge_node,
and robot.launch.py defaults to the jetson path. Both bridges now share
GyroConditioner; this pins its behaviour so they cannot drift apart again.
"""
import math
import sys
import types

sys.path.insert(0, sys.argv[1])

import perceptron_hardware.gyro_conditioner as gc

# ---- fake clock so the test is deterministic -------------------------------
CLOCK = {'t': 1000.0}
gc.time = types.SimpleNamespace(time=lambda: CLOCK['t'])


class FakeParam:
    def __init__(self, v):
        self.value = v


class FakeLog:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    def warn(self, m):
        self.lines.append(m)


class FakeNode:
    def __init__(self, **overrides):
        self._p = {}
        self._log = FakeLog()
        self._over = overrides

    def has_parameter(self, n):
        return n in self._p

    def declare_parameter(self, n, d):
        self._p[n] = FakeParam(self._over.get(n, d))

    def get_parameter(self, n):
        return self._p[n]

    def get_logger(self):
        return self._log


def build(**over):
    n = FakeNode(**over)
    gc.GyroConditioner.declare_parameters(n)
    return n, gc.GyroConditioner(n)


def tick(dt=0.01):
    CLOCK['t'] += dt


fails = []


def check(label, got, want, tol=1e-9):
    ok = abs(got - want) <= tol if isinstance(want, float) else got == want
    print('%-58s %-14s %s' % (label, ('%.6f' % got) if isinstance(got, float) else got,
                              'OK' if ok else 'FAIL (want %s)' % want))
    if not ok:
        fails.append(label)


TRUE_BIAS = -0.031          # rad/s, the value their own comment records
SCALE = 6.28

# --- 1. calibration publishes nothing, then seeds the bias -----------------
CLOCK['t'] = 1000.0
node, g = build(gyro_scale_z=SCALE, calibrate_gyro_seconds=5.0)
g.set_wheel_motion(False)
out = g.condition(TRUE_BIAS)
check('during calibration -> publishes nothing', out is None, True)
for _ in range(600):                       # 6 s at 100 Hz
    tick()
    g.set_wheel_motion(False)
    g.condition(TRUE_BIAS)
check('bias seeded from stationary samples', g.bias, TRUE_BIAS, 1e-6)
check('calibration reported complete', g.calibrated, True)

# --- 2. bias removed BEFORE scale ------------------------------------------
# A raw reading of TRUE_BIAS must come out as 0, not as TRUE_BIAS*SCALE.
tick(2.0)
g.set_wheel_motion(True)                   # moving -> ZUPT off
gz, var = g.condition(TRUE_BIAS)
check('stationary-value reading -> zero rate', gz, 0.0, 1e-9)

# A real 0.4 rad/s turn arrives raw as 0.4/6.28 on top of the bias.
raw = TRUE_BIAS + 0.4 / SCALE
tick()
g.set_wheel_motion(True)
gz, var = g.condition(raw)
check('real 0.4 rad/s turn survives bias+scale', gz, 0.4, 1e-6)
check('moving -> normal yaw variance', var, 0.01)

# --- 3. ZUPT engages only after the settle window --------------------------
tick()
g.set_wheel_motion(False)                  # wheels just stopped
gz, var = g.condition(TRUE_BIAS + 0.001)
check('ZUPT not yet engaged during settle', var, 0.01)
tick(0.5)                                  # past zupt_settle_seconds 0.3
g.set_wheel_motion(False)
gz, var = g.condition(TRUE_BIAS + 0.001)
check('ZUPT engaged -> rate forced to zero', gz, 0.0)
check('ZUPT engaged -> variance says trust it', var, 1e-6)

# --- 4. ZUPT disengages on the FIRST sign of motion ------------------------
tick()
g.set_wheel_motion(True)
gz, var = g.condition(TRUE_BIAS + 0.4 / SCALE)
check('ZUPT drops instantly on motion', var, 0.01)
check('start of a real turn is not swallowed', gz, 0.4, 1e-6)

# --- 5. a hand-spin must NOT be absorbed as bias ---------------------------
# Wheels report still (robot lifted) but the gyro sees a genuine 0.4 rad/s.
CLOCK['t'] = 2000.0
node, g = build(gyro_scale_z=SCALE, gyro_bias_z=TRUE_BIAS, zupt_yaw=False)
check('pre-seeded bias skips calibration', g.calibrated, True)
spin_raw = TRUE_BIAS + 0.4 / SCALE         # 0.4 rad/s real, well over bias_max_rate
for _ in range(1000):                      # 10 s of hand-spinning
    tick()
    g.set_wheel_motion(False)
    g.condition(spin_raw)
check('hand-spin not absorbed as bias', g.bias, TRUE_BIAS, 1e-6)

# --- 6. genuine slow bias drift IS tracked ---------------------------------
CLOCK['t'] = 3000.0
node, g = build(gyro_scale_z=SCALE, gyro_bias_z=TRUE_BIAS, zupt_yaw=False,
                bias_time_constant=1.0)
drifted = TRUE_BIAS + 0.004                # 0.004 rad/s -> under bias_max_rate/scale
for _ in range(2000):                      # 20 s parked
    tick()
    g.set_wheel_motion(False)
    g.condition(drifted)
check('slow thermal drift tracked while parked', g.bias, drifted, 1e-4)

# --- 7. covariance is never all-zero ---------------------------------------
cov = g.angular_velocity_covariance(0.01)
check('angular_velocity_covariance[8] populated', cov[8], 0.01)
check('covariance has no zero on the diagonal', all(cov[i] > 0 for i in (0, 4, 8)), True)

print()
if fails:
    print('FAILURES: %d -> %s' % (len(fails), fails))
    sys.exit(1)
print('all %d checks passed' % (16 - len(fails)))
