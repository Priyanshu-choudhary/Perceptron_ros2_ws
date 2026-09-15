"""Offline check of sensor_guard. No ROS needed."""
import sys

sys.path.insert(0, sys.argv[1])
from perceptron_hardware import sensor_guard as sg

NAN = float('nan')
INF = float('inf')
fails = []
ran = []


def check(label, got, want):
    ok = got == want
    ran.append(label)
    print('%-56s %-8s %s' % (label, got, 'OK' if ok else 'FAIL (want %s)' % want))
    if not ok:
        fails.append(label)


# --- the two fields the EKF actually fuses --------------------------------
check('IMU: NaN gyro z rejected',
      sg.imu_sample_ok(0, 0, 9.8, 0, 0, NAN), False)
check('IMU: inf gyro z rejected',
      sg.imu_sample_ok(0, 0, 9.8, 0, 0, INF), False)
check('odom: NaN linear velocity rejected',
      sg.odom_sample_ok(NAN, 0.0), False)
check('odom: inf angular velocity rejected',
      sg.odom_sample_ok(0.1, INF), False)

# --- corruption that is finite but physically impossible ------------------
check('IMU: 100 rad/s spin spike rejected',
      sg.imu_sample_ok(0, 0, 9.8, 0, 0, 100.0), False)
check('IMU: absurd acceleration rejected',
      sg.imu_sample_ok(0, 0, 9999.0, 0, 0, 0.1), False)
check('odom: 50 m/s wheel velocity rejected',
      sg.odom_sample_ok(50.0, 0.0), False)
check('odom: teleported position rejected',
      sg.odom_sample_ok(0.1, 0.0, 1e9, 0.0, 0.0), False)

# --- real data must survive ------------------------------------------------
# Raw gyro during a real 0.4 rad/s turn at scale 6.28 is only 0.064.
check('IMU: real turn accepted (raw 0.064 rad/s)',
      sg.imu_sample_ok(0.1, 0.2, 9.81, 0.01, 0.01, 0.064), True)
# Saturation is 4.36 rad/s since the firmware moved the gyro to +/-250 dps.
# 8.7 was legal at the old +/-500 range and is corruption now, so it must flip
# from accepted to rejected - if this pair ever disagrees with mpu6050.c's
# GYRO_CONFIG, one of the two has been changed without the other.
check('IMU: near-full-scale 4.3 rad/s accepted',
      sg.imu_sample_ok(0, 0, 9.81, 0, 0, 4.3), True)
check('IMU: 8.7 rad/s (past +/-250 dps saturation) rejected',
      sg.imu_sample_ok(0, 0, 9.81, 0, 0, 8.7), False)
check('IMU: bias-only reading accepted',
      sg.imu_sample_ok(0, 0, 9.81, 0, 0, -0.031), True)
check('odom: top speed 0.3 m/s accepted',
      sg.odom_sample_ok(0.3, 0.5, 12.0, -3.0, 1.57), True)
check('odom: reverse accepted',
      sg.odom_sample_ok(-0.3, -0.5), True)
check('odom: standstill accepted',
      sg.odom_sample_ok(0.0, 0.0), True)

# --- helpers ---------------------------------------------------------------
check('finite() rejects None', sg.finite(None), False)
check('finite() rejects a non-number', sg.finite('garbage'), False)
check('finite() accepts a normal float', sg.finite(1.23), True)

print()
if fails:
    print('FAILURES: %d -> %s' % (len(fails), fails))
    sys.exit(1)
print('all %d checks passed' % len(ran))
