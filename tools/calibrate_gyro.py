#!/usr/bin/env python3
"""Calibrate the gyro's zero-rate bias and scale factor.

Run with the ROS stack STOPPED -- this talks to the ECU directly, and two
readers on one serial port each get half the bytes.

    python3 tools/calibrate_gyro.py --port /dev/ttyUSB0

WHY BOTH NUMBERS, AND WHY ONLY ONE OF THEM IS WORTH RE-MEASURING

  bias   The reading when not turning. It is NOT constant: measured on this
         unit across three runs it was -0.047, +0.006 and +0.038 rad/s, a
         5 deg/s spread, because MEMS bias walks with temperature and time.
         So a number written into a config file goes stale. stm32_bridge_node
         re-measures it at every boot and keeps tracking it while stationary.
         Phase 1 exists to show you how large and how unstable it is, not to
         produce a value you paste anywhere.

  scale  The LSB-per-dps conversion inside the firmware. This one IS a
         constant -- it is arithmetic, not physics, and does not care about
         wheel slip, temperature or load. If it is wrong, every turn is
         reported in the wrong units and no amount of fusion can fix it,
         because the EKF has exactly one source of yaw rate to compare
         against. Measure it once, write it down, done.

THE REFERENCE IS THE WALL, NOT THE WHEELS

Wheel-derived yaw is useless as a reference on a skid-steer: the tyres scrub
sideways through every turn, so the encoders disagree with reality by an amount
that changes with surface and speed. This tool therefore never uses them. The
reference is a physical angle you set up yourself: align the chassis against a
straight edge, turn through a whole number of full rotations, and come back to
the same edge. Three rotations rather than one because the alignment error at
each end is fixed, so spreading it over 1080 degrees cuts its effect threefold.

    gyro_scale_z = true_angle / gyro_integrated_angle

Below 1.0 means the gyro over-reports -- the map turns further than the robot.

Put the result in config/hardware_params.yaml as gyro_scale_z. Leave
gyro_bias_z at 0.0 so the node keeps finding it itself.
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit('pyserial is missing: pip3 install pyserial')

TEL_HDR, TEL_SIZE, TEL_FMT = 0x55, 39, '<BIHhiiiifffBB'
IMU_HDR, IMU_SIZE, IMU_FMT = 0x56, 31, '<BIffffffBB'
END = 0x0A
RAD2DEG = 57.29577951308232
IMU_DT = 0.01          # the ECU's own 100 Hz tick
TRACE_PATH = '/tmp/gyro_trace.txt'


def xorsum(data):
    c = 0
    for b in bytearray(data):
        c ^= b
    return c


class Reader(object):
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.ser.reset_input_buffer()
        self.buf = bytearray()

    def poll(self):
        """Decode whatever has arrived; returns a list of gz samples."""
        self.buf.extend(self.ser.read(max(1, self.ser.in_waiting)))
        gz = []
        consumed = i = 0
        while i < len(self.buf):
            h = self.buf[i]
            size = TEL_SIZE if h == TEL_HDR else (IMU_SIZE if h == IMU_HDR else 0)
            if size:
                if i + size > len(self.buf):
                    break
                f = bytes(self.buf[i:i + size])
                if f[-1] == END and f[-2] == xorsum(f[:-2]):
                    if h == IMU_HDR:
                        gz.append(struct.unpack(IMU_FMT, f)[7])
                    i += size
                    consumed = i
                    continue
            i += 1
            consumed = i
        del self.buf[:consumed]
        return gz

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def phase_bias(r, seconds):
    print('\n=== PHASE 1: ZERO-RATE BIAS ===')
    print('Robot completely still on the ground. Do not touch it for %.0f s.'
          % seconds)
    for i in range(3, 0, -1):
        print('  starting in %d...' % i)
        time.sleep(1)

    r.poll()
    samples = []
    windows = []
    win = []
    t0 = last = time.time()
    while time.time() - t0 < seconds:
        got = r.poll()
        samples.extend(got)
        win.extend(got)
        now = time.time()
        if now - last >= 2.0 and win:
            windows.append(sum(win) / len(win))
            print('  %4.0fs  running mean %+.5f rad/s (%+.3f deg/s)  n=%d'
                  % (now - t0, sum(samples) / len(samples),
                     sum(samples) / len(samples) * RAD2DEG, len(samples)))
            win = []
            last = now
        time.sleep(0.01)

    if not samples:
        sys.exit('no IMU frames decoded -- is the ROS stack still running?')

    bias = sum(samples) / len(samples)
    std = (sum((g - bias) ** 2 for g in samples) / len(samples)) ** 0.5
    print('\n  bias  %+.6f rad/s  (%+.4f deg/s)  from %d samples'
          % (bias, bias * RAD2DEG, len(samples)))
    print('  noise %.6f rad/s std' % std)
    if len(windows) > 1:
        spread = (max(windows) - min(windows)) * RAD2DEG
        print('  drift %.4f deg/s between 2 s windows' % spread)
        print('  -> %s' % ('the offset moves even over this short run; keep '
                           'track_gyro_bias enabled' if spread > 0.3 else
                           'stable over this run, but it still walks between '
                           'boots -- keep track_gyro_bias enabled'))
    return bias


def phase_scale(r, bias, rotations):
    true_deg = 360.0 * rotations
    print('\n=== PHASE 2: SCALE FACTOR ===')
    print('  1. Align the chassis against a wall or straight edge.')
    print('  2. Mark the position so you can return to it exactly.')
    print('  3. Press Enter, turn the robot through %d FULL rotations'
          % rotations)
    print('     (%.0f degrees), realign against the same edge, press Enter.'
          % true_deg)
    print('     Direction does not matter. Speed does not matter.')
    print('')
    print('  IMPORTANT: do not stop recording until the chassis is actually')
    print('  back on the mark. If you drive the turn with teleop, the angular')
    print('  speed ceiling applies -- at 0.5 rad/s (29 deg/s) a %.0f degree'
          % true_deg)
    print('  turn takes at least %.0f seconds. Turning it by hand is quicker'
          % (true_deg / 28.6))
    print('  and just as valid; the reference is the mark, not the motors.')
    print('\nPress Enter to start recording.')
    try:
        input()
    except EOFError:
        sys.exit('needs an interactive terminal for the Enter prompts')

    import select
    r.poll()                      # discard what queued while you were reading
    angle = 0.0
    n = 0
    peak = 0.0
    trace = []
    t0 = time.time()
    print('recording... turn now, then press Enter')
    while True:
        for g in r.poll():
            c = g - bias
            peak = max(peak, abs(c))
            angle += c * IMU_DT   # ECU tick, not host timing
            trace.append(c)
            n += 1
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break
        time.sleep(0.01)

    # Keep the raw trace. When the integrated angle disagrees with the angle
    # you know you turned, the shape of the rate over time is what settles it:
    # three rotations look like a long sustained plateau, a clipped recording
    # looks like a plateau with an end missing.
    try:
        with open(TRACE_PATH, 'w') as fh:
            fh.write('# bias_corrected_gz_rad_s, one sample per 10 ms\n')
            for c in trace:
                fh.write('%.6f\n' % c)
        print('  raw rate trace saved to %s' % TRACE_PATH)
    except Exception as exc:
        print('  (could not save trace: %s)' % exc)

    measured = angle * RAD2DEG
    elapsed = time.time() - t0
    peak_deg = peak * RAD2DEG
    needed_avg = true_deg / elapsed if elapsed > 0 else 0.0

    print('\n  recorded %.1f s, %d samples, peak rate %.3f rad/s (%.1f deg/s)'
          % (elapsed, n, peak, peak_deg))
    print('  gyro integrated : %+.2f deg' % measured)
    print('  true angle      : %+.2f deg' % true_deg)
    print('  implied average : %.1f deg/s over the window' % needed_avg)

    if abs(measured) < 30.0:
        print('\n  the gyro barely moved -- did the turn happen while '
              'recording was running?')
        return None

    # Sanity gate. A turn cannot average faster than its own peak rate, so if
    # the claimed angle needs a higher average than the gyro ever saw, the
    # claimed angle did not happen -- the run was cut short, or the robot did
    # not complete the rotations. Without this the tool happily reports a huge
    # scale factor, and applying it would make the heading wildly wrong in the
    # opposite direction.
    if needed_avg > peak_deg:
        print('\n  IMPOSSIBLE: %.0f deg in %.1f s needs %.1f deg/s average, '
              'but the peak\n  rate seen was only %.1f deg/s. A turn cannot '
              'average faster than its\n  peak, so the robot did not turn '
              '%.0f deg during the recording.'
              % (true_deg, elapsed, needed_avg, peak_deg, true_deg))
        print('\n  Most likely the recording stopped early, or the speed '
              'ceiling made the\n  turn impossible in the time available. At '
              'the peak rate observed,\n  %.0f deg needs at least %.0f s.'
              % (true_deg, true_deg / peak_deg))
        print('  Re-run and keep recording until the chassis is back on the '
              'mark.')
        return None

    # Even below that hard limit, needing most of the peak as the average means
    # the turn had no acceleration or pauses, which no real turn does.
    if needed_avg > 0.85 * peak_deg:
        print('\n  SUSPICIOUS: the claimed angle needs %.1f deg/s average '
              'against a\n  %.1f deg/s peak. Real turns accelerate and '
              'decelerate, so the average\n  should be well below the peak. '
              'Treat this result with suspicion.'
              % (needed_avg, peak_deg))

    # Sign is the turn direction, which is not what we are measuring.
    scale = true_deg / abs(measured)
    print('\n  gyro_scale_z = %.4f' % scale)
    err = (1.0 / scale - 1.0) * 100.0
    if abs(scale - 1.0) < 0.03:
        print('  -> within 3%%. Scale is fine, leave gyro_scale_z at 1.0.')
        print('     The heading problem is bias, not scale.')
    elif scale < 1.0:
        print('  -> gyro OVER-reports by %.0f%%. This is what makes the map '
              'turn further than the robot.' % err)
    else:
        print('  -> gyro UNDER-reports by %.0f%%.' % (-err))
    return scale


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyUSB0',
                    help='ECU serial port (run detect_ports if unsure)')
    ap.add_argument('--bias-seconds', type=float, default=10.0,
                    help='stationary sampling window (default 10 s = ~1000 samples)')
    ap.add_argument('--rotations', type=int, default=3,
                    help='full turns for the scale phase (default 3)')
    ap.add_argument('--skip-scale', action='store_true')
    args = ap.parse_args()

    r = Reader(args.port)
    try:
        bias = phase_bias(r, args.bias_seconds)
        scale = None if args.skip_scale else phase_scale(r, bias, args.rotations)
    finally:
        r.close()

    print('\n=== RESULT ===')
    if scale:
        print('config/hardware_params.yaml:')
        print('    gyro_scale_z: %.4f' % scale)
    print('\nLeave gyro_bias_z at 0.0. The measured bias (%+.5f rad/s) is only '
          'valid\nfor right now -- the node re-measures it every boot and keeps '
          'tracking it.' % bias)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
