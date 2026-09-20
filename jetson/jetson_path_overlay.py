#!/usr/bin/env python3
"""Draw the host's projected Nav2 path onto the live camera and stream it out.

    host path_overlay_node.py --ZMQ PULL :5558--> here
    /dev/video0 --GStreamer--> appsink --> cv2 draw --> appsrc --H.265 RTP--> operator

Run this INSTEAD OF letting jetson_robot_bridge.py own the camera:

    python3 jetson_robot_bridge.py --no-aruco          # frees /dev/video0
    python3 jetson_path_overlay.py --host-ip 192.168.1.5

and receive it on the operator machine with the same line you already use:

    gst-launch-1.0 -v udpsrc port=5000 \
      caps="application/x-rtp, media=video, encoding-name=H265, payload=96" \
      ! rtph265depay ! h265parse ! avdec_h265 ! autovideosink sync=false

THIS PROCESS KNOWS NOTHING ABOUT ROS, TF OR THE ROBOT

It receives a list of shapes in pixel coordinates and draws them. Every piece
of geometry -- the transform chain, the intrinsics, the lens model, which Nav2
topic is which -- lives on the host in path_overlay_node.py. Keeping the split
here means this file never needs rebuilding when the robot changes, and it
means a bug in the projection is debuggable on a machine with a debugger on it.

WHY STALENESS IS MEASURED ON THIS MACHINE'S MONOTONIC CLOCK

The obvious way to age an overlay is to compare the host's timestamp against
time.time() here. That is wrong on this pair of machines and would be a very
slow bug to find: the two clocks are independent and undisciplined, which is
the entire reason jetson_bridge_node.py carries a JetsonClock class. A skew of
even a few seconds would make every overlay look either permanently stale or
permanently fresh, depending on which way the drift went.

So no cross-machine arithmetic happens anywhere in this file. The age of an
overlay is time.monotonic() here minus when THIS process received it. The
host's stamp is carried through and displayed, never compared.

WHAT HAPPENS WHEN THE LINK DIES

The overlay ages out after --max-age and the video keeps flowing, unmarked.
The operator sees a clean camera picture rather than a path frozen where the
robot used to be going, which would be actively misleading.
"""

import argparse
import sys
import threading
import time

import cv2
import numpy as np

try:
    import msgpack
except ImportError:
    sys.exit('msgpack missing. Install it with:  pip3 install msgpack')
try:
    import zmq
except ImportError:
    sys.exit('pyzmq missing. Install it with:  pip3 install pyzmq')

WIRE_VERSION = 1

# The CPU path, matching the pipeline already proven on this camera. jpegdec
# and the two videoconverts are the expensive parts; --hw-decode swaps the
# first for the Nano's hardware JPEG block.
# videorate sits BEFORE the decoder in both paths, on the still-compressed
# JPEG stream. The camera offers 30 fps and nothing slower (v4l2-ctl says
# 1280x720 and 1920x1080, 30/1 only), so the halving has to happen somewhere,
# and dropping a JPEG costs nothing while decoding one then throwing it away
# costs a full frame. Measured over 10 s windows, CPU above a 2.4% idle
# baseline on the 4 core Nano:
#
#     jpegdec,        videorate after    15.7%
#     jpegdec,        videorate before   11.9%
#     nvv4l2decoder,  videorate before    0.5%
#
# Hence hardware decode is the default: it is ~30x cheaper than jpegdec on a
# board that is also running the lidar, odometry and IMU bridge. --sw-decode
# is the fallback for a JetPack where the nv element misbehaves.
#
# Do NOT move videorate after nvvidconv with an explicit framerate capsfilter.
# That combination fails to negotiate (-4) because the second capsfilter drops
# the format nvvidconv settled on; `videorate max-rate=15` there does work, but
# it still decodes every frame, which is the cost this ordering avoids.
CAPTURE_HW = (
    'v4l2src device={dev} io-mode=2 ! '
    'image/jpeg,width={w},height={h},framerate={cfps}/1 ! '
    'videorate ! image/jpeg,framerate={ofps}/1 ! '
    'nvv4l2decoder mjpeg=1 ! '
    'nvvidconv ! video/x-raw,format=BGRx ! '
    'videoconvert ! video/x-raw,format=BGR ! '
    'appsink drop=true max-buffers=1 sync=false'
)

CAPTURE_CPU = (
    'v4l2src device={dev} io-mode=2 ! '
    'image/jpeg,width={w},height={h},framerate={cfps}/1 ! '
    'videorate ! image/jpeg,framerate={ofps}/1 ! '
    'jpegdec ! '
    'videoconvert ! video/x-raw,format=BGR ! '
    'appsink drop=true max-buffers=1 sync=false'
)

CAPTURE_TEST = (
    'videotestsrc pattern=smpte is-live=true ! '
    'video/x-raw,width={w},height={h},framerate={ofps}/1 ! '
    'videoconvert ! video/x-raw,format=BGR ! '
    'appsink drop=true max-buffers=1 sync=false'
)

# config-interval=1 is the one real change from the hand-rolled pipeline: it
# repeats SPS/PPS every second so a receiver started AFTER the stream still
# gets a decodable picture, instead of waiting for the next IDR's parameter
# sets and showing green mush until then.
OUTPUT = (
    'appsrc is-live=true do-timestamp=true block=true format=TIME ! '
    'video/x-raw,format=BGR ! '
    'videoconvert ! video/x-raw,format=I420 ! '
    'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
    'nvv4l2h265enc bitrate={bitrate} control-rate=1 maxperf-enable=1 '
    'preset-level=1 iframeinterval={ofps} insert-sps-pps=1 ! '
    'h265parse ! rtph265pay pt=96 config-interval=1 ! '
    'udpsink host={host} port={port} sync=false async=false'
)

# Software fallback for a board where the nv* elements are missing or the
# encoder refuses to negotiate. Much slower, but it proves the rest works.
OUTPUT_SW = (
    'appsrc is-live=true do-timestamp=true block=true format=TIME ! '
    'video/x-raw,format=BGR ! '
    'videoconvert ! video/x-raw,format=I420 ! '
    'x265enc bitrate={kbitrate} speed-preset=ultrafast tune=zerolatency ! '
    'h265parse ! rtph265pay pt=96 config-interval=1 ! '
    'udpsink host={host} port={port} sync=false async=false'
)


def say(*parts):
    """print() that actually reaches a pipe.

    Under nohup, systemd or `ssh host cmd` stdout is not a tty, so Python
    block-buffers it and none of the status below appears until the process
    exits -- which is exactly when it stops being useful.
    """
    print(*parts, flush=True)


def rgb_to_bgr(rgb):
    """Wire format is RGB so it reads the same as the host's parameters."""
    try:
        r, g, b = (int(c) for c in rgb)
    except Exception:
        return (255, 255, 255)
    return (max(0, min(255, b)), max(0, min(255, g)), max(0, min(255, r)))


class OverlayReceiver:
    """Binds PULL and keeps only the newest drawing.

    Newest-only is deliberate. An overlay is a snapshot of where the path was,
    so a queued one is not useful work waiting to be done, it is a picture of
    the past. Draining to the last message matches what aruco_localizer_node
    does with its observation inbox and for the same reason.
    """

    def __init__(self, port, bind_addr='0.0.0.0'):
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.PULL)
        self.sock.setsockopt(zmq.RCVHWM, 2)
        self.sock.setsockopt(zmq.RCVTIMEO, 500)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.url = f'tcp://{bind_addr}:{port}'
        self.sock.bind(self.url)

        self._lock = threading.Lock()
        self._payload = None
        self._rx_monotonic = 0.0
        self.received = 0
        self.bad = 0

        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _worker(self):
        while self.running:
            try:
                raw = self.sock.recv()
            except zmq.Again:
                continue
            except Exception:
                if not self.running:
                    break
                continue
            try:
                payload = msgpack.unpackb(raw, raw=False)
            except Exception:
                self.bad += 1
                continue
            if not isinstance(payload, dict) or payload.get('v') != WIRE_VERSION:
                self.bad += 1
                continue
            with self._lock:
                self._payload = payload
                self._rx_monotonic = time.monotonic()
                self.received += 1

    def latest(self, max_age):
        """Newest payload, or None once it has aged past max_age."""
        with self._lock:
            if self._payload is None:
                return None, 0.0
            age = time.monotonic() - self._rx_monotonic
            if age > max_age:
                return None, age
            return self._payload, age

    def close(self):
        self.running = False
        try:
            self.thread.join(timeout=1.0)
            self.sock.close()
            self.ctx.term()
        except Exception:
            pass


class Renderer:
    """Draws wire shapes onto a BGR frame, rescaling from calibration size."""

    def __init__(self, show_hud=True):
        self.show_hud = show_hud
        self._scale = (1.0, 1.0)
        self._ref = None

    def _set_ref(self, ref, shape):
        h, w = shape[:2]
        if not ref or len(ref) != 2 or ref[0] <= 0 or ref[1] <= 0:
            self._scale = (1.0, 1.0)
            return
        # The host always projects at the resolution the camera was calibrated
        # at (1280x720). Encoding at anything else is a local decision here, so
        # the rescale belongs here too -- that way --proc-width needs no
        # matching change on the host.
        self._scale = (w / float(ref[0]), h / float(ref[1]))
        self._ref = tuple(ref)

    def _pts(self, raw):
        sx, sy = self._scale
        a = np.asarray(raw, dtype=np.float64)
        if a.ndim != 2 or a.shape[0] < 2 or a.shape[1] != 2:
            return None
        a[:, 0] *= sx
        a[:, 1] *= sy
        return np.rint(a).astype(np.int32)

    def _pt(self, raw):
        sx, sy = self._scale
        try:
            return (int(round(raw[0] * sx)), int(round(raw[1] * sy)))
        except Exception:
            return None

    def draw(self, frame, payload, age):
        self._set_ref(payload.get('ref'), frame.shape)

        polygons = payload.get('polygons') or []
        # One frame copy for every translucent polygon, blended once. Copying
        # per polygon would be 2.7 MB of memcpy each at 720p, which this board
        # does not have to spare at 15 fps.
        if polygons:
            overlay = frame.copy()
            alpha = 0.0
            drew = False
            for poly in polygons:
                pts = self._pts(poly.get('pts'))
                if pts is None:
                    continue
                cv2.fillPoly(overlay, [pts], rgb_to_bgr(poly.get('rgb')))
                alpha = max(alpha, float(poly.get('alpha', 0.3)))
                drew = True
            if drew:
                alpha = max(0.0, min(1.0, alpha))
                cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0.0, dst=frame)

        for line in payload.get('polylines') or []:
            pts = self._pts(line.get('pts'))
            if pts is None:
                continue
            cv2.polylines(frame, [pts], False, rgb_to_bgr(line.get('rgb')),
                          int(line.get('w', 2)), lineType=cv2.LINE_AA)

        for circ in payload.get('circles') or []:
            c = self._pt(circ.get('c'))
            if c is None:
                continue
            cv2.circle(frame, c, int(circ.get('r', 6)), rgb_to_bgr(circ.get('rgb')),
                       int(circ.get('w', 2)), lineType=cv2.LINE_AA)

        for lab in payload.get('labels') or []:
            p = self._pt(lab.get('p'))
            if p is None:
                continue
            text = str(lab.get('t', ''))
            scale = float(lab.get('s', 0.55))
            colour = rgb_to_bgr(lab.get('rgb'))
            # Black underlay first: a thin coloured glyph over a bright floor
            # or a white wall is unreadable, and this view is for an operator.
            cv2.putText(frame, text, p, cv2.FONT_HERSHEY_SIMPLEX, scale,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, text, p, cv2.FONT_HERSHEY_SIMPLEX, scale,
                        colour, 1, cv2.LINE_AA)

        if self.show_hud:
            y = 24
            hud = list(payload.get('hud') or [])[:6]
            # An overlay that is merely old rather than dead still gets drawn,
            # but the operator is told, because a lagging path over a moving
            # robot is the one case where the picture lies convincingly.
            if age > 0.35:
                hud.append(f'overlay lagging {age * 1000:.0f} ms')
            for text in hud:
                cv2.putText(frame, str(text), (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(frame, str(text), (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 1, cv2.LINE_AA)
                y += 24
        return frame


def build_args():
    ap = argparse.ArgumentParser(
        description='Draw the host-projected Nav2 path on the camera and stream H.265.')
    ap.add_argument('--host-ip', default='192.168.1.5',
                    help='where to send the RTP stream (the operator machine)')
    ap.add_argument('--port', type=int, default=5000, help='RTP udpsink port')
    ap.add_argument('--zmq-port', type=int, default=5558,
                    help='PULL port this process binds for overlay data')
    ap.add_argument('--device', default='/dev/video0')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--capture-fps', type=int, default=30,
                    help='what the camera is asked for')
    ap.add_argument('--fps', type=int, default=15,
                    help='what is drawn on and encoded')
    ap.add_argument('--bitrate', type=int, default=500000, help='H.265 bits/s')
    ap.add_argument('--max-age', type=float, default=0.7,
                    help='seconds before an overlay is dropped as stale')
    ap.add_argument('--sw-decode', action='store_true',
                    help='CPU jpegdec instead of the default hardware decoder '
                         '(~30x more CPU; for boards where nvv4l2decoder misbehaves)')
    ap.add_argument('--sw-encode', action='store_true',
                    help='fall back to x265enc when the nv encoder will not open')
    ap.add_argument('--test-src', action='store_true',
                    help='videotestsrc instead of the camera, to prove the link')
    ap.add_argument('--no-hud', action='store_true')
    ap.add_argument('--stats-interval', type=float, default=10.0)
    ap.add_argument('--print-pipelines', action='store_true',
                    help='print both pipelines and exit')
    return ap


def main():
    args = build_args().parse_args()

    if args.test_src:
        cap_desc = CAPTURE_TEST.format(w=args.width, h=args.height, ofps=args.fps)
    else:
        template = CAPTURE_CPU if args.sw_decode else CAPTURE_HW
        cap_desc = template.format(dev=args.device, w=args.width, h=args.height,
                                   cfps=args.capture_fps, ofps=args.fps)

    out_template = OUTPUT_SW if args.sw_encode else OUTPUT
    out_desc = out_template.format(bitrate=args.bitrate,
                                   kbitrate=max(1, args.bitrate // 1000),
                                   ofps=args.fps, host=args.host_ip, port=args.port)

    if args.print_pipelines:
        say('CAPTURE:\n  ' + cap_desc + '\n\nOUTPUT:\n  ' + out_desc)
        return 0

    if not cv2.getBuildInformation().count('GStreamer'):
        print('WARNING: this OpenCV may lack GStreamer support', file=sys.stderr)

    cap = cv2.VideoCapture(cap_desc, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print('Capture pipeline would not open. Tried:\n  ' + cap_desc,
              file=sys.stderr)
        print('\nIs something else holding the camera? jetson_robot_bridge.py must '
              'be started with --no-aruco, and a previous run can outlive the ssh '
              'session that started it. Retry with --sw-decode to rule out the '
              'hardware JPEG decoder.', file=sys.stderr)
        return 1

    ok, frame = cap.read()
    if not ok or frame is None:
        print('Capture opened but produced no frame.', file=sys.stderr)
        cap.release()
        return 1
    h, w = frame.shape[:2]
    say(f'Camera up: {w}x{h} @ {args.fps} fps')

    writer = cv2.VideoWriter(out_desc, cv2.CAP_GSTREAMER, 0,
                             float(args.fps), (w, h), True)
    if not writer.isOpened():
        print('Encoder pipeline would not open. Tried:\n  ' + out_desc,
              file=sys.stderr)
        print('\nRetry with --sw-encode to rule out the hardware encoder.',
              file=sys.stderr)
        cap.release()
        return 1
    say(f'Streaming H.265 to {args.host_ip}:{args.port}')

    receiver = OverlayReceiver(args.zmq_port)
    say(f'Listening for overlay data on {receiver.url}')

    renderer = Renderer(show_hud=not args.no_hud)

    frames = 0
    drawn = 0
    repeated = 0
    last_stats = time.monotonic()
    last_seq = None

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print('Camera returned no frame; stopping.', file=sys.stderr)
                break
            frames += 1

            payload, age = receiver.latest(args.max_age)
            if payload is not None:
                seq = payload.get('seq')
                # The host projects at 15 Hz and so does this loop, but they
                # free-run against each other. Redrawing the same seq is normal;
                # a high ratio means the host or the link is behind.
                if seq == last_seq:
                    repeated += 1
                last_seq = seq
                renderer.draw(frame, payload, age)
                drawn += 1

            writer.write(frame)

            now = time.monotonic()
            if now - last_stats >= args.stats_interval:
                span = now - last_stats
                say(f'{frames / span:5.1f} fps out | {drawn} of {frames} frames '
                      f'overlaid | {receiver.received} payloads | {repeated} repeats'
                      + (f' | {receiver.bad} malformed' if receiver.bad else ''))
                frames = drawn = repeated = 0
                receiver.received = 0
                last_stats = now
    except KeyboardInterrupt:
        say('\nStopping.')
    finally:
        receiver.close()
        cap.release()
        writer.release()
    return 0


if __name__ == '__main__':
    sys.exit(main())
