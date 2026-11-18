#!/usr/bin/env python3
import argparse
import collections
import math
import os
import sys
import time

import numpy as np
import cv2
import torch
from ultralytics import YOLO

HOLD_GAP = 0.3  # seconds the die may leave the circle or go undetected during the hold
DIE_SIDE_METERS = 0.1524  # real side length of the die (6 in)
CAMERA_FOV_DEGREES = 82.0  # Arducam AR0234 horizontal field of view, from the datasheet
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(1, os.path.join(HERE, "modules"))

# Only needed with a drone connected; --bench runs without it.
try:
    import control
except ImportError:
    control = None


def parse_args():
    """Parse command-line options for camera, tracking, and flight settings."""
    parser = argparse.ArgumentParser(description="Visually center and track a die")
    parser.add_argument("--mode", choices=("test", "flight"), default="test")
    parser.add_argument("--debug_path", default="debug/dice_tracker")
    parser.add_argument("--control", choices=("PID", "P"), default="PID")
    parser.add_argument("--altitude", type=float, default=1.5)
    parser.add_argument("--weights", default=os.path.join(HERE, "models", "dice_v3.pt"))
    parser.add_argument("--target", help="die to track: black_dice, blue_dice, yellow_dice or red_dice")
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument("--source", default="0", help="camera index, or a video file for testing")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1200)
    parser.add_argument("--margin", type=int, default=60, help="radius in pixels of the center circle counted as aligned")
    parser.add_argument("--hold", type=float, default=0.5, help="seconds the die must stay inside the circle to count as aligned")
    parser.add_argument("--bench", action="store_true", help="no drone: only show the detection and centering error")
    parser.add_argument("--show-commands", action="store_true", help="print every command sent to the drone")
    parser.add_argument("--dry-run", action="store_true",
                        help="real camera, pretend drone: see the commands while moving a die by hand")
    parser.add_argument("--sim", action="store_true", help="no drone or camera: fly a simulated drone over simulated dice")
    parser.add_argument("--sim-offset", type=float, nargs=2, default=(0.4, -0.5), metavar=("FORWARD", "RIGHT"),
                        help="where the target die starts, in m from the drone")
    parser.add_argument("--sim-drift", type=float, default=0.03, help="m/s the simulated drone may drift")
    parser.add_argument("--sim-latency", type=float, default=0.1, help="seconds of simulated camera delay")
    return parser.parse_args()


def open_camera(source, width, height):
    """Open the Arducam (a USB camera) or a video file with OpenCV."""
    camera = cv2.VideoCapture(int(source) if source.isdigit() else source)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return camera


def find_dice(model, image, target, conf, device):
    """Find every die of the target color in a BGR frame.

    Return a list of ``(area, center, bounds, label, confidence)``. ``center`` is an
    ``(x, y)`` pixel coordinate; ``bounds`` is ``(x, y, width, height)``. With
    ``target=None``, every die color matches.
    """
    boxes = model.predict(image, conf=conf, imgsz=640, device=device, verbose=False)[0].boxes
    candidates = []

    for xyxy, class_id, confidence in zip(boxes.xyxy.tolist(), boxes.cls.tolist(), boxes.conf.tolist()):
        label = model.names[int(class_id)]
        if target is not None and label != target:
            continue

        x1, y1, x2, y2 = [int(value) for value in xyxy]
        width, height = x2 - x1, y2 - y1
        center = (x1 + width // 2, y1 + height // 2)
        candidates.append((width * height, center, (x1, y1, width, height), label, confidence))

    return candidates


DieMeasure = collections.namedtuple("DieMeasure", "corners angle area source rotated_area")


def die_blob_contour(frame, bounds, clusters=3, work_size=64):
    """Return the outline of the die's main single-color region, in full-frame pixels.

    No color ranges are used. The YOLO box's pixels are grouped into a few color
    clusters (k-means in Lab space), the cluster at the center of the box is taken as
    the die, and the outline of its largest connected region is returned. Return None
    if the box is too small or no region is found. Call this on a clean frame, before
    anything is drawn on it.
    """
    x, y, width, height = bounds
    x1, y1 = max(x, 0), max(y, 0)
    x2, y2 = min(x + width, frame.shape[1]), min(y + height, frame.shape[0])
    crop = frame[y1:y2, x1:x2]
    if crop.shape[0] < 8 or crop.shape[1] < 8:
        return None

    # Cluster on a shrunken copy for speed, then stretch the labels back to full size.
    scale = work_size / max(crop.shape[:2])
    small = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else crop
    lab = cv2.cvtColor(small, cv2.COLOR_BGR2LAB).reshape(-1, 3).astype(np.float32)
    if len(lab) < clusters:
        return None

    cv2.setRNGSeed(0)  # same frame gives the same clusters every time
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    _, labels, _ = cv2.kmeans(lab, clusters, None, criteria, 3, cv2.KMEANS_PP_CENTERS)
    labels = labels.reshape(small.shape[:2]).astype(np.uint8)
    labels = cv2.resize(labels, (crop.shape[1], crop.shape[0]), interpolation=cv2.INTER_NEAREST)

    # The box is drawn around the die, so the cluster most common in its middle half is the die.
    rows, cols = labels.shape
    middle = labels[rows // 4: 3 * rows // 4, cols // 4: 3 * cols // 4]
    die_label = np.bincount(middle.ravel()).argmax()

    mask = (labels == die_label).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    return contour + np.array([x1, y1], dtype=np.int32)  # crop coordinates -> frame coordinates


def measure_die(frame, box_area, bounds, tolerance=1.15, minimum=0.4):
    """Fit a rotated rectangle to the die's color block, or fall back to the YOLO box.

    Return a DieMeasure:
      corners       4x2 int array of the rotated rectangle (None when falling back)
      angle         degrees from vertical in [-45, 45), positive = clockwise on screen
                    (None when falling back)
      area          pixel area in use: the rotated rectangle's, or the YOLO box's
      source        "rotated" or "yolo"
      rotated_area  the rotated rectangle's area even if it was rejected (None if
                    it could not be measured)

    The YOLO box is used if anything goes wrong, if the rotated rectangle's area is
    more than ``tolerance`` times the YOLO box area (15% over by default), or if it is
    less than ``minimum`` times the YOLO box area (40% by default).
    """
    rotated_area = None
    try:
        contour = die_blob_contour(frame, bounds)
        if contour is not None:
            rect = cv2.minAreaRect(contour)
            rotated_area = int(rect[1][0] * rect[1][1])
            if rotated_area > 0 and minimum * box_area <= rotated_area <= tolerance * box_area:
                corners = cv2.boxPoints(rect)
                edge = corners[1] - corners[0]
                # A die is square, so its tilt repeats every 90 degrees: fold into [-45, 45).
                angle = (math.degrees(math.atan2(edge[1], edge[0])) + 45) % 90 - 45
                return DieMeasure(corners.astype(np.int32), angle, rotated_area, "rotated", rotated_area)
    except Exception:
        pass  # any failure in the measurement falls back to the YOLO box
    return DieMeasure(None, None, box_area, "yolo", rotated_area)


def estimate_height_meters(die_area_px, image_width):
    """Estimate the camera's height above a flat die from the die's size in the image.

    ``die_area_px`` is the rotated-box area, or the YOLO box area when the color-block
    fit fell back. Return None if the area is zero. Assumes the die is flat and uses
    the horizontal field of view only.
    """
    side_length_px = math.sqrt(die_area_px)
    if side_length_px <= 0:
        return None
    ground_meters = DIE_SIDE_METERS * image_width / side_length_px  # width of ground in view
    width_degrees = CAMERA_FOV_DEGREES
    height_meters = (ground_meters / 2) / math.tan(math.radians(width_degrees / 2))
    return height_meters


def close_resources(camera, writer):
    """Optional video resources, camera, display windows, and debug logs."""
    if writer is not None:
        writer.release()
    if camera is not None:
        camera.release()
    cv2.destroyAllWindows()
    if control is not None:
        for log_file in (control.debug_yaw, control.debug_velocity):
            if log_file is not None:
                log_file.close()


def main():
    """Initialize the camera and drone, track the die, then land and clean up."""
    args = parse_args()
    if not args.bench and control is None:
        raise SystemExit("Could not import control. Install requirements.txt, or use --bench to test without a drone.")
    if not args.bench and args.target is None:
        raise SystemExit("--target is required when a drone is connected")
    if (args.sim or args.dry_run) and (args.bench or args.mode != "test"):
        raise SystemExit("--sim and --dry-run run in --mode test without --bench")
    if args.sim and args.dry_run:
        raise SystemExit("Choose --sim or --dry-run, not both")

    model = YOLO(args.weights)
    if args.target is not None and args.target not in model.names.values():
        raise SystemExit("Unknown --target {}. Choose from {}".format(args.target, list(model.names.values())))
    device = "mps" if torch.backends.mps.is_available() else 0 if torch.cuda.is_available() else "cpu"

    camera = None
    writer = None
    simulator = None
    aligned_reported = False
    airborne = False
    landed = False

    try:
        if args.sim or args.dry_run:
            import sim
            simulator = sim.Simulator(args.target, args.width, args.height, tuple(args.sim_offset),
                                      args.sim_drift, args.sim_latency)
        if args.sim:
            camera = simulator.camera
        else:
            camera = open_camera(args.source, args.width, args.height)
        if not camera.isOpened():
            raise SystemExit("Could not open camera {}".format(args.source))
        image_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        image_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print("Camera {}: {}x{}".format("simulated" if args.sim else args.source, image_width, image_height))
        image_center_x = image_width // 2
        image_center_y = image_height // 2
        x_errors = collections.deque(maxlen=5)
        y_errors = collections.deque(maxlen=5)
        inside_since = None
        last_inside = None

        debug_dir = os.path.dirname(args.debug_path)
        if debug_dir:
            os.makedirs(debug_dir, exist_ok=True)

        if args.mode == "flight":
            connection = "/dev/ttyACM0"
            writer = cv2.VideoWriter(
                args.debug_path + ".avi",
                cv2.VideoWriter_fourcc(*"MJPG"),
                25.0,
                (image_width, image_height),
            )
        else:
            connection = "127.0.0.1:14551"

        if not args.bench:
            control.show_commands = args.show_commands
            if args.dry_run:
                control.connection = simulator.connection
                print("Dry run: pretend drone, nothing is sent to a real drone. Move the die under the camera.")
            elif args.sim:
                control.connection = simulator.connection
                print("Simulated drone; target {} starts {} m forward, {} m right".format(args.target, *args.sim_offset))
            else:
                control.connect_drone(connection)
            control.configure_PID(args.control)
            control.set_flight_altitude(args.altitude)
            control.initialize_debug_logs(args.debug_path)

            print("Dice tracking: {}; sideways and forward/back correction".format(args.target))
            control.print_drone_report()
            control.arm_and_takeoff(args.altitude)
            airborne = True
        takeoff_time = time.time()

        while True:
            ok, frame = camera.read()
            if not ok:
                if args.bench or not (args.sim or args.source.isdigit()):
                    print("End of video")
                    break
                control.stop_drone()
                time.sleep(0.02)
                continue

            candidates = find_dice(model, frame, args.target, args.conf, device)

            # Measure on the clean frame, before anything is drawn on it.
            die_measures = [measure_die(frame, box_area, bounds)
                            for box_area, _, bounds, _, _ in candidates]

            # The most confident die is the one the error and drone commands follow.
            detection = max(candidates, key=lambda candidate: candidate[4]) if candidates else None
            forward_speed, right_speed = 0.0, 0.0
            error_text = "dx: -  dy: -"
            inside = False

            if detection is None:
                x_errors.clear()
                y_errors.clear()
                if not args.bench:
                    control.stop_drone()
                status = "No die detected; holding position"
            else:
                area, center, bounds, label, confidence = detection
                x_errors.append(center[0] - image_center_x)
                y_errors.append(center[1] - image_center_y)
                average_error = sum(x_errors) / len(x_errors)
                average_y_error = sum(y_errors) / len(y_errors)
                inside = (average_error ** 2 + average_y_error ** 2) ** 0.5 <= args.margin

                if not args.bench:
                    if inside:
                        control.stop_drone()
                    else:
                        control.setXdelta(average_error)
                        control.setYdelta(average_y_error)
                        control.control_drone()
                    forward_speed, right_speed = control.getVelocity()
                cv2.line(frame, (image_center_x, image_center_y), center, (0, 255, 255), 2)
                status = "{} {:.2f}, area: {} px".format(label, confidence, int(area))
                error_text = "dx: {:.0f}  dy: {:.0f}".format(average_error, average_y_error)

            # A flying drone drifts and misses frames, so short gaps don't restart the hold.
            now = time.time()
            if inside:
                last_inside = now
                if inside_since is None:
                    inside_since = now
            elif inside_since is not None and now - last_inside > HOLD_GAP:
                inside_since = None
            aligned = inside_since is not None and last_inside - inside_since >= args.hold
            if aligned:
                error_text += "  ALIGNED"
                if simulator is not None and not aligned_reported:
                    forward, right = simulator.target_offset()
                    print("ALIGNED {:.1f} s after takeoff; really {:.0f} cm off (forward {:+.0f}, right {:+.0f})".format(
                        time.time() - takeoff_time, 100 * (forward ** 2 + right ** 2) ** 0.5, 100 * forward, 100 * right))
                aligned_reported = True
            elif inside_since is None:
                aligned_reported = False
            elif inside_since is not None:
                error_text += "  HOLDING"

            # Draw each die: a rotated box with its tilt and area, or the YOLO box if the fit failed.
            for (_, die_center, (x, y, width, height), die_label, _), measure in zip(candidates, die_measures):
                if measure.source == "rotated":
                    color = (255, 255, 0)  # cyan: rotated color-block box
                    cv2.polylines(frame, [measure.corners], True, color, 2)
                    rotation_text = "rot: {:+.1f} deg".format(measure.angle)
                else:
                    color = (0, 0, 255)  # red: YOLO box fallback
                    cv2.rectangle(frame, (x, y), (x + width, y + height), color, 2)
                    rotation_text = "rot: n/a"
                cv2.circle(frame, die_center, 5, (0, 0, 255), -1)
                die_text = "{} dx: {} dy: {}  {}  area: {} px".format(
                    die_label, die_center[0] - image_center_x, die_center[1] - image_center_y,
                    rotation_text, measure.area)
                cv2.putText(frame, die_text, (x, max(y - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            overlay = frame.copy()
            cv2.circle(overlay, (image_center_x, image_center_y), args.margin, (0, 255, 0), -1)
            frame = cv2.addWeighted(overlay, 0.5 if aligned else 0.2, frame, 0.5 if aligned else 0.8, 0)
            cv2.circle(frame, (image_center_x, image_center_y), args.margin, (0, 255, 0), 3)
            cv2.circle(frame, (image_center_x, image_center_y), 5, (0, 255, 0), -1)
            cv2.putText(frame, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, "forward: {:.2f}  right: {:.2f} m/s".format(forward_speed, right_speed), (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, error_text, (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if aligned else (0, 0, 255), 2)

            # Camera height from the followed die's apparent size, shown in dark blue.
            height_meters = None
            if detection is not None:
                height_meters = estimate_height_meters(die_measures[candidates.index(detection)].area, image_width)
            height_text = "n/a" if height_meters is None else "{:.2f} m".format(height_meters)
            cv2.putText(frame, "height: " + height_text, (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            if args.mode == "flight":
                writer.write(frame)
            else:
                cv2.imshow("Dice tracker", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    if not args.bench:
                        print("Q pressed; landing")
                        control.land()
                        landed = True
                    break

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("Interrupted; landing")
    finally:
        if airborne and not landed:
            control.land()
        close_resources(camera, writer)


if __name__ == "__main__":
    main()
