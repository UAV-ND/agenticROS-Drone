#!/usr/bin/env python3
import argparse
import collections
import os
import sys
import time

import cv2
import jetson.utils

sys.path.insert(1, os.path.join(os.path.dirname(os.path.abspath(__file__)), "modules"))

import control
import keyboard


def parse_args():
    """Parse command-line options for camera, tracking, and flight settings."""
    parser = argparse.ArgumentParser(description="Visually center and track a white die")
    parser.add_argument("--mode", choices=("test", "flight"), default="test")
    parser.add_argument("--debug_path", default="debug/dice_tracker")
    parser.add_argument("--control", choices=("PID", "P"), default="PID")
    parser.add_argument("--altitude", type=float, default=1.5)
    parser.add_argument("--min-area", type=float, default=300.0)
    parser.add_argument("--min-value", type=int, default=150)
    parser.add_argument("--max-saturation", type=int, default=100)
    parser.add_argument("--deadband", type=float, default=12.0)
    return parser.parse_args()


def to_bgr(image):
    """Convert a camera image to BGR format, copying it when already compatible."""
    if image.ndim == 3 and image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    return image.copy()


def find_die(image, min_area, min_value, max_saturation):
    """Find the largest bright, low-saturation, roughly square candidate in a BGR frame.

    Return ``(area, center, bounds)`` for the best candidate, or ``None`` if none
    pass the size and shape filters. ``center`` is an ``(x, y)`` pixel coordinate;
    ``bounds`` is ``(x, y, width, height)``.

    Modify this function to a different script that we have that detects the die of choice
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        (0, 0, min_value),
        (179, max_saturation, 255),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    frame_area = image.shape[0] * image.shape[1]
    candidates = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area or area > frame_area * 0.3:
            continue

        x, y, width, height = cv2.boundingRect(contour)
        aspect_ratio = width / float(height)
        rectangularity = area / float(width * height)
        if not 0.55 <= aspect_ratio <= 1.8 or rectangularity < 0.55:
            continue

        perimeter = cv2.arcLength(contour, True)
        polygon = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        if not 4 <= len(polygon) <= 10:
            continue

        center = (x + width // 2, y + height // 2)
        candidates.append((area, center, (x, y, width, height)))

    if not candidates:
        return None
    return max(candidates, key=lambda candidate: candidate[0])


def close_resources(camera, writer):
    """Optional video resources, camera, display windows, and debug logs."""
    if writer is not None:
        writer.release()
    if camera is not None:
        camera.Close()
    cv2.destroyAllWindows()
    for log_file in (control.debug_yaw, control.debug_velocity):
        if log_file is not None:
            log_file.close()


def main():
    """Initialize the camera and drone, track the die, then land and clean up."""
    args = parse_args()
    camera = None
    writer = None
    airborne = False
    landed = False

    try:
        camera = jetson.utils.videoSource("csi://0")
        image_width, image_height = camera.GetWidth(), camera.GetHeight()
        image_center_x = image_width // 2
        x_errors = collections.deque(maxlen=5)

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

        control.connect_drone(connection)
        control.configure_PID(args.control)
        control.set_flight_altitude(args.altitude)
        control.initialize_debug_logs(args.debug_path)

        print("Dice tracking: white, square-like targets; yaw-only correction")
        control.print_drone_report()
        control.arm_and_takeoff(args.altitude)
        airborne = True

        while True:
            if keyboard.is_pressed("q"):
                print("Q pressed; landing")
                control.land()
                landed = True
                break

            frame = to_bgr(jetson.utils.cudaToNumpy(camera.Capture()))
            detection = find_die(frame, args.min_area, args.min_value, args.max_saturation)
            yaw_command = 0.0

            if detection is None:
                x_errors.clear()
                control.stop_drone()
                status = "No die detected; holding position"
            else:
                area, center, bounds = detection
                x_error = center[0] - image_center_x
                x_errors.append(x_error)
                average_error = sum(x_errors) / len(x_errors)

                if abs(average_error) <= args.deadband:
                    control.setXdelta(0)
                else:
                    control.setXdelta(average_error)
                control.setZDelta(0)
                control.control_drone()
                yaw_command = control.getMovementYawAngle()
                x, y, width, height = bounds
                cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 0, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                status = "Die area: {} px".format(int(area))

            cv2.circle(frame, (image_center_x, image_height // 2), 5, (0, 255, 0), -1)
            cv2.putText(frame, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, "yaw: {:.1f}".format(yaw_command), (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            if args.mode == "flight":
                writer.write(frame)
            else:
                cv2.imshow("Dice tracker", frame)
                cv2.waitKey(1)

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("Interrupted; landing")
    finally:
        if airborne and not landed:
            control.land()
        close_resources(camera, writer)


if __name__ == "__main__":
    main()