# MIT License

# Copyright (c) 2023 Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Bring-up check for the ten LX-16A servos on the bus.

This talks to the servos directly over the BusLinker; it is NOT a ROS node, so
make sure the motor controller node is stopped first or the two will fight over
the serial port.

    ros2 run rover_motor_controller check_motors
    ros2 run rover_motor_controller check_motors --blink
    ros2 run rover_motor_controller check_motors --scan

--blink walks the configured IDs one at a time and flashes each servo's LED, so
you can confirm that the ID assignment in lx16a_consts.py really matches where
the servo sits on the rover.
"""

import argparse
import time

from serial import SerialException

from rover_motor_controller.lx16a.lx16a import LX16A
from rover_motor_controller.lx16a.lx16a_consts import (
    SERVO_LEFT_FRONT,
    SERVO_RIGHT_FRONT,
    SERVO_LEFT_BACK,
    SERVO_RIGHT_BACK,
    MOTOR_LEFT_FRONT,
    MOTOR_LEFT_MIDDLE,
    MOTOR_LEFT_BACK,
    MOTOR_RIGHT_FRONT,
    MOTOR_RIGHT_MIDDLE,
    MOTOR_RIGHT_BACK,
)

# (role, id, kind) in the order the rover uses them
MOTORS = [
    ("corner  front left ", SERVO_LEFT_FRONT, "servo"),
    ("corner  front right", SERVO_RIGHT_FRONT, "servo"),
    ("corner  back  left ", SERVO_LEFT_BACK, "servo"),
    ("corner  back  right", SERVO_RIGHT_BACK, "servo"),
    ("drive   front left ", MOTOR_LEFT_FRONT, "motor"),
    ("drive   mid   left ", MOTOR_LEFT_MIDDLE, "motor"),
    ("drive   back  left ", MOTOR_LEFT_BACK, "motor"),
    ("drive   front right", MOTOR_RIGHT_FRONT, "motor"),
    ("drive   mid   right", MOTOR_RIGHT_MIDDLE, "motor"),
    ("drive   back  right", MOTOR_RIGHT_BACK, "motor"),
]


def probe(lx16a: LX16A, servo_id: int) -> dict:
    """Read the identity and health of one servo. Returns {} if it is silent."""

    try:
        echoed_id = lx16a.get_servo_id(servo_id, timeout=0.2)
    except Exception:
        return {}

    if echoed_id is None:
        return {}

    result = {"id": echoed_id}
    for key, getter in (
        ("voltage", lx16a.get_voltage),
        ("temperature", lx16a.get_temperature),
        ("position", lx16a.get_position),
    ):
        try:
            result[key] = getter(servo_id, timeout=0.2)
        except Exception:
            result[key] = None
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-d", "--device", default="/dev/lx16a", help="serial device")
    parser.add_argument("-b", "--baud-rate", type=int, default=115200)
    parser.add_argument(
        "--blink",
        action="store_true",
        help="flash each servo's LED in turn to identify it physically",
    )
    parser.add_argument(
        "--scan",
        action="store_true",
        help="probe every ID from 1 to 253 to find servos not in lx16a_consts.py",
    )
    args = parser.parse_args()

    try:
        lx16a = LX16A(args.device, args.baud_rate)
    except SerialException as e:
        print(f"Could not open {args.device}: {e}")
        print("Is the BusLinker plugged in and are the udev rules installed?")
        print("  ros2 run rover_bringup list_usb_serial.sh")
        return 1

    print(f"Bus: {args.device} @ {args.baud_rate} baud\n")
    print(f"{'role':<20}{'id':>4}{'reply':>7}{'volt':>8}{'temp':>7}{'pos':>7}")
    print("-" * 53)

    missing = []
    for role, servo_id, _kind in MOTORS:
        info = probe(lx16a, servo_id)
        if not info:
            print(f"{role:<20}{servo_id:>4}{'---':>7}{'':>8}{'':>7}{'':>7}   NO RESPONSE")
            missing.append((role, servo_id))
            continue

        volt = f"{info['voltage'] / 1000:.1f}V" if info.get("voltage") else "?"
        temp = f"{info['temperature']}C" if info.get("temperature") is not None else "?"
        pos = info.get("position")
        pos = str(pos) if pos is not None else "?"
        flag = "" if info["id"] == servo_id else f"   ID MISMATCH (replied {info['id']})"
        print(f"{role:<20}{servo_id:>4}{'ok':>7}{volt:>8}{temp:>7}{pos:>7}{flag}")

    print("-" * 53)
    found = len(MOTORS) - len(missing)
    print(f"{found}/{len(MOTORS)} motors responding")
    if missing:
        print("\nSilent IDs:")
        for role, servo_id in missing:
            print(f"  id {servo_id:<4} ({role.strip()})")
        print(
            "\nCheck: servo power rail on, daisy chain seated, and that the ID in\n"
            "rover_motor_controller*/lx16a_consts.* matches the ID burned into the\n"
            "servo. Run with --scan to see which IDs are actually on the bus."
        )

    if args.scan:
        print("\nScanning IDs 1-253 ...")
        present = []
        for servo_id in range(1, 254):
            if probe(lx16a, servo_id):
                present.append(servo_id)
                print(f"  found id {servo_id}")
        print(f"IDs on the bus: {present}")

    if args.blink:
        print("\nBlinking each servo in turn (Ctrl-C to stop) ...")
        for role, servo_id, _kind in MOTORS:
            print(f"  -> {role.strip()} (id {servo_id})")
            for _ in range(3):
                lx16a.led_on(servo_id)
                time.sleep(0.2)
                lx16a.led_off(servo_id)
                time.sleep(0.2)
            time.sleep(0.5)

    return 0 if not missing else 1


if __name__ == "__main__":
    raise SystemExit(main())
