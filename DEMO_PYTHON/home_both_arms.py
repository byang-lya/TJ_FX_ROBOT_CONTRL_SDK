"""Move SDK channels A and B to joint zero. Preview unless --execute is given."""

import argparse
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from SDK_PYTHON.fx_robot import DCSS, Marvin_Robot

ZERO = [0.0] * 7
TOLERANCE = 0.1
LIMIT_PERCENT = 5


def wait_feedback(robot, dcss, index, predicate, timeout, description):
    last_serial = robot.subscribe(dcss)["outputs"][index]["frame_serial"]
    matches = 0
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = robot.subscribe(dcss)
        serial = data["outputs"][index]["frame_serial"]
        if serial and serial != last_serial:
            last_serial = serial
            matches = matches + 1 if predicate(data) else 0
            if matches >= 2:
                return data
        time.sleep(0.02)
    raise RuntimeError(f"Timed out waiting for {description}")


def send(robot, setter, *args):
    robot.clear_set()
    setter(*args)
    if not robot.send_cmd():
        raise RuntimeError(f"Failed to send {setter.__name__}")


def read_parameter(robot, kind, key):
    result = robot.get_param(kind, key)
    if result is None or result[0] != 0:
        raise RuntimeError(f"Could not read {key}")
    return result[1]


def check_faults(robot, dcss, arm, index, execute):
    def idle(data):
        return data["states"][index]["cur_state"] in (0, 100)

    data = wait_feedback(robot, dcss, index, idle, 3, f"{arm} idle or faulted")
    state = data["states"][index]
    servo_errors = [
        read_parameter(robot, "int", f"SERVO{index}ERR{joint}")
        for joint in range(7)
    ]
    if state["err_code"] or state["cur_state"] == 100 or any(servo_errors):
        print(f"{arm} faults: arm={state['err_code']}, servos={servo_errors}")
        if not execute:
            raise RuntimeError(f"{arm} is faulted")
        send(robot, robot.clear_error, arm)
        wait_feedback(
            robot, dcss, index,
            lambda d: d["states"][index]["cur_state"] == 0
            and d["states"][index]["err_code"] == 0,
            3, f"{arm} fault reset",
        )
        for joint in range(7):
            if read_parameter(robot, "int", f"SERVO{index}ERR{joint}"):
                raise RuntimeError(f"{arm} joint {joint + 1} still has a servo fault")


def check_pose(robot, dcss, arm, index):
    limits = [
        tuple(
            read_parameter(robot, "float", f"R.A{index}.L{joint}.BASIC.{key}")
            for key in ("LimitNeg", "LimitPos")
        )
        for joint in range(7)
    ]

    def idle(data):
        state = data["states"][index]
        return state["cur_state"] == 0 and state["err_code"] == 0

    data = wait_feedback(robot, dcss, index, idle, 3, f"{arm} disabled")
    start = data["outputs"][index]["fb_joint_pos"]
    for joint, (value, target, (low, high)) in enumerate(zip(start, ZERO, limits), 1):
        if not (math.isfinite(low) and math.isfinite(high)
                and low <= value <= high and low <= target <= high):
            raise RuntimeError(
                f"{arm} J{joint}: start {value}, target {target}, limits [{low}, {high}]"
            )
    print(f"{arm}: {start} -> {ZERO} degrees")
    return start


def home(robot, dcss, arm, index, start):
    def position_mode(data):
        state = data["states"][index]
        if state["err_code"] or state["cur_state"] == 100:
            raise RuntimeError(f"{arm} faulted: {state}")
        return state["cur_state"] == 1

    send(robot, robot.set_vel_acc, arm, LIMIT_PERCENT, LIMIT_PERCENT)
    wait_feedback(
        robot, dcss, index,
        lambda d: d["inputs"][index]["joint_vel_ratio"] == LIMIT_PERCENT
        and d["inputs"][index]["joint_acc_ratio"] == LIMIT_PERCENT,
        3, f"{arm} speed and acceleration limits",
    )

    planning_attempted = False
    arrived = False
    try:
        send(robot, robot.set_state, arm, 1)
        time.sleep(1)
        data = wait_feedback(robot, dcss, index, position_mode, 3, f"{arm} position mode")
        actual = data["outputs"][index]["fb_joint_pos"]
        if any(not abs(a - b) <= TOLERANCE for a, b in zip(actual, start)):
            raise RuntimeError(f"{arm} moved on enable. Aborting before planning.")

        planning_attempted = True
        ratio = LIMIT_PERCENT / 100
        if not robot.setPln_joint(arm, actual, ZERO, ratio, ratio):
            raise RuntimeError(f"{arm} joint planner rejected the target")

        def at_zero(data):
            if not position_mode(data):
                raise RuntimeError(f"{arm} left position mode")
            output = data["outputs"][index]
            return (
                output["traj_state"] == b"\x00"
                and output["low_speed_flag"] == b"\x01"
                and all(abs(q) <= TOLERANCE for q in output["fb_joint_pos"])
            )

        wait_feedback(robot, dcss, index, at_zero, 120, f"{arm} at zero")
        arrived = True
        print(f"{arm} reached zero within {TOLERANCE} degrees")
    finally:
        try:
            if planning_attempted and not arrived and not robot.stopRunPln_joint(arm):
                print(f"{arm}: planner stop request failed", file=sys.stderr)
        finally:
            send(robot, robot.set_state, arm, 0)
            wait_feedback(
                robot, dcss, index,
                lambda d: d["states"][index]["cur_state"] == 0,
                3, f"{arm} servo shutdown",
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", required=True, help="Controller IPv4 address")
    parser.add_argument("--config", type=Path, help="Model-matched .MvKDCfg for joint planning")
    parser.add_argument(
        "--execute", action="store_true",
        help="Authorize motion after verifying clearance for both full paths",
    )
    args = parser.parse_args()
    if args.execute and (args.config is None or not args.config.is_file()):
        parser.error("--execute requires --config pointing to the robot's model-matched .MvKDCfg")
    robot = Marvin_Robot()
    if not robot.connect(args.ip):
        raise RuntimeError("Controller connection failed")
    try:
        dcss = DCSS()
        for index, arm in enumerate("AB"):
            check_faults(robot, dcss, arm, index, args.execute)
            check_pose(robot, dcss, arm, index)
        if args.execute:
            if not robot.pln_init(str(args.config.resolve())):
                raise RuntimeError("Joint planner initialization failed")
            for index, arm in enumerate("AB"):
                check_faults(robot, dcss, arm, index, True)
                start = check_pose(robot, dcss, arm, index)
                home(robot, dcss, arm, index, start)
        else:
            print("Preview complete. No motion or state commands sent.")
    finally:
        if not robot.release_robot():
            raise RuntimeError("Controller release failed")


if __name__ == "__main__":
    main()
