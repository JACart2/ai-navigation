import argparse
import sys
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.utilities import remove_ros_args


def _service_prefix(node_name):
    return node_name if node_name.startswith("/") else f"/{node_name}"


def _spin_future(node, future, timeout_s):
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_until_future_complete(node, future, timeout_sec=0.1)
        if future.done():
            return future.result()
    raise TimeoutError(f"timed out after {timeout_s:.1f}s")


def _get_state(node, client, timeout_s):
    response = _spin_future(node, client.call_async(GetState.Request()), timeout_s)
    return response.current_state


def _change_state(node, client, transition_id, transition_label, timeout_s):
    request = ChangeState.Request()
    request.transition.id = transition_id
    request.transition.label = transition_label
    response = _spin_future(node, client.call_async(request), timeout_s)
    return response.success


def _wait_for_state(node, client, wanted_states, timeout_s):
    deadline = time.monotonic() + timeout_s
    last_state = None
    while rclpy.ok() and time.monotonic() < deadline:
        state = _get_state(node, client, timeout_s=1.0)
        last_state = state.label
        if state.label in wanted_states:
            return state
        time.sleep(0.2)
    raise TimeoutError(
        f"waited {timeout_s:.1f}s for {sorted(wanted_states)}, last state was {last_state}"
    )


def _bring_up_lifecycle_node(args):
    target_prefix = _service_prefix(args.target_node)
    get_state_client = args.node.create_client(GetState, f"{target_prefix}/get_state")
    change_state_client = args.node.create_client(
        ChangeState, f"{target_prefix}/change_state"
    )

    if not get_state_client.wait_for_service(timeout_sec=args.startup_timeout):
        raise TimeoutError(f"{target_prefix}/get_state service did not appear")
    if not change_state_client.wait_for_service(timeout_sec=args.startup_timeout):
        raise TimeoutError(f"{target_prefix}/change_state service did not appear")

    state = _get_state(args.node, get_state_client, args.transition_timeout)
    args.node.get_logger().info(f"{args.target_node} lifecycle state: {state.label}")
    if state.label == "active":
        return

    if state.label == "unconfigured":
        args.node.get_logger().info(f"Configuring {args.target_node}")
        if not _change_state(
            args.node,
            change_state_client,
            Transition.TRANSITION_CONFIGURE,
            "configure",
            args.transition_timeout,
        ):
            raise RuntimeError(f"{args.target_node} rejected configure transition")
        state = _wait_for_state(
            args.node,
            get_state_client,
            {"inactive", "active"},
            args.transition_timeout,
        )

    if state.label == "inactive":
        args.node.get_logger().info(f"Activating {args.target_node}")
        if not _change_state(
            args.node,
            change_state_client,
            Transition.TRANSITION_ACTIVATE,
            "activate",
            args.transition_timeout,
        ):
            raise RuntimeError(f"{args.target_node} rejected activate transition")
        _wait_for_state(
            args.node,
            get_state_client,
            {"active"},
            args.transition_timeout,
        )
        return

    if state.label in {"configuring", "activating"}:
        _wait_for_state(
            args.node,
            get_state_client,
            {"active"},
            args.transition_timeout,
        )
        return

    if state.label != "active":
        raise RuntimeError(
            f"{args.target_node} is in unsupported lifecycle state {state.label}"
        )


def main(argv=None):
    argv = list(sys.argv if argv is None else argv)
    rclpy.init(args=argv)
    node = rclpy.create_node("localization_lifecycle_bringup")

    parser = argparse.ArgumentParser()
    parser.add_argument("--target-node", default="/lidar_localization")
    parser.add_argument("--startup-timeout", type=float, default=45.0)
    parser.add_argument("--transition-timeout", type=float, default=45.0)

    parsed = parser.parse_args(remove_ros_args(args=argv)[1:])
    parsed.node = node

    try:
        _bring_up_lifecycle_node(parsed)
        node.get_logger().info(f"{parsed.target_node} lifecycle bring-up complete")
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
