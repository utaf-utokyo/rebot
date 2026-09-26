"""Diagnostic helper shared by bridge_node and moveit_ik_bridge.

Running either node twice by accident (a known M5 rehearsal pitfall -- see
docs/architecture.md "既知のはまりポイント") makes two processes compete for
the same service/topic name with no clear error. This turns the manual
`ps aux | grep ...` check into something that shows up in the node's own log
(and therefore in ~/.ros/log/) automatically.
"""

from __future__ import annotations

from rclpy.node import Node


def warn_if_duplicate_node(node: Node, name: str) -> None:
    count = node.get_node_names().count(name)
    if count > 1:
        node.get_logger().error(
            f"duplicate node detected: '{name}' appears {count} times on the ROS "
            "graph -- another instance of this process is likely still running "
            "(check `ps aux | grep " + name + "`) and competing for the same "
            "service/topic names"
        )
