import rclpy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from veni_vidi_vici_bot_one.VVV_util import *
from veni_vidi_vici_bot_one.VVV_sm_node import StateMachineNode
from veni_vidi_vici_bot_one.VVV_sm_node_methods import *

def main(args=None):

    rclpy.init(args=args)
    state_machine_node = StateMachineNode()

    executor = MultiThreadedExecutor()
    executor.add_node(state_machine_node)

    try:
        executor.spin()
    finally:
        state_machine_node.destroy_node()
        rclpy.shutdown()