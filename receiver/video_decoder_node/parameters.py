from rclpy.node import Node
from rclpy.parameter import Parameter


def required_parameter(node: Node, name: str, param_type: Parameter.Type):
    node.declare_parameter(name, param_type)
    value = node.get_parameter(name).value
    if value is None:
        raise RuntimeError(f"missing required parameter: {name}")
    return value
