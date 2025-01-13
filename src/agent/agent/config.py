class Config:
    def __init__(self, node: Node):
        node.declare_parameters(
            parameters=[
                ('drone_id', -1),
                ('group_id', -1)
            ])

    def get_int_parameter(self, key: str):
        return node.get_parameter(key).get_parameter_value().integer_value
