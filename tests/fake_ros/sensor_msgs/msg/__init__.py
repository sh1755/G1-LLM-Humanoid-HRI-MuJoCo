class _Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""


class JointState:
    def __init__(self):
        self.header = _Header()
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []
