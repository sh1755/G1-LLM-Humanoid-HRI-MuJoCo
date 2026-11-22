import threading
import rclpy


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def __init__(self, name):
        self.name = name

    def _log(self, level, text):
        rclpy.LOG.append((self.name, level, text))

    def info(self, t): self._log("INFO", t)
    def warn(self, t): self._log("WARN", t)
    def warning(self, t): self._log("WARN", t)
    def error(self, t): self._log("ERROR", t)


class _Publisher:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        rclpy.publish(self.topic, msg)


class _Stamp:
    sec = 0
    nanosec = 0


class _Now:
    def to_msg(self):
        return _Stamp()


class _Clock:
    def now(self):
        return _Now()


class _Timer:
    def __init__(self, period, cb):
        self._stop = False
        def run():
            while not self._stop and rclpy.ok():
                threading.Event().wait(period)
                if not self._stop and rclpy.ok():
                    cb()
        threading.Thread(target=run, daemon=True).start()

    def cancel(self):
        self._stop = True


class Node:
    OVERRIDES = {}   # node_name -> {param: value}; set by tests

    def __init__(self, name):
        self._name = name
        self._params = {}
        self._timers = []
        self._logger = _Logger(name)

    def get_name(self):
        return self._name

    def declare_parameter(self, name, default=None):
        value = Node.OVERRIDES.get(self._name, {}).get(name, default)
        self._params[name] = _Param(value)
        return self._params[name]

    def get_parameter(self, name):
        return self._params[name]

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock()

    def create_publisher(self, msg_type, topic, qos):
        return _Publisher(topic)

    def create_subscription(self, msg_type, topic, cb, qos):
        with rclpy._BUS_LOCK:
            rclpy._BUS.setdefault(topic, []).append(cb)
        return cb

    def create_timer(self, period, cb):
        t = _Timer(period, cb)
        self._timers.append(t)
        return t

    def destroy_node(self):
        for t in self._timers:
            t.cancel()
