"""Minimal fake rclpy for offline tests (see ../README.md). Not ROS 2."""
import threading, time

_ok = False
_BUS = {}          # topic -> list of callbacks
_BUS_LOCK = threading.RLock()
LOG = []           # (node, level, text)


def init(args=None):
    global _ok
    _ok = True


def ok():
    return _ok


def shutdown():
    global _ok
    _ok = False
    with _BUS_LOCK:
        _BUS.clear()


def spin(node):
    while _ok:
        time.sleep(0.05)


def spin_once(node, timeout_sec=0.1):
    time.sleep(timeout_sec)


def publish(topic, msg):
    with _BUS_LOCK:
        cbs = list(_BUS.get(topic, []))
    for cb in cbs:
        cb(msg)
