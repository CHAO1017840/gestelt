"""Small rospy-shaped API backed by rclpy.

The original scenario scripts create publishers at module import time.  This
layer therefore creates the rclpy node lazily, immediately before first use.
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


_node = None
_publishers = {}


def _ensure_node(prefix="gestelt_py"):
    global _node
    if _node is not None:
        return _node
    if not rclpy.ok():
        rclpy.init()
    _node = rclpy.create_node(f"{prefix}_{os.getpid()}")
    return _node


def init_node(name, anonymous=False, args=None, **kwargs):
    global _node
    if _node is not None:
        return _node
    if not rclpy.ok():
        rclpy.init(args=args)
    if anonymous:
        name = f"{name}_{os.getpid()}"
    _node = rclpy.create_node(name, **kwargs)
    return _node


class Publisher:
    def __init__(self, topic, msg_type, queue_size=10, latch=False, **kwargs):
        self.topic = topic
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.latch = latch
        self._publisher = None
        self._waited_for_subscribers = False

    def _get_publisher(self):
        if self._publisher is None:
            key = (self.topic, self.msg_type, self.queue_size, self.latch)
            if key in _publishers:
                self._publisher = _publishers[key]
                return self._publisher
            qos = QoSProfile(depth=self.queue_size)
            if self.latch:
                qos.transient_local()
            self._publisher = _ensure_node().create_publisher(
                self.msg_type, self.topic, qos
            )
            _publishers[key] = self._publisher
        return self._publisher

    def publish(self, message):
        publisher = self._get_publisher()
        if not self._waited_for_subscribers:
            deadline = time.monotonic() + 1.5
            while rclpy.ok() and time.monotonic() < deadline:
                if publisher.get_subscription_count() > 0:
                    break
                rclpy.spin_once(_ensure_node(), timeout_sec=0.05)
            self._waited_for_subscribers = True
        publisher.publish(message)


class Subscriber:
    def __init__(self, topic, msg_type, callback, queue_size=10, **kwargs):
        self._subscription = _ensure_node().create_subscription(
            msg_type, topic, callback, queue_size
        )


def wait_for_message(topic, msg_type, timeout=None):
    node = _ensure_node()
    received = []

    def callback(message):
        if not received:
            received.append(message)

    subscription = node.create_subscription(msg_type, topic, callback, 1)
    deadline = None if timeout is None else time.monotonic() + timeout
    try:
        while rclpy.ok() and not received:
            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Timed out after {timeout:.1f}s waiting for {topic}"
                )
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_subscription(subscription)
    return received[0]


class Rate:
    def __init__(self, frequency):
        self.period = 1.0 / float(frequency)
        self.next_time = time.monotonic()

    def sleep(self):
        self.next_time += self.period
        delay = self.next_time - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            self.next_time = time.monotonic()


def is_shutdown():
    return not rclpy.ok()


def sleep(duration):
    time.sleep(float(duration))


def spin():
    if _node is not None:
        rclpy.spin(_node)


def shutdown():
    rclpy.shutdown()


def loginfo(message, *args, **kwargs):
    print(message, *args, **kwargs)


def logwarn(message, *args, **kwargs):
    print(message, *args, **kwargs)


def logerr(message, *args, **kwargs):
    print(message, *args, **kwargs)
