from rclpy.qos import QoSProfile
from rclpy.node import Node
import rclpy

from typing import Union

class TemporarySubscriber():
    """
    A class to manage a temporary ROS 2 subscription using the `with` statement.

    This class allows you to create a ROS 2 subscriber that exists only within the scope 
    of a `with` block. Once the block is exited, the subscription is automatically destroyed.

    Example usage:
        >>> with TemporarySubscriber(node, msg_type, topic, qos_profile, callback) as sub:
        >>>     # Perform operations while the subscription is active
        >>> # Subscription is automatically cleaned up when the block is exited.

    Args:
        node (Node): The ROS 2 node where the subscription is created.
        msg (type): The type of the message to subscribe to.
        topic (str): The name of the topic to subscribe to.
        qos_profile (Union[int, QoSProfile]): The Quality of Service (QoS) settings for the subscription.
        cb (function): The callback function to be executed when a message is received.
        *args: Additional arguments to be passed to the callback function.
    """

    def __init__(
        self,
        node: Node,
        msg,
        topic: str,
        qos_profile: Union[int, QoSProfile],
        cb,
        *args
    ) -> None:
        """
        Initialize the TemporarySubscriber instance.

        Args:
            node (Node): The ROS 2 node where the subscription is created.
            msg (type): The type of the message to subscribe to.
            topic (str): The name of the topic to subscribe to.
            qos_profile (Union[int, QoSProfile]): The Quality of Service (QoS) settings for the subscription.
            cb (function): The callback function to be executed when a message is received.
            *args: Additional arguments to be passed to the callback function.
        """
        self.node = node
        self.msg = msg
        self.topic = topic
        self.qos_profile = qos_profile
        self.cb = cb
        self.args = args

    def __enter__(self) -> Node.create_subscription:
        """
        Enter the context of the TemporarySubscriber.

        This method creates the subscription when entering the `with` block.

        Returns:
            Node.create_subscription: The created subscription object.
        """
        self.sub = self.node.create_subscription(
            msg_type=self.msg,
            topic=self.topic,
            callback=self.cb,
            qos_profile=self.qos_profile,
        )
        return self.sub

    def __exit__(self, exctype, excval, traceback):
        """
        Exit the context of the TemporarySubscriber.

        This method destroys the subscription when exiting the `with` block.

        Args:
            exctype (type): The exception type, if any exception was raised.
            excval (BaseException): The exception value, if any exception was raised.
            traceback (TracebackType): The traceback object, if any exception was raised.
        """
        pass
        #self.sub.destroy()
