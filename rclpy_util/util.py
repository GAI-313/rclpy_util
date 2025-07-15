from rclpy.qos import QoSProfile
from rclpy.node import Node
import rclpy

import message_filters

from typing import Union, List, Tuple, Type


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
        #pass
        #self.sub.destroy()
        self.node.destroy_subscription(self.sub)


class TemporaryApproximateTimeSynchronizer:
    """
    A class to manage a temporary ROS 2 ApproximateTimeSynchronizer using the `with` statement.

    This allows you to create synchronized subscriptions to multiple topics that exist
    only within the scope of a `with` block. When the block is exited, all subscriptions
    are automatically destroyed.

    Example usage:
        >>> from std_msgs.msg import String, Header
        >>>
        >>> sub_topics = [(String, 'topic1'), (Header, 'topic2')]
        >>> with TemporaryApproximateTimeSynchronizer(node, sub_topics, 10, 0.1, callback) as ts:
        >>>     # Perform operations while the synchronized subscriptions are active.
        >>>     rclpy.spin_once(node, timeout_sec=1.0)
        >>> # Subscriptions are automatically cleaned up here.
    """

    def __init__(
        self,
        node: Node,
        sub_topics: List[Tuple[Type, str]],
        qos_profile: Union[int, QoSProfile],
        slop: float,
        callback,
        *args
    ):
        """
        Initializes the TemporaryApproximateTimeSynchronizer.

        Args:
            node (Node): The ROS 2 node where the subscriptions are created.
            sub_topics (List[Tuple[Type, str]]): A list of tuples, where each tuple contains
                                                 the message type and the topic name to subscribe to.
            queue_size (int): The queue size for the synchronizer.
            slop (float): The time window (in seconds) for message synchronization.
            callback (function): The callback function to execute with synchronized messages.
            *args: Additional arguments to be passed to the callback function.
        """
        self.node = node
        self.sub_topics = sub_topics
        
        # 内部で message_filters.Subscriber のリストを作成
        self.filters = [message_filters.Subscriber(node, msg_type, topic, qos_profile=qos_profile) for msg_type, topic in sub_topics]
        
        # ApproximateTimeSynchronizer を初期化
        self.ts = message_filters.ApproximateTimeSynchronizer(self.filters, 10, slop, allow_headerless=True)
        
        # コールバックを登録
        self.ts.registerCallback(callback, *args)
        

    def __enter__(self) -> message_filters.ApproximateTimeSynchronizer:
        """
        Enters the runtime context and returns the synchronizer instance.
        """
        return self.ts


    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Exits the runtime context, destroying all created subscriptions.
        """
        # 各フィルターが内部に持つ rclpy のサブスクリプションを安全に破棄する
        for sub_filter in self.filters:
            if hasattr(sub_filter, 'sub') and sub_filter.sub is not None:
                self.node.destroy_subscription(sub_filter.sub)
