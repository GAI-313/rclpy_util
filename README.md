# rclpy_util

`rclpy_util` is a utility package for enhancing the usability of ROS 2 applications built using `rclpy`. This package provides tools and classes, such as temporary subscribers, to simplify common patterns in ROS 2 development.

---

## Features

- **TemporarySubscriber**: A class that allows the creation of temporary subscriptions using the `with` statement.
- Simplifies resource management by automatically cleaning up resources after usage.

---

## Installation

1. Ensure you have ROS 2 installed on your system.
2. Clone the repository into your ROS 2 workspace:

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/GAI-313/rclpy_util.git
    ```

3. Build the package:

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select rclpy_util
    ```

4. Source your workspace:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

---

## Usage

### Example: TemporarySubscriber

The `TemporarySubscriber` class allows you to create temporary subscribers within a `with` block. The subscription is automatically destroyed when the block is exited.

#### Code Example

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy_util.temporary_subscriber import TemporarySubscriber

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')

    def temporary_subscription_demo(self):
        def callback(msg):
            self.get_logger().info(f'Received: {msg.data}')

        with TemporarySubscriber(
            node=self,
            msg=String,
            topic='example_topic',
            qos_profile=10,
            cb=callback
        ):
            self.get_logger().info('Subscription active. Waiting for messages...')
            rclpy.spin_once(self, timeout_sec=5)

rclpy.init()
node = ExampleNode()
node.temporary_subscription_demo()
rclpy.shutdown()
```

### Steps

1. Import the `TemporarySubscriber` class from `rclpy_util.temporary_subscriber`.
2. Use the class within a `with` statement, passing the necessary arguments (node, message type, topic, QoS profile, and callback).
3. Perform actions while the subscription is active.

---

## API Reference

### TemporarySubscriber

**Arguments:**
- `node` (Node): The ROS 2 node.
- `msg` (type): Message type (e.g., `std_msgs.msg.String`).
- `topic` (str): Topic name.
- `qos_profile` (Union[int, QoSProfile]): Quality of Service settings.
- `cb` (function): Callback function for handling received messages.
- `*args`: Additional arguments for the callback function.

**Methods:**
- `__enter__()`: Initializes the subscription.
- `__exit__()`: Cleans up the subscription.

はい、承知いたしました。
既存のREADMEのスタイルに合わせて、`TemporaryApproximateTimeSynchronizer`に関する説明を記述します。
以下を`README.md`に追記してください。

-----

### Example: TemporaryApproximateTimeSynchronizer

The `TemporaryApproximateTimeSynchronizer` class allows you to create temporary synchronized subscriptions to multiple topics using `message_filters`. All subscriptions are automatically destroyed when the `with` block is exited, simplifying resource management for complex subscription patterns.

#### Code Example

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Header
from rclpy_util.temporary_subscriber import TemporaryApproximateTimeSynchronizer

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_sync_node')

    def temporary_sync_demo(self):
        # Callback for synchronized messages
        def sync_callback(msg1, msg2):
            self.get_logger().info('Received synchronized messages:')
            self.get_logger().info(f'  Topic 1 (String): "{msg1.data}"')
            self.get_logger().info(f'  Topic 2 (Header): Frame ID "{msg2.frame_id}" at stamp {msg2.stamp.sec}')

        # Define the topics to be synchronized
        # The format is a list of (message_type, topic_name) tuples
        sub_topics = [
            (String, 'topic1'),
            (Header, 'topic2')
        ]

        with TemporaryApproximateTimeSynchronizer(
            node=self,
            sub_topics=sub_topics,
            qos_profile=10,
            slop=0.1,  # Allow 0.1 seconds difference
            callback=sync_callback
        ):
            self.get_logger().info('Synchronized subscription active. Waiting for messages...')
            # Spin for a few seconds to receive messages
            # In a real scenario, this block would contain logic that needs the synchronized data
            end_time = self.get_clock().now().nanoseconds / 1e9 + 5.0
            while self.get_clock().now().nanoseconds / 1e9 < end_time:
                rclpy.spin_once(self, timeout_sec=0.1)

rclpy.init()
node = ExampleNode()
node.temporary_sync_demo()
rclpy.shutdown()

```

### Steps

1.  Import the `TemporaryApproximateTimeSynchronizer` class.
2.  Define a list of topics to subscribe to. Each item in the list should be a tuple containing the message type and topic name.
3.  Define a callback function that accepts arguments corresponding to the messages from the subscribed topics, in the same order.
4.  Use the class within a `with` statement, passing the node, topic list, QoS profile, slop (maximum time difference), and the callback function.
5.  Perform actions while the synchronized subscriptions are active.

-----

### TemporaryApproximateTimeSynchronizer

**Arguments:**

  - `node` (Node): The ROS 2 node.
  - `sub_topics` (List[Tuple[Type, str]]): A list of tuples, where each tuple contains the message type and the topic name to subscribe to.
  - `qos_profile` (Union[int, QoSProfile]): Quality of Service settings applied to each subscriber.
  - `slop` (float): The time window in seconds within which messages are considered synchronized.
  - `callback` (function): The callback function to execute with the synchronized messages. The number of arguments must match the number of topics.
  - `*args`: Additional arguments to be passed to the callback function.

**Methods:**

  - `__enter__()`: Initializes the `ApproximateTimeSynchronizer` and all underlying subscriptions.
  - `__exit__()`: Cleans up and destroys all underlying subscriptions.

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---

## Contributing

1. Fork the repository.
2. Create a feature branch (`git checkout -b feature-name`).
3. Commit your changes (`git commit -m 'Add feature'`).
4. Push to the branch (`git push origin feature-name`).
5. Open a pull request.

