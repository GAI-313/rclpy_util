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

