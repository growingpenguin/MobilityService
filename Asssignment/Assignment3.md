# MobilityService
## 2 Variables
### Supporting Files
**ArithmeticArgument2.msg** <br/>
```
# Messages
builtin_interfaces/Time stamp
float32 argument_a
float32 argument_b

```

**ArithmeticOperator2.srv** <br/>
```
# Constants
int8 PLUS = 1
int8 MINUS = 2
int8 MULTIPLY = 3
int8 DIVIDE = 4

# Request
int8 arithmetic_operator1  # First operator

---

# Response
float32 final_result

```

**ArithmeticChecker2.action** <br/>
```
# Goal
float32 goal_sum
---
# Result
string[] all_formula
float32 total_sum
---
# Feedback
string[] formula

```

**CMakelists.txt** <br/>
```

################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(msg_srv_action_interface_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################
set(msg_files
  "msg/ArithmeticArgument.msg"
  "msg/ArithmeticArgument2.msg"
)

set(srv_files
  "srv/ArithmeticOperator.srv"
  "srv/ArithmeticOperator2.srv"
)

set(action_files
  "action/ArithmeticChecker.action"
  "action/ArithmeticChecker2.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rosidl_default_runtime)
ament_package()

```

### Publisher Subscriber
**calpubpub.py** <br/>

```
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from msg_srv_action_interface_example.msg import ArithmeticArgument2


class CalculatorPublisher(Node):

    def __init__(self):
        super().__init__('calculator_publisher')

        # Declare parameters
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value
        self.declare_parameter('min_random_num', 0)
        self.min_random_num = self.get_parameter('min_random_num').value
        self.declare_parameter('max_random_num', 9)
        self.max_random_num = self.get_parameter('max_random_num').value

        # Define QoS settings
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        # Publisher setup
        self.arithmetic_argument_publisher = self.create_publisher(
            ArithmeticArgument2,
            'arithmetic_argument',
            QOS_RKL10V)

        # Timer setup to publish messages periodically
        self.timer = self.create_timer(1.0, self.publish_random_arithmetic_arguments)

    def publish_random_arithmetic_arguments(self):
        # Create and publish random arguments
        msg = ArithmeticArgument2()
        msg.stamp = self.get_clock().now().to_msg()
        msg.argument_a = float(random.randint(self.min_random_num, self.max_random_num))
        msg.argument_b = float(random.randint(self.min_random_num, self.max_random_num))
        self.arithmetic_argument_publisher.publish(msg)
        self.get_logger().info('Published: a={0}, b={1}'.format(
            msg.argument_a, msg.argument_b))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CalculatorPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**calsubsub.py** <br/>
```
from msg_srv_action_interface_example.msg import ArithmeticArgument2
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class CalculatorSubscriber(Node):

    def __init__(self):
        super().__init__('calculator_subscriber')
        self.argument_a = 0.0
        self.argument_b = 0.0

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        # Subscription
        self.arithmetic_argument_subscriber = self.create_subscription(
            ArithmeticArgument2,
            'arithmetic_argument',
            self.get_arithmetic_argument,
            QOS_RKL10V)

    def get_arithmetic_argument(self, msg):
        self.argument_a = msg.argument_a
        self.argument_b = msg.argument_b
        self.get_logger().info('Received arguments: a={0}, b={1}'.format(
            self.argument_a, self.argument_b))


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = CalculatorSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node terminated.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```
### Service

**calssss.py** <br/>
```

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from msg_srv_action_interface_example.srv import ArithmeticOperator2
from msg_srv_action_interface_example.msg import ArithmeticArgument2


class Calssss(Node):
    """
    Service Server Node for Arithmetic Operations.
    Handles subscription to arithmetic arguments and processes requests with two operators.
    """

    def __init__(self):
        super().__init__('calss')
        self.argument_a = 2.0
        self.argument_b = 4.0
        self.operators = ['+', '-', '*', '/']  # Supported operators
        self.callback_group = ReentrantCallbackGroup()

        # QoS Profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Subscriber for receiving arithmetic arguments
        self.create_subscription(
            ArithmeticArgument2,
            'arithmetic_argument',
            self.get_arithmetic_argument,
            qos_profile,
            callback_group=self.callback_group,
        )

        # Service Server for handling arithmetic operator requests
        self.create_service(
            ArithmeticOperator2,
            'arithmetic_operator',
            self.handle_arithmetic_operator,
            callback_group=self.callback_group,
        )
        self.get_logger().info('Calss service server is ready.')

    def get_arithmetic_argument(self, msg):
        """Callback for receiving arithmetic arguments."""
        self.argument_a = msg.argument_a
        self.argument_b = msg.argument_b
        self.get_logger().info(f'Received: a={self.argument_a}, b={self.argument_b}')

    def handle_arithmetic_operator(self, request, response):
        """Service handler for processing arithmetic operations."""
        operator1 = request.arithmetic_operator1

        # First operation: a and b
        final_result = self.perform_operation(self.argument_a, self.argument_b, operator1)

        response.final_result = final_result
        self.get_logger().info(f'Processed result: {final_result}')
        return response

    def perform_operation(self, x, y, operator):
        """Performs arithmetic operation based on the operator."""
        try:
            if operator == 1:  # Addition
                return x + y
            elif operator == 2:  # Subtraction
                return x - y
            elif operator == 3:  # Multiplication
                return x * y
            elif operator == 4:  # Division
                if y == 0:
                    self.get_logger().warn('Division by zero encountered. Returning 0.0 as result.')
                    return 0.0
                return x / y
            else:
                self.get_logger().error(f'Invalid operator: {operator}. Returning 0.0.')
                return 0.0
        except Exception as e:
            self.get_logger().error(f'Error during calculation: {str(e)}')
        return 0.0



def main(args=None):
    rclpy.init(args=args)
    node = Calssss()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Calss node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```

**calscsc.py** <br/>
```
import rclpy
from rclpy.node import Node
from msg_srv_action_interface_example.srv import ArithmeticOperator2
import random


class Calscsc(Node):
    """
    Service Client Node for Arithmetic Operations.
    Sends requests with two random operators to the service server.
    """

    def __init__(self):
        super().__init__('calsc')
        self.client = self.create_client(ArithmeticOperator2, 'arithmetic_operator')

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the service server...')

        self.get_logger().info('Calsc client is ready.')

    def send_request(self):
        """Sends a service request with two random operators."""
        request = ArithmeticOperator2.Request()
        request.arithmetic_operator1 = random.randint(1, 4)  # Random operator 1

        self.get_logger().info(f'Sending operators: {request.arithmetic_operator1}')
        return self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    client_node = Calscsc()
    future = client_node.send_request()

    try:
        while rclpy.ok():
            rclpy.spin_once(client_node)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    client_node.get_logger().error(f'Error during service call: {str(e)}')
                else:
                    client_node.get_logger().info(f'Result received: {response.final_result}')
                break
    except KeyboardInterrupt:
        client_node.get_logger().info('Keyboard Interrupt detected.')
    finally:
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```
### Action
**calasas.py** <br/>
```

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from msg_srv_action_interface_example.action import ArithmeticChecker2
from msg_srv_action_interface_example.msg import ArithmeticArgument2
from msg_srv_action_interface_example.srv import ArithmeticOperator2


class Calasas(Node):
    """
    Action Server Node for ArithmeticChecker.
    Receives goals from the client and processes them iteratively.
    """

    def __init__(self):
        super().__init__('calas')
        self.argument_a = 10.0
        self.argument_b = 5.0

        self.action_server = ActionServer(
            self, ArithmeticChecker2, 'arithmetic_checker', self.execute_callback
        )

        self.get_logger().info('Calas action server is ready.')

    def execute_callback(self, goal_handle):
        """Handles the received goal and provides feedback during execution."""
        self.get_logger().info(f'Received goal: {goal_handle.request.goal_sum}')
        goal_sum = goal_handle.request.goal_sum

        total_sum = 0.0
        feedback_msg = ArithmeticChecker2.Feedback()
        feedback_msg.formula = []

        while total_sum < goal_sum:
            # Perform the operation: a + b
            final_result = self.argument_a + self.argument_b
            total_sum += final_result

            # Feedback to the client
            formula_str = f'({self.argument_a} + {self.argument_b}) = {final_result}'
            feedback_msg.formula.append(formula_str)
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Feedback: {formula_str}, Running Total: {total_sum}')
            time.sleep(1)  # Simulate processing time

        goal_handle.succeed()

        # Final result
        result = ArithmeticChecker2.Result()
        result.all_formula = feedback_msg.formula
        result.total_sum = total_sum
        return result


def main(args=None):
    rclpy.init(args=args)
    node = Calasas()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down calas node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```

**calacac.py** <br/>
```

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from msg_srv_action_interface_example.action import ArithmeticChecker2


class Calacac(Node):
    """
    Action Client Node for ArithmeticChecker.
    Sends a goal to the action server and handles feedback and results.
    """

    def __init__(self):
        super().__init__('calac')
        self.action_client = ActionClient(self, ArithmeticChecker2, 'arithmetic_checker')

    def send_goal(self, goal_sum):
        """Sends a goal to the action server."""
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available.')
            return

        goal_msg = ArithmeticChecker2.Goal()
        goal_msg.goal_sum = goal_sum
        self.get_logger().info(f'Sending goal: {goal_sum}')

        # Send the goal asynchronously
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handles the server's response to the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return
        self.get_logger().info('Goal accepted by server.')

        # Get the result asynchronously
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handles feedback received from the server."""
        feedback = feedback_msg.feedback.formula
        self.get_logger().info(f'Received feedback: {feedback}')

    def result_callback(self, future):
        """Handles the result received from the server."""
        result = future.result().result
        self.get_logger().info(f'Final Result - Formulas: {result.all_formula}, Total Sum: {result.total_sum}')


def main(args=None):
    rclpy.init(args=args)
    node = Calacac()

    try:
        node.send_goal(120.0)  # Example: Sending a goal to calculate a sum of 100
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down calac node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```

## 3 Variables
### Supporting Files
**ArithmeticArgument.msg** <br/>
```
# Messages
builtin_interfaces/Time stamp
float32 argument_a
float32 argument_b
float32 argument_c
```

**ArithmeticOperator.srv** <br/>
```
# Constants
int8 PLUS = 1
int8 MINUS = 2
int8 MULTIPLY = 3
int8 DIVISION = 4

# Request
int8 arithmetic_operator1
int8 arithmetic_operator2
---
# Response
float32 arithmetic_result
```


**ArithmeticChecker.action** <br/>
```
# Goal
float32 goal_sum
---
# Result
string[] all_formula
float32 total_sum
---
# Feedback
string[] formula

```


### Publisher Subscriber 

**calpub.py** <br/>
```
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from msg_srv_action_interface_example.msg import ArithmeticArgument


class CalculatorPublisher(Node):

    def __init__(self):
        super().__init__('calculator_publisher')

        # Declare parameters
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value
        self.declare_parameter('min_random_num', 0)
        self.min_random_num = self.get_parameter('min_random_num').value
        self.declare_parameter('max_random_num', 9)
        self.max_random_num = self.get_parameter('max_random_num').value

        # Define QoS settings
        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        # Publisher setup
        self.arithmetic_argument_publisher = self.create_publisher(
            ArithmeticArgument,
            'arithmetic_argument',
            QOS_RKL10V)

        # Timer setup to publish messages periodically
        self.timer = self.create_timer(1.0, self.publish_random_arithmetic_arguments)

    def publish_random_arithmetic_arguments(self):
        # Create and publish random arguments
        msg = ArithmeticArgument()
        msg.stamp = self.get_clock().now().to_msg()
        msg.argument_a = float(random.randint(self.min_random_num, self.max_random_num))
        msg.argument_b = float(random.randint(self.min_random_num, self.max_random_num))
        msg.argument_c = float(random.randint(self.min_random_num, self.max_random_num))
        self.arithmetic_argument_publisher.publish(msg)
        self.get_logger().info('Published: a={0}, b={1}, c={2}'.format(
            msg.argument_a, msg.argument_b, msg.argument_c))


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CalculatorPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```

**calsub.py** <br/>
```
from msg_srv_action_interface_example.msg import ArithmeticArgument
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class CalculatorSubscriber(Node):

    def __init__(self):
        super().__init__('calculator_subscriber')
        self.argument_a = 0.0
        self.argument_b = 0.0
        self.argument_c = 0.0  # Adding argument_c

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        # Subscription
        self.arithmetic_argument_subscriber = self.create_subscription(
            ArithmeticArgument,
            'arithmetic_argument',
            self.get_arithmetic_argument,
            QOS_RKL10V)

    def get_arithmetic_argument(self, msg):
        self.argument_a = msg.argument_a
        self.argument_b = msg.argument_b
        self.argument_c = msg.argument_c  # Incorporating argument_c
        self.get_logger().info('Received arguments: a={0}, b={1}, c={2}'.format(
            self.argument_a, self.argument_b, self.argument_c))


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = CalculatorSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node terminated.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```

### Service Server Client
**calss.py** <br/>
```
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from msg_srv_action_interface_example.srv import ArithmeticOperator
from msg_srv_action_interface_example.msg import ArithmeticArgument


class Calss(Node):
    """
    Service Server Node for Arithmetic Operations.
    Handles subscription to arithmetic arguments and processes requests with two operators.
    """

    def __init__(self):
        super().__init__('calss')
        self.argument_a = 2.0
        self.argument_b = 4.0
        self.argument_c = 6.0
        self.operators = ['+', '-', '*', '/']  # Supported operators
        self.callback_group = ReentrantCallbackGroup()

        # QoS Profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Subscriber for receiving arithmetic arguments
        self.create_subscription(
            ArithmeticArgument,
            'arithmetic_argument',
            self.get_arithmetic_argument,
            qos_profile,
            callback_group=self.callback_group,
        )

        # Service Server for handling arithmetic operator requests
        self.create_service(
            ArithmeticOperator,
            'arithmetic_operator',
            self.handle_arithmetic_operator,
            callback_group=self.callback_group,
        )
        self.get_logger().info('Calss service server is ready.')

    def get_arithmetic_argument(self, msg):
        """Callback for receiving arithmetic arguments."""
        self.argument_a = msg.argument_a
        self.argument_b = msg.argument_b
        self.argument_c = msg.argument_c
        self.get_logger().info(f'Received: a={self.argument_a}, b={self.argument_b}, c={self.argument_c}')

    def handle_arithmetic_operator(self, request, response):
        """Service handler for processing arithmetic operations."""
        operator1 = request.arithmetic_operator1
        operator2 = request.arithmetic_operator2

        # First operation: a and b
        intermediate_result = self.perform_operation(self.argument_a, self.argument_b, operator1)
        # Second operation: intermediate result and c
        final_result = self.perform_operation(intermediate_result, self.argument_c, operator2)

        response.arithmetic_result = final_result
        self.get_logger().info(f'Processed result: {final_result}')
        return response

    def perform_operation(self, x, y, operator):
        """Performs arithmetic operation based on the operator."""
        try:
            if operator == 1:  # Addition
                return x + y
            elif operator == 2:  # Subtraction
                return x - y
            elif operator == 3:  # Multiplication
                return x * y
            elif operator == 4:  # Division
                if y == 0:
                    self.get_logger().warn('Division by zero encountered. Returning 0.0 as result.')
                    return 0.0
                return x / y
            else:
                self.get_logger().error(f'Invalid operator: {operator}. Returning 0.0.')
                return 0.0
        except Exception as e:
            self.get_logger().error(f'Error during calculation: {str(e)}')
        return 0.0



def main(args=None):
    rclpy.init(args=args)
    node = Calss()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Calss node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


```

**calsc.py** <br/>
```
import rclpy
from rclpy.node import Node
from msg_srv_action_interface_example.srv import ArithmeticOperator
import random


class Calsc(Node):
    """
    Service Client Node for Arithmetic Operations.
    Sends requests with two random operators to the service server.
    """

    def __init__(self):
        super().__init__('calsc')
        self.client = self.create_client(ArithmeticOperator, 'arithmetic_operator')

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the service server...')

        self.get_logger().info('Calsc client is ready.')

    def send_request(self):
        """Sends a service request with two random operators."""
        request = ArithmeticOperator.Request()
        request.arithmetic_operator1 = random.randint(1, 4)  # Random operator 1
        request.arithmetic_operator2 = random.randint(1, 4)  # Random operator 2

        self.get_logger().info(f'Sending operators: {request.arithmetic_operator1}, {request.arithmetic_operator2}')
        return self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    client_node = Calsc()
    future = client_node.send_request()

    try:
        while rclpy.ok():
            rclpy.spin_once(client_node)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    client_node.get_logger().error(f'Error during service call: {str(e)}')
                else:
                    client_node.get_logger().info(f'Result received: {response.arithmetic_result}')
                break
    except KeyboardInterrupt:
        client_node.get_logger().info('Keyboard Interrupt detected.')
    finally:
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```
## Action Server Client
**calas.py** <br/>
```
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from msg_srv_action_interface_example.action import ArithmeticChecker
from msg_srv_action_interface_example.msg import ArithmeticArgument
from msg_srv_action_interface_example.srv import ArithmeticOperator


class Calas(Node):
    """
    Action Server Node for ArithmeticChecker.
    Receives goals from the client and processes them iteratively.
    """

    def __init__(self):
        super().__init__('calas')
        self.argument_a = 10.0
        self.argument_b = 5.0
        self.argument_c = 2.0

        self.action_server = ActionServer(
            self, ArithmeticChecker, 'arithmetic_checker', self.execute_callback
        )

        self.get_logger().info('Calas action server is ready.')

    def execute_callback(self, goal_handle):
        """Handles the received goal and provides feedback during execution."""
        self.get_logger().info(f'Received goal: {goal_handle.request.goal_sum}')
        goal_sum = goal_handle.request.goal_sum

        total_sum = 0.0
        feedback_msg = ArithmeticChecker.Feedback()
        feedback_msg.formula = []

        while total_sum < goal_sum:
            # Perform the operation: (a + b) * c
            intermediate_result = self.argument_a + self.argument_b
            final_result = intermediate_result * self.argument_c
            total_sum += final_result

            # Feedback to the client
            formula_str = f'({self.argument_a} + {self.argument_b}) * {self.argument_c} = {final_result}'
            feedback_msg.formula.append(formula_str)
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Feedback: {formula_str}, Running Total: {total_sum}')
            time.sleep(1)  # Simulate processing time

        goal_handle.succeed()

        # Final result
        result = ArithmeticChecker.Result()
        result.all_formula = feedback_msg.formula
        result.total_sum = total_sum
        return result


def main(args=None):
    rclpy.init(args=args)
    node = Calas()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down calas node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```

**calac.py** <br/>
```
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from msg_srv_action_interface_example.action import ArithmeticChecker


class Calac(Node):
    """
    Action Client Node for ArithmeticChecker.
    Sends a goal to the action server and handles feedback and results.
    """

    def __init__(self):
        super().__init__('calac')
        self.action_client = ActionClient(self, ArithmeticChecker, 'arithmetic_checker')

    def send_goal(self, goal_sum):
        """Sends a goal to the action server."""
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server not available.')
            return

        goal_msg = ArithmeticChecker.Goal()
        goal_msg.goal_sum = goal_sum
        self.get_logger().info(f'Sending goal: {goal_sum}')

        # Send the goal asynchronously
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handles the server's response to the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return
        self.get_logger().info('Goal accepted by server.')

        # Get the result asynchronously
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handles feedback received from the server."""
        feedback = feedback_msg.feedback.formula
        self.get_logger().info(f'Received feedback: {feedback}')

    def result_callback(self, future):
        """Handles the result received from the server."""
        result = future.result().result
        self.get_logger().info(f'Final Result - Formulas: {result.all_formula}, Total Sum: {result.total_sum}')


def main(args=None):
    rclpy.init(args=args)
    node = Calac()

    try:
        node.send_goal(120.0)  # Example: Sending a goal to calculate a sum of 100
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down calac node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```
