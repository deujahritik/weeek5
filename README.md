# weeek5

# Managing Dependencies with rosdep
## What is rosdep?
*`rosdep` is ROS’s dependency management utility that can work with ROS packages and external libraries. rosdep is a command-line utility for identifying and installing dependencies to build or install a package.*

## How does rosdep work?
*`rosdep` will check for package.xml files in its path or for a specific package and find the rosdep keys stored within. Once the packages are found, they are installed and ready to go!*

## How do I use the rosdep tool?
We are prepared to use the utility now that we have a basic understanding of rosdep, package.xml, and rosdistro. First, if using rosdep for the first time, it must be initialized using:

```
sudo rosdep init
rosdep update
```
<img width="433" alt="Screenshot_2" src="https://user-images.githubusercontent.com/92029196/196000334-0bc9adc8-17be-4e8f-83ca-9c825b9c048b.png">



We can finally use rosdep install to install dependencies. This is often called once across a workspace that contains numerous packages in order to install all dependencies. If the workspace's root contained the source-code-containing directory src, a call for that might look like the following.


```
rosdep install --from-paths src -y --ignore-src
```
<img width="333" alt="Screenshot_1" src="https://user-images.githubusercontent.com/92029196/196000309-4dd4c008-b3cf-440f-b664-3770d415f871.png">

# Creating an action
## Prerequisites

Colcon and ROS 2 must be installed.

Establish a workspace and a package called "action tutorials interfaces":

(Remember to source first from your ROS 2 installation.)
```
mkdir -p ros2_ws/src #you can reuse existing workspace with this naming convention
cd ros2_ws/src
ros2 pkg create action_tutorials_interfaces
```
<img width="403" alt="Screenshot_3" src="https://user-images.githubusercontent.com/92029196/196000477-817e96db-0c2c-4fa3-9c57-40a7d8aef42e.png">

# Tasks
## 1. Defining an action
Actions are described in .action files with the following format:
```
# Request
---
# Result
---
# Feedback
```
Create a directory called "action" in our ROS 2 package called "action tutorials interfaces":

```
cd action_tutorials_interfaces
mkdir action
```
<img width="374" alt="Screenshot_4" src="https://user-images.githubusercontent.com/92029196/196000505-d1d6a558-05e6-4bfe-bbbb-32ac95f6a590.png">


Make a file called Fibonacci in the action directory. action that includes the following information:
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```
## 2. Building an action
The definition of the new Fibonacci action type must be sent to the pipeline that generates Rosidl code before we can use it in our code.

The following lines need be added to our CMakeLists.txt before the ament package() line in the action tutorials interfaces to achieve this:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```
<img width="441" alt="Screenshot_5" src="https://user-images.githubusercontent.com/92029196/196000566-ee571b72-e5cc-45d9-a71f-a509d194d98a.png">


We must also include the necessary dependencies in our package.xml file:
```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<depend>action_msgs</depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```
The fact that action definitions contain additional metadata means that we must rely on action msgs (e.g. goal IDs).

The package containing the definition of the Fibonacci action should now be able to be built:
```
# Change to the root of the workspace
cd ~/ros2_ws
# Build
colcon build
```
<img width="434" alt="Screenshot_6" src="https://user-images.githubusercontent.com/92029196/196000590-acd154ef-ebad-4b20-959d-f205f8f66c21.png">


We are done!

Action types will often start with the term "action" and the package name. As a result, our new action will be referred to by its complete name, action tutorials interfaces/action/Fibonacci.

Using the command line tool, we can verify that our action was built successfully:
```
# Source our workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that our action definition exists
ros2 interface show action_tutorials_interfaces/action/Fibonacci
```
<img width="435" alt="Screenshot_7" src="https://user-images.githubusercontent.com/92029196/196000613-23067963-4946-4b9c-a80c-acd61260f345.png">

The definition of the Fibonacci action should appear on the screen.

# Writing an action server and client
## Prerequisites
You will require both the Fibonacci.action interface from the previous tutorial, "Creating an action," and the action tutorials interfaces package.

## Tasks
## 1. Writing an action server
You should create a new file in your home directory called fibonacci action server.py and add the following code to it:

```
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```
Let's attempt to run our action server:
```
python3 fibonacci_action_server.py
```
<img width="444" alt="Screenshot_8" src="https://user-images.githubusercontent.com/92029196/196000652-45e82e7d-d4c5-4526-a7f5-c21223887a47.png">

We can communicate a goal via the command line interface to another terminal:

```
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```
<img width="445" alt="Screenshot_9" src="https://user-images.githubusercontent.com/92029196/196000672-7b223c10-bb37-42ab-af48-48e2d58d92d9.png">

You should see the logged message "Executing goal..." followed by a notice that the goal state was not established in the terminal that is running the action server. The aborted state is assumed by default if the goal handle state is not set in the execute callback.

The succeed() method on the goal handle can be used to show that the goal was successful:

```
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        goal_handle.succeed()

        result = Fibonacci.Result()
        return result
```
You should see the goal completed with the status SUCCEED if you restart the action server and send another goal at this point.

<img width="368" alt="Screenshot_10" src="https://user-images.githubusercontent.com/92029196/196000707-e2b4f46e-e81a-4bc6-afb8-f527ad096ea7.png">

<img width="368" alt="Screenshot_11" src="https://user-images.githubusercontent.com/92029196/196000714-18f9cf02-7541-4a7f-ac60-807818828441.png">
<img width="363" alt="Screenshot_12" src="https://user-images.githubusercontent.com/92029196/196000726-deac91e0-5829-4dde-b42c-fc9de674ce64.png">
<img width="357" alt="Screenshot_13" src="https://user-images.githubusercontent.com/92029196/196000740-ad7a7fa4-9e4f-4ab7-aeb8-8352975ac969.png">


Let's now make sure that our target execution computes and returns the specified Fibonacci sequence:

```
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')


        sequence = [0, 1]



        for i in range(1, goal_handle.request.order):

            sequence.append(sequence[i] + sequence[i-1])


        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = sequence

        return result
```
We compute the sequence, assign it to the result message field, and then proceed to the return.

Send another goal and restart the action server. The aim should be completed with the expected results in order.

## 1.2 Publishing feedback
The sequence variable will be swapped out, and the sequence will now be stored in a feedback message. We publish the feedback message and then fall asleep after each update of the feedback message in the for-loop for impact:
```
import time


import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')


        feedback_msg = Fibonacci.Feedback()

        feedback_msg.partial_sequence = [0, 1]


        for i in range(1, goal_handle.request.order):

            feedback_msg.partial_sequence.append(

                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1)


        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = feedback_msg.partial_sequence

        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```
Utilizing the command line tool with the --feedback option after restarting the action server, we can verify that feedback has now been published:
```
ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```
<img width="453" alt="Screenshot_14" src="https://user-images.githubusercontent.com/92029196/196000762-d837c8dc-5d12-459a-8ea4-ae81db728015.png">
<img width="530" alt="Screenshot_15" src="https://user-images.githubusercontent.com/92029196/196000777-e12bebd4-5b73-4560-a1ef-070d334af3ce.png">


## 2. Writing an action client
We'll limit the action client to just one file as well. Then, open a new file and name it fibonacci action client.py. Add the following boilerplate code to the new file:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
```
Let's test our action client by first launching the earlier-built action server:

```
python3 fibonacci_action_server.py
```
Run the action client in an other terminal.

```
python3 fibonacci_action_client.py
```
As the action server completes the goal, the following messages should be printed:
```
[INFO] [fibonacci_action_server]: Executing goal...
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
# etc.
```
<img width="370" alt="Screenshot_16" src="https://user-images.githubusercontent.com/92029196/196000797-4808a084-b28e-4851-9c4e-8178b55d8e0e.png">

The action client should begin and complete as soon as possible. We currently have a working action client, but we receive no feedback or results.


## 2.1 Getting a result
We must first obtain a goal handle for the goal that we sent. The result can then be requested using the goal handle.

The full code for this example is provided here:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

Go ahead and attempt to run our Fibonacci action client while an action server is running in a separate terminal!


```
python3 fibonacci_action_client.py
```
<img width="364" alt="Screenshot_17" src="https://user-images.githubusercontent.com/92029196/196000831-9211e39e-7966-43d6-a60e-02d6adbf72be.png">
<img width="368" alt="Screenshot_18" src="https://user-images.githubusercontent.com/92029196/196000853-5c7b6f48-e44b-4082-9ef4-505b10cef69b.png">

You should be able to see the goal being accepted and the outcome in the logs.

## 2.2 Getting feedback
We can send goals to our action client. Nice! However, it would be wonderful to hear some input regarding the goals we transmit from the action server.

Here is the whole code for this illustration:
```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```
Everything is ready for us. Your screen should display feedback if we run our action client.

```
python3 fibonacci_action_client.py
```

<img width="363" alt="Screenshot_20" src="https://user-images.githubusercontent.com/92029196/196000882-9c370b6f-4509-4e6d-903a-f53b83c22482.png">
<img width="364" alt="Screenshot_19" src="https://user-images.githubusercontent.com/92029196/196000885-49bd3ed8-7e38-4e0e-a091-290e240fe5f4.png">





