---
title: 'ROS python tutorial (beginner-level)'
publish: 2022-04-04
permalink: /posts/2022/04/rospy-tutorial-beginner/
tags: 
  - ROS
---

Here are some commanders and examples for ros python tutorial beginner level. Main reference is [roswiki](http://wiki.ros.org/)

ROS Python Tutorial (beginner level)
======
## 1. Create & Build workspace

Create a ROS workspace:
```bash
$ mkdir -p ~/one_ws/src
$ cd ~/one_ws/
$ catkin build
```
After that, '/devel' ,'/build' and '/log' folders will be generated.
Do not forget to source the bash file:
```bash
$ source ~/one_ws/devel/setup.bash
```

## 2. Create & Build rospackage

Create a catkin package:
```bash
$ cd ~/one_ws/src
$ catkin_create_pkg <package_name> (with dependence: e.g.) std_msgs rospy roscpp
``` 
Then build it:
```bash
$ catkin build <package_name>
```

## 3. PYTHON Configuration
ROS package containing python code could be install to PYTHONPATH use a `setup.py` when you source the devel.
```python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['package_name'],
    # scripts=['bin/myscript'],
    package_dir={'': 'src'}
)

setup(**d)
``` 
Details are described in [Handing of setup.py](http://docs.ros.org/en/hydro/api/catkin/html/user_guide/setup_dot_py.html)

And add `catkin_python_setup()` into the CMakeLists.txt of your project. **DO NOT**  use `python setup.py install` to install it manually.

## 4. Node
Call `rospy_init_node()` first which initializes the node in a rospy program is necessary. You can only have one node in a rospy program, so you can only call it once.

A simple example for node initialization and shut down:
```python
import rospy
rospy.init_node('node_name', anonymous=True)
# The anonymous keyword argument is mainly used for nodes where you normally expect many of them to be running and don't care about their names (e.g. tools, GUIs). 

while not rospy.is_shutdown():
    # do some work

rospy.spin()
```
## 5. Topics
Topic is continuous data flow (e.g. sensor data or robot state). Data might be published and subscribed at any time. It can be one-to-one or, one-to-many dependence. (Imagine in the _conveyor belt sushi_, the topic is likes a conveyor belt that keeps moving and anyone can take one or more sushi from it.)

### 5.1 Message 
A message is a simple data structure which sended in ros topics. Nodes communicate with each other by publish messages to topics. (Like the sushi convey on the belt.)

There are many message types in ROS, such as [geometry_msgs](http://wiki.ros.org/geometry_msgs?distro=melodic), [sensor_msgs](http://wiki.ros.org/sensor_msgs?distro=melodic) and [moveit.msgs](http://wiki.ros.org/moveit_msgs) etc.

* `msg`: msg files are simple text files that describe the field of a ROS message.
Here is an example of msg:
```
# message_type                   message_name
  Header                           header
  string                           frame_id
geometry_msgs.PoseWithCovariance pose
```
* `srv`: srv files are compose of two part: a request and response.
```
# (request) type    name
            int64   a
            int64   b
---
# (response)type    name
            int64   sum
```
See also about how to create and use msg & srv from [here](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv). 

### 5.2 Publish
You can call `rospy.Publisher('topic_name', message_type, queue_size)` to publish a message. In which, `queue_size` is the size of outgoing message queue used for publishing (Saved how many message waiting for sending). [Here](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers) provides some tips for choosing a good queue_size:

* Set _queue_size=rate_, if you just send one message at a fixed rate.
* Set _queue_size=1_, if you only care about the latest message (i.e. never send older message if a newer one exists likes a sensor).
* Set _queue_size=10 or more_, is user interface message (e.g. a push button) that would prevent missing a change in value.

Example for publish:
```python
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('topic_name', String, queue_size=10)  # queue_size = publish_rate
rospy.init_node('node_name')
r = rospy.Rate(10)  # pushlish frequency is 10hz. 
while not rospy.is_shutdown():
   pub.publish('hello world')
   r.sleep()
``` 
### 5.3 Subscribe
Example for subscribe from single topic to callback:
```python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo('I heard %s', data.data)
    # do some work ...

def listener():
    rospy.init_node('node_name')
    rospy.Subscriber('topic_name', String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

Suppose you need to subscribe from **multiple synchronized topics** (e.g. RGB image and camera information) in single callback, [message_filers](http://wiki.ros.org/message_filters) can be used, for example: [`message_filters.py`](https://github.com/ou-ais/ros-python-example/blob/main/message_filters.py).

### * Using numpy with rospy
We can use `rospy.numpy_msg` module with the `numpy_msg()` wrapper, which allows Nodes to publish/subscribe messages directly into numpy arrays.

The publish code probably look something like this: [`ros_numpy_talker.py`](https://github.com/ou-ais/ros-python-example/blob/main/ros_numpy_talker.py)

Similarly, the subscribe code will looks like: [`ros_numpy_listener.py`](https://github.com/ou-ais/ros-python-example/blob/main/ros_numpy_listener.py)

## 6. Services
Services are another way that exchange messages between each node. Services allow nodes to send a **request** and receive a **response**. The big difference between services and topics is that services only work (e.g. sending messages) when you call it, whereas topics are continuously sending messages until you shut down it. (Again, imagine services are like _waiters_ in the conveyor belt sushi, who will bring the prepared sushi to you when you order.)

Refer to [`add_two_ints_server.py`](https://github.com/ou-ais/ros-python-example/blob/main/script/add_two_ints_server.py) and [`add_two_ints_client.py`](https://github.com/ou-ais/ros-python-example/blob/main/script/add_two_ints_client.py) for how to write a service and client and how to configure your `CMakeLists.txt` and `package.xml` files in a ROS package.

## 7. Actions
In some case, however, if the service takes a long time to execute, the user might want the ability to **cancel** the request during execution or get **feedback** about how the request is progressing. The `actionlib` can implement these for you.

An action specification defines the _Goal_, _Feedback_ and _Result_ messages with which clients and servers communicated.

See [5. .action File](http://wiki.ros.org/actionlib) for some information about how to create `.action` file, and how to use it in your ros package.

A example about generating fibonacci sequence by ros actionlib could be seen a actionlib [server](https://github.com/ou-ais/ros-python-example/blob/main/script/fibonacci_action_server.py) and callback-based [client](https://github.com/ou-ais/ros-python-example/blob/main/script/fibonacci_action_client.py).

## 8. Rosbag
The _rosbag_ package provides a command-line tool for recording/playing message from all or special topics.
### 8.1 Recording and playing data
Use '_turtlsesim_node_' and '_turtle_teleop_key_' nodes of rospackage '_turtlesim_' for example.
First, execute the following commands in separate terminals:
```bash
# 1st terminal
roscore
# 2nd terminal
rosrun turtlesim turtlesim_node
# 3rd terminal
rosrun turtlesim turtle_teleop_key
```
You can make a temporary directory to save the recorded data or not.
```bash
mkdir bagfile && cd bagfile
```
If you want to record all published topics, you can use option `-a`:
```bash
rosbag roscord -a
```
Then, you can press the arrow key to move the turtle around the screen. When you finished your recording, press **Ctrl + c** in the rosbag terminal.

If you want record the topics of interest to them. Use `rostopic list -v` first to check the list of published topics and message type corresponded to it.

Here we just record two topics of the '_turtle1_' node:
```bash
rosbag record -O bagfile_name /turtle1/cmd_vel /turtle1/pose
```
You can use `rosbag info <bagfile_name>` to see the contents of the bagfile, and use `rosbag play <bagfile_name>` to replay the bag file to represent behavior of the turtle moved previous.

If you execute:
```bash
rosbag play -r 2 <bagfile_name>
```
you might see the turtle's trajectory that would have the result had you press your keyboard commands twice as fast.

### 8.2 Reading message from a bag file
See use the [`ros_readbagfile`](http://wiki.ros.org/ROS/Tutorials/reading%20msgs%20from%20a%20bag%20file) scrip to easily read topics of interest.
