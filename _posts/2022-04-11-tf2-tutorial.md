---
title: 'TF2 tutorial'
publish: 2022-04-05
permalink: /posts/2022/04/tf2-tutorial/
tags:
  - TF2
  - ROS
---

tf2 is the second version of the transform library, which lets the user keep track of multiple coordinate frame over time.

See the first version tf [here](http://wiki.ros.org/tf).

Main reference is [tf2 tutorial](http://wiki.ros.org/tf2/Tutorials)

## 1. Broadcaster
Broadcaster is the tool to create frames with relation to existed frames. There are two types of broadcaster: _static_ and non-static_ in tf2.

Static broadcaster publishes transforms that are assumed not to change (or very infrequently). So it doesn't need to be periodically published. For transforms that do change, this is not suitable, so you have to use regular broadcaster instead.

Example for _Static broadcaster_:
```python
#!/usr/bin/env python
import rospy
import sys
import tf  # for tf.transformation

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    if len(sys.argv) < 8:
        rospy.logerr('Invalid number of parameters\nusage: '
                     './static_turtle_tf2_broadcaster.py '
                     'child_frame_name x y z roll pitch yaw')
        sys.exit(0)
    else:
        if sys.argv[1] == 'world':
            rospy.logerr('Your static turtle name cannot be "world"')
            sys.exit(0)

        rospy.init_node('my_static_tf2_broadcaster')
        
        # use 'StaticTransformBroadcaster' API
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        # Create 'transformStamped' message
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        # The time stamp is the now time.
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = sys.argv[1]

        static_transformStamped.transform.translation.x = float(sys.argv[2])
        static_transformStamped.transform.translation.y = float(sys.argv[3])
        static_transformStamped.transform.translation.z = float(sys.argv[4])

        # Convey euler to quaternion
        quat = tf.transformations.quaternion_from_euler(
                   float(sys.argv[5]),float(sys.argv[6]),float(sys.argv[7]))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        # Publish static transform
        broadcaster.sendTransform(static_transformStamped)
        # Program block
        rospy.spin()
```
You can also use `static_transform_publisher` command-line to publish static transform quickly instead of writing a program.
```bash
$ static_transform_publisher x y z yaw pitch roll header_frame_id child_frame_id 
# Or use quaternion
static_transform_publisher x y z qx qy qz qw header_frame_id child_frame_id 
```
Also, you can write a roslaunch file for setting static transforms.
```
<launch>
<node pkg="tf2_ros" type="static_transform_publisher" name="child_frame_id" args="1 0 0 0 0 0 1 header_frame_id child_frame_id" />
</launch>
```
Example for _Regular broadcaster_:
```python
#!/usr/bin/env python  
import rospy
# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg
import turtlesim.msg


def handle_turtle_pose(msg, turtlename):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    turtlename = rospy.get_param('~turtle')
    # rospy.Subscriber('topics_name', message_type, callback, callback_argus)
    # 'turtlename' is the callback (handle_turtle_pose) arguments
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
```
If you see the upon two examples, you might find _static broadcaster_ was called once in the program, 
and the _regular broadcaster_ was updated constantly from the subscribed message.

## 2. Listener
Listener is the tool to get access to frame transformations.

```python
#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.wait_for_service('spawn')
    # ServiceProxy(service_name, service_type)
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    turtle_name = rospy.get_param('turtle', 'turtle2')
    spawner(4, 2, 0, turtle_name)

    turtle_vel = rospy.Publisher('%s/cmd_vel' % turtle_name, geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0) # 10hz
    while not rospy.is_shutdown():
        try:
            # trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())
            # Note: This is often failing, telling you that the frame does not exist or that the data is in the future. To fix this problem, using below code instead is usual.
            trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time.now(), rospy.Duration(1.0))
            # So lookup_transform() will actually block until the transform between the two frames becomes available. Once the timeout ('rospy.Duration(1.0)' has been reached, an exception will be raised if the transform is still not available.)

            # trans is the message of 'geometry_msgs.msg.TransformStamped()'

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.Twist()

        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        turtle_vel.publish(msg)

        rate.sleep()
```

_*_ At some times, you want to publish a fixed frame which does't change over time in relation to the parent frame. Then your code will look like this:
```python
#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg


class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "turtle1"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "carrot1"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 2.0
            t.transform.translation.z = 0.0

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()
```
The code is very similar to the example in the tf2 broadcaster. Only here is using 'TFMessage' to publish a message, instead of 'TransformBroadcaster'.

## 3. Quaternion Basics
ROS uses quaternions to represent rotation. A quaternion has 4 components (x,y,z,w). The commonly-used unit quaternion that yields no rotation about the x/y/z axes is (0,0,0,1). The normal of a quaternion should be 1. ROS uses two quaternion data types: _msg_ and _tf_. To covert between them, use the methods of _tf_geometry_msgs_.
```python
from geometry_msgs.msg import Quaternion

# Create a list of floats, which is compatible with tf
quat_tf = [0, 1, 0, 0]
quat_msg = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
```
It is easy for humans to think of rotation about axes but hard to think in terms of quaternions. So it is a good way to think in RBY the convert to a quaternion.
```python
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    q = quaternion_from_euler(1.57, 0, -1.57)
    print ('The quaternion representation is:', q)
```
Two quaternion could be multiplied to represent a new quaternion. The order of this multiplication matters.
```python
from tf.transformation import quaternion_from_euler, quaternion_multiply

q_orig = quaternion_from_euler(0, 0, 0)
q_rot = quaternion_from_euler(3.14, 0, 0)
q_new = quaternion_multiply(q_rot, q_orig)
print(q_new)
```
An easy way to invert a quaternion is to negate the w-component. In python, it might be like `q[3] = -q[3]`.

## 4. Debug tf2
If you want to find out whether tf2 knows about transform between _turtle3_ and _turtle1_:
```bash
$ rosrun tf2 tf2_echo turtle3 turtle1
```
If you like to get a graphical representation of this, type:
```bash
$ rosrun tf2_tools view_frames
$ evince frames.pdf
```
If you want to check the timestamps of transform between _turtle2_ and _turtle_ at time "now", run:
```bash
$ rosrun tf2 tf2_monitor turtle2 turtle1
```
The output shows there if an average delay of 8 milliseconds between _turtle2_ to _turtle_. This means that tf2 can only transform between the turtles after 8 milliseconds are passed.

## 5. KDL
The Kinematics and Dynamics Library (KDL) develops an application  independent framework for modelling and computation of kinematics chains, such as robots. It can be used for 3D frame and vector transformations, kinematics and dynamics of kinematics chains and kinematics of kinematics trees. Here we use its python wrapper (PyKDL) for examples.
```python
import PyKDL
import tf_conversions import posemath
from geometry_msgs.msg import Pose

# Frame 
f1 = PyKDL.Frame(PyKDL.Rotation.RPY(0, 1, 0),
                PyKDL.Vector(3, 2, 4))

# Get the origin (a vector) of the frame
origin = f1.p

# Get the x component of the origin
x = origin.x()
# Or
x = origin[0]

# Get the rotation of the frame
rot = f.M

# Get the ZYX Euler angles from the rotation
[Rz, Ry, Rx] = rot.GetEulerZYX()

# Get the RPY (fixed axis) from the rotation
[R, P, Y] = rot.GetRPY()

# ROS message pose()
pose = Pose()

# Create the pose into a KDL frame
f2 = posemath.fromMsg(pose)

# Combine the two frame
f21 = f2 * f1

# Convert the result back to a pose message
pose = toMsg(f21)

# Define a point
point = PyKDL.Vector(1, 0, 0)
print(point)

# Transform this point with f1 transform
newpoint = f * point
print(newpoint)
```