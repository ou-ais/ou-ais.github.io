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
Also, you can write a roslanch file for setting static transforms.
```launch
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
