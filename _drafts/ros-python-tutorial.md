ROS Python Tutorial (beginner level)
======
## 1. Create & Build workspace

Create a ROS workspace:
```bash
$ mkdir -p ~/one_ws/src
$ cd ~/one_ws/
$ catkin build
```
Ather that, '/devel' ,'/build' and '/log' floders will be generated.
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

And add `catkin_python_setup()` into the CMakeLists.txt of your project. *DO NOT*  use `python setup.py install`.

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
Topic is continuous data flow (e.g. sensor data or robot state). Data might be published and subscribed at any time. It can be one-to-one or, one-to-many dependence. (Imagine in the _converyor belt sushi_, the topic is likes a conveyor belt that keeps moving and anyone can take one or more sushi from it.)

### Message 
A message is a simple data structure which sended in ros topics. Nodes communicate with each other by publish messages to topics. (Like the sushi converied on the belt.)

There are many messsage types in ROS, such as [geometry_msgs](http://wiki.ros.org/geometry_msgs?distro=melodic), [sensor_msgs](http://wiki.ros.org/sensor_msgs?distro=melodic) and [moveit.msgs](http://wiki.ros.org/moveit_msgs) etc.

### Publish
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
### Subscribe
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
Suppose you need to subscribe from **multiple synchronized topics** (e.g. RGB image and camera infomation) in single callback, [message_filers](http://wiki.ros.org/message_filters) can be used:
```python
import message_filters
from sensor_msgs.msg import Image, CameraInfo

def callback(image, camera_info):
  # do some work ...

image_sub = message_filters.Subscriber('image', Image)
info_sub = message_filters.Subscriber('camera_info', CameraInfo)

ts = message_filters.TimeSynchronizer([image_sub, info_sub], queze_size=10)
'''
Alternatively, approximately synchronizes messages by their timestamps, you can use another function like this: 
ts = message_filters.ApproximateTimeSynchronizer([mode_sub, penalty_sub], queze_size=10, slop=0.1, allow_headerless=True)
where argument 'slop' defines the delay (in seconds) which message can be synchronized.
'''
ts.registerCallback(callback)
rospy.spin()
```
### Using numpy with rospy
We can use `rospy.numpy_msg` module with the `numpy_msg()` wrapper, which allows Nodes to publish/subscribe messages directly into numpy arrays.

The publish code probably look something like this:
```python
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats # float32[] data
import numpy

def talker():
    pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        Array = numpy.array([1.0, 2.1, 3.2, 4.3, 5.4, 6.5], dtype=numpy.float32)
        pub.publish(Array)
        r.sleep()

if __name__ == '__main__':
    talker()
```
Similarly, the subscribe code will looks like:
```python
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

def callback(data):
    print rospy.get_name(), 'I heard %s"%str(data.data)
    # do some work ...

def listener():
    rospy.init_node('listener')
    rospy.Subscriber('floats', numpy_msg(Floats), callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```

## 6. Services


