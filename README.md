# grace_ros
ROS wrapper for GRACE - GeometRic ApproaCh to mutual Engagement

Create your ros workspace
```
cd ~
mkdir -p ros_ws/src
```

Get the current ros package
```
cd ~/ros_ws/src
git clone git@github.com:vignif/grace_ros.git
```

Explicit external dependecy of GRACE
git@github.com:vignif/grace.git


```
cd ~/ros_ws
python -m venv venv_grace
source venv_grace/bin/activate
```

```
pip install --upgrade pip
wget https://github.com/vignif/grace/releases/download/v1.0.0/grace-1.0.0-py3-none-any.whl
pip install grace-1.0.0-py3-none-any.whl
pip install empy catkin_pkg rospkg PySide2
```

Build your package

```
catkin build
```

Source the bin of the ros package

```
source ./devel/setup.sh
```

You should be ready to go now.

Try to run the demo communication with:

```
roslaunch grace_ros demo.launch
```

Install dependencies for visualization with:

```
sudo apt-get install ros-noetic-rqt-common-plugins
```

At this point grace is running and you should have:

- An rviz visualization pops up showing a human reference frame moving with respect to the robot.

- The real-time plot of the computed mutual engagement (published on the topic `/mutual_engagement`) 


### Play around

Inspect the current level of the mutual engagement with:
```
rostopic echo /mutual_engagement
```

### Dynamic reconfigure

The weights between the features as well as the epsilon of the proxemics feature can be tuned in real-time.
These values are published in the reconfigure server with names:

- "proxemics_weight"
- "gaze_weight"
- "proxemics_epsilon"

You can fire the reconfigure server in another terminal with:

```
rosrun rqt_reconfigure rqt_reconfigure 
```

and see how they influence the response on the value of `/mutual_engagement`