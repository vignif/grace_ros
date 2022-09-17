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

At this point grace is running and you should have:

- An rviz visualization pops up showing a human reference frame moving with respect to the robot.

- The real-time plot of the computed mutual engagement (published on the topic `/mutual_engagement`) 