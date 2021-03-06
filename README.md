# Home Service Robot

The goal of this project is to bring together a couple of ROS/RViz/Gazebo concepts:

- World and odometry simulation in Gazebo,
- Visualization of the odometry, sensor and localization data, as well as virtual objects in RViz,
- SLAM via gmapping, and
- Adaptive Monte-Carle Localization via the AMCL package.

The robot in this scenario is tasked with picking up a virtual item from a start position,
visualized by a red sphere in RViz, in order to bring to to a goal position.

![](.readme/pickup.webp)

Navigation to the start and end positions is implemented in [`pick_objects_node.cpp`](src/pick_objects/src/pick_objects_node.cpp) and
visualization of the item is implemented in [`add_markers_node.cpp`](src/add_markers/src/add_markers_node.cpp).

![](.readme/dropoff.webp)

Note that in both cases, the start and goal positions are provided statically, and the whole
implementation can be (drastically) improved by publishing pick-up and drop-off locations via a dedicated topic.

Previous projects have shown that attempting to run SLAM on a synthetic world like this are bound to fail.
Here's the intermediate result after leaving from the start position and taking a left turn around the
center triangular room, back to the start position:

![](.readme/slam.png)

Because this clearly fails, the map was instead created using the `pgm_map_creator` node directly from Gazebo
and then retouched to add in doors.
Adding more (natural) textures and items to this world would have improved SLAM, but also
deviated a bit from the main idea of this repo. Again, investing more time here would make this quite an
interesting project overall.

---

To run the code, execute either one of these scripts; each of them will spin up all nodes required
for the individual test task at hand: 

- [`src/scripts/test_slam.sh`](src/scripts/test_slam.sh) for testing SLAM,
- [`src/scripts/test_navigation.sh`](src/scripts/test_navigation.sh) for testing AMCL navigation,
- [`src/scripts/pick_objects.sh`](src/scripts/pick_objects.sh) for testing navigation automation,
- [`src/scripts/add_markers.sh`](src/scripts/add_markers.sh) for testing markers,
- [`src/scripts/home_service.sh`](src/scripts/home_service.sh) for running the actual project.

Make sure to `catkin_make` and `source devel/setup.bash` the workspace before.

---

To build the project, you can try executing `./run-nvidia.sh` to drop
into an X11 aware Docker container with NVIDIA GPU support.


## Docker environment

An issue exists with ROS Kinetic and TurtleBot in Docker, since the TurtleBot packages rely on
`librealsense`, which in turn depends on a Kernel module that cannot be installed. This is pointed out by
[IntelRealSense/librealsense#4781](https://github.com/IntelRealSense/librealsense/issues/4781), which
(thankfully) also provides a workaround. That workaround is implemented in [`docker/build-librealsense.sh`](docker/build-librealsense.sh) and is executed as part of the Docker image build in [`build-docker.sh`](build-docker.sh).

## Building with CLion IDE

**Note:** This does not _really_ work, as CLion will be unable to find generated headers. It's still a bit
          better than doing everything the hard way.

The full requirements for setting up CLion are given in the [sunsided/robond-ros-docker](https://github.com/sunsided/robond-ros-docker)
repository. In short, run SSHD in Docker, configure a Remote Host build to connect to it, then configure
the your build settings for ROS. For this repo and the included Dockerfile, this configuration will do:

**CMake options:**

```
-DCATKIN_DEVEL_PREFIX:PATH=/workspace/devel -DCMAKE_PREFIX_PATH=/workspace/devel;/opt/ros/kinetic;/opt/ros/kinetic/share
```

**Environment:**

```
ROS_ROOT=/opt/ros/kinetic/share/ros;ROS_PACKAGE_PATH=/workspace/src:/opt/ros/kinetic/share;ROS_MASTER_URI=http://localhost:11311;ROS_PYTHON_VERSION=2;ROS_VERSION=1;ROSLISP_PACKAGE_DIRECTORIES=/workspace/devel/share/common-lisp;ROS_DISTRO=kinetic;ROS_ETC_DIR=/opt/ros/kinetic/etc/ros;PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages;PKG_CONFIG_PATH=/workspace/devel/lib/pkgconfig:/opt/ros/kinetic/lib/pkgconfig:/opt/ros/kinetic/lib/x86_64-linux-gnu/pkgconfig;LD_LIBRARY_PATH=/workspace/devel/lib:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH;PATH=/opt/ros/kinetic/bin:$PATH
```
