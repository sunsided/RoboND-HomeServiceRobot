FROM sunside/ros-gazebo-gpu:kinetic-nvidia

USER root

# Install required ROS packages, as well as GDB for debugging.
RUN apt-get update && apt-get install -y \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-effort-controllers \
    ros-kinetic-joint-state-controller \
    gdb \
 && rm -rf /var/lib/apt/lists/*

# Install packages for Where Am I? project
RUN apt-get update && apt-get install -y \
    ros-kinetic-navigation \
    ros-kinetic-map-server \
    ros-kinetic-move-base \
    ros-kinetic-amcl \
    libignition-math2-dev \
    protobuf-compiler \
 && rm -rf /var/lib/apt/lists/*

# librealsense cannot be installed in this container; see
# https://github.com/IntelRealSense/librealsense/issues/4781

# Installation of librealsense fix
RUN mkdir -p /tmp/librealsense
COPY docker/build-librealsense.sh /tmp/librealsense/build-librealsense.sh
COPY docker/postinst.fix /tmp/librealsense/postinst
RUN cd /tmp/librealsense && sh /tmp/librealsense/build-librealsense.sh \
 && rm -rf /tmp/librealsense \
 && rm -rf /var/lib/apt/lists/*

# Install packages the Home Service Robot? project
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-kinetic-gmapping \
    ros-kinetic-joy \
    ros-kinetic-turtlebot \
    ros-kinetic-turtlebot-gazebo \
    ros-kinetic-turtlebot-navigation \
    ros-kinetic-turtlebot-interactions \
 && rm -rf /var/lib/apt/lists/*

# Allow SSH login into the container.
# See e.g. https://github.com/JetBrains/clion-remote/blob/master/Dockerfile.remote-cpp-env
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin yes'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd

# Start SSH server and run bash as the ros user.
USER ros
ENTRYPOINT sudo service ssh restart && bash
