#!/usr/bin/env bash
# see https://github.com/IntelRealSense/librealsense/issues/4781

export DEBIAN_FRONTEND=noninteractive

# basic container setup
apt-get update
apt-get install -y lsb-release
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
apt-get update

# dependencies needed by librealsense. `deb -i` will not resolve these
apt-get install -y binutils cpp cpp-5 dkms fakeroot gcc gcc-5 kmod libasan2 libatomic1 libc-dev-bin libc6-dev libcc1-0 libcilkrts5 libfakeroot libgcc-5-dev libgmp10 libgomp1 libisl15 libitm1 liblsan0 libmpc3 libmpfr4 libmpx0 libquadmath0 libssl-dev libssl-doc libtsan0 libubsan0 libusb-1.0-0 libusb-1.0-0-dev libusb-1.0-doc linux-headers-4.4.0-159 linux-headers-4.4.0-159-generic linux-headers-generic linux-libc-dev make manpages manpages-dev menu patch zlib1g-dev
apt-get install -y libssl-dev libssl-doc libusb-1.0-0 libusb-1.0-0-dev libusb-1.0-doc linux-headers-4.4.0-159 linux-headers-4.4.0-159-generic linux-headers-generic zlib1g-dev

# modify librealsense deb (unpack, replace script, repack)
apt-get download ros-kinetic-librealsense
dpkg-deb -R ros-kinetic-librealsense*.deb ros-rslib/

wget https://gist.githubusercontent.com/dizz/404ef259a15e1410d692792da0c27a47/raw/3769e80a051b5f2ce2a08d4ee6f79c766724f495/postinst
chmod +x postinst
cp postinst ros-rslib/DEBIAN

PACKAGE_VERSION=$(cat ros-rslib/DEBIAN/control | grep Version | cut -d'-' -f1 | cut -d' ' -f2)
PACKAGE_DATE=$(cat ros-rslib/DEBIAN/control | grep Version | cut -d'-' -f3)
PACKAGE_NAME=ros-kinetic-librealsense_${PACKAGE_VERSION}-0xenial-${PACKAGE_DATE}_icrlab_amd64.deb

echo ${PACKAGE_VERSION} ${PACKAGE_DATE}

dpkg-deb -b ./ros-rslib/ ${PACKAGE_NAME}

# install container friendly libsense
dpkg -i ${PACKAGE_NAME}

# lock from updates
apt-mark hold ros-kinetic-librealsense
