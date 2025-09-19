#!/bin/bash -eu

# The BSD License
# Copyright (c) 2018 PickNik Consulting
# Copyright (c) 2014 OROCA and ROS Korea Users Group

#set -x

function usage {
    # Print out usage of this script.
    echo >&2 "usage: $0"
    echo >&2 "[-h|--help] Print help message."
    exit 0
}

# Parse command line. If the number of argument differs from what is expected, call `usage` function.
OPT=`getopt -o h -l help -- $*`
eval set -- $OPT
while [ -n "$1" ] ; do
    case $1 in
        -h|--help) usage ;;
        --) shift; break;;
        *) echo "Unknown option($1)"; usage;;
    esac
done

if ! command -v lsb_release &> /dev/null
then
    sudo apt-get install lsb-release
fi

version=`lsb_release -sc`
echo ""
echo "INSTALLING ROS --------------------------------"
echo ""
echo "Checking the Ubuntu version"
case $version in
  "trusty" | "xenial" | "bionic" | "focal" | "jammy")
  ;;
  *)
    echo "ERROR: This script will only work on Trusty / Xenial / Bionic / Focal / Jammy. Exit."
    exit 0
esac

case $version in
  "trusty")
  ROS_DISTRO="indigo"
  ;;
  "xenial")
  ROS_DISTRO="kinetic"
  ;;
  "bionic")
  ROS_DISTRO="melodic"
  ;;
  "focal")
  ROS_DISTRO="noetic"
  ;;
  "jammy")
  ROS_DISTRO="humble"

esac

relesenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
if [ "$relesenum" = "14.04.2" ]
then
  echo "Your ubuntu version is $relesenum"
  echo "Install the libgl1-mesa-dev-lts-utopic package to solve the dependency issues for the ROS installation specifically on $relesenum"
  sudo apt-get install -y libgl1-mesa-dev-lts-utopic
else
  echo "Your ubuntu version is $relesenum"
fi

echo "Add the ROS repository"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
fi

echo "Download the ROS keys"
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update

echo "Installing ROS $ROS_DISTRO"

# Support for Python 3 in Noetic
if [ "$ROS_DISTRO" = "noetic" ]
then
  sudo apt install -y \
  python3-rosdep \
  python3-rosinstall \
  python3-bloom \
  python3-rosclean \
  python3-wstool \
  python3-pip \
  python3-catkin-lint \
  python3-catkin-tools \
  python3-rosinstall \
  ros-$ROS_DISTRO-desktop-full
elif [ "$ROS_DISTRO" = "humble" ]
then
  sudo apt install -y \
  python3-rosdep \
  python3-rosinstall \
  python3-bloom \
  python3-rosclean \
  python3-wstool \
  python3-pip \
  python3-colcon-common-extensions \
  ros-humble-ament-cmake \
  python3-rosinstall \
  ros-$ROS_DISTRO-desktop-full
else
  sudo apt install -y \
  python-rosdep \
  python-rosinstall \
  python-bloom \
  python-rosclean \
  python-wstool \
  python-pip \
  python-catkin-lint \
  python-catkin-tools \
  python-rosinstall \
  ros-$ROS_DISTRO-desktop-full
fi

# Only init if it has not already been done before
if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

if ! command -v roscore &> /dev/null
then
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
fi

echo "Done installing ROS"

exit 0
