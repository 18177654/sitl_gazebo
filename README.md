# Gazebo for MAVLink SITL and HITL [![Build Status](https://travis-ci.org/PX4/sitl_gazebo.svg?branch=master)](https://travis-ci.org/PX4/sitl_gazebo)

This is a flight simulator for multirotors, VTOL and fixed wing. It uses the motor model and other pieces from the RotorS simulator, but in contrast to RotorS has no dependency on ROS. This repository is in the process of being re-integrated into RotorS, which then will support ROS and MAVLink as transport options: https://github.com/ethz-asl/rotors_simulator

**If you use this simulator in academic work, please cite RotorS as per the README in the above link.**

## Install Gazebo Simulator

Follow instructions on the [official site](http://gazebosim.org/tutorials?cat=install) to install Gazebo. Mac OS and Linux users should install Gazebo 7.


## Protobuf

Install the protobuf library, which is used as interface to Gazebo.

### Ubuntu Linux

```bash
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev \
			gazebo7 libgazebo7-dev libxml2-utils python-rospkg python-jinja2
```

### Mac OS

```bash
pip install rospkg jinja2
brew install graphviz libxml2 sdformat3 eigen opencv
brew install gazebo7
```

An older version of protobuf (`< 3.0.0`) is required on Mac OS:

```bash
brew tap homebrew/versions
brew install homebrew/versions/protobuf260
```

## Build Gazebo Plugins (all operating systems)

Clone the gazebo plugins repository to your computer. IMPORTANT: If you do not clone to ~/esl-sun/sitl_gazebo, all remaining paths in these instructions will need to be adjusted.

```bash
mkdir -p ~/esl-sun
cd esl-sun
git clone --recursive https://github.com/esl-sun/sitl_gazebo.git
cd sitl_gazebo
```

Create a build folder in the top level of your repository:

```bash
mkdir build
```

Next add the location of this build directory to your gazebo plugin path, e.g. add the following lines to your .bashrc (Linux) or .bash_profile (Mac) file:

```bash
# Set path to sitl_gazebo repository
export SITL_GAZEBO_PATH=$HOME/esl-sun/sitl_gazebo
# Set the plugin path so Gazebo finds our model and sim
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${SITL_GAZEBO_PATH}/build
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SITL_GAZEBO_PATH}/models
# Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=""
```

Remember to `source` the file and/or reboot your computer.

Navigate into the build directory and invoke CMake from it:

```bash
cd ~/esl-sun/sitl_gazebo
cd build
cmake ..
```

Now build the gazebo plugins by typing:

```bash
make
```

### Build Errors

#### Could NOT find MAVLink

If you receive the following error:
```
Could NOT find MAVLink (missing:  MAVLINK_INCLUDE_DIRS) (found version "2.0")
CMake Error: The following variables are used in this project, but they are set to NOTFOUND.
Please set them or make sure they are set and tested correctly in the CMake files:
_MAVLINK_INCLUDE_DIR
```
Install the ROS MAVLink node:
```
sudo apt-get install ros-<your-ros-distro>-mavlink
```

#### velocity_covariance / twist_covariance not found

If the build fails due to a line of code referring to the velocity_covariance or twist_covariance not found, do the following:

- If you are using ROS Melodic: Change the variable to twist_covariance
- If you are using ROS Kinetic: Chnage the variables to velocity_covariance

### GStreamer Support
If you want support for the GStreamer camera plugin, make sure to install
GStreamer before running `cmake`. Eg. on Ubuntu with:
```
sudo apt-get install gstreamer1.0-* libgstreamer1.0-*
```

### Geotagging Plugin
If you want to use the geotagging plugin, make sure you have `exiftool`
installed on your system. On Ubuntu it can be installed with:
```
sudo apt-get install libimage-exiftool-perl
```

## Install

If you wish the libraries and models to be usable anywhere on your system without
specifying th paths, install as shown below.

**Note: If you are using ubuntu, it is best to see the packaging section.**

```bash
sudo make install
```

## Testing

Gazebo will now launch when typing 'gazebo' on the shell:

```bash
. /usr/share/gazebo/setup.sh
. /usr/share/mavlink_sitl_gazebo/setup.sh
gazebo worlds/iris.world
```

Please refer to the documentation of the particular flight stack how to run it against this framework, e.g. [PX4](http://dev.px4.io/simulation-gazebo.html)


## Unit Tests

For building and running test an installation of 'googletest' is needed.

On Ubuntu it can be installed with:

```bash
sudo apt-get install libgtest-dev
cd /usr/src/googletest
sudo cmake . && cd googletest
sudo make
sudo cp *.a /usr/lib
```

On macOS it needs to be installed from source:

```bash
git clone https://github.com/google/googletest
pushd googletest
mkdir build
pushd build
cmake ..
make && make install
popd
popd
```bash

When writing test it’s important to be careful which API functions of Gazebo are called. As no Gazebo server is running during the tests some functions can produce undefined behaviour (e.g. segfaults).


### catkin

With catkin the test are enabled by default.

```bash
# After setting up the catkin workspace
catkin build -j4 -l4 -DBUILD_ROS_INTERFACE=ON
cd build/mavlink_sitl_gazebo/
catkin run_tests
```

### Plain CMake

For building the tests with plain CMake, the flag `ENABLE_UNIT_TESTS` needs to be provided.

```bash
mkdir build && cd build
cmake -DENABLE_UNIT_TESTS=On ..
```

Then build and run the tests:

```bash
make && make test
```

## Packaging

### Deb

To create a debian package for ubuntu and install it to your system.

```bash
cd Build
cmake ..
make
rm *.deb
cpack -G DEB
sudo dpkg -i *.deb
```
