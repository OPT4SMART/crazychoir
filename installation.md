# Installation
On this page, we provide the installation procedure for the **CrazyChoir** package

## Docker Installation 🐋
If you want to install **CrazyChoir** using our docker image, follow the following commands.
First, open a terminal and clone the repo in `~/crazychoir_ws/src`:

```bash
mkdir -p ~/crazychoir_ws/src
cd ~/crazychoir_ws/src
git clone --recursive https://github.com/OPT4SMART/crazychoir.git .
git submodule update --remote --merge ChoiRbot/
```
Now, follow the instructions written [here](https://github.com/OPT4SMART/crazychoir/blob/master/docker).

Otherwise, if you want to proceed with the classical installation, please refer to the next chapter.

## Requirements and Installation

**CrazyChoir** currently requires ROS 2 Foxy to be installed on your system.
Please refer to the [ROS 2 website](https://index.ros.org/doc/ros2/) for a comprehensive
tutorial on how to install ROS 2. We suggest to perform the *Desktop Install* of ROS 2,
which provides useful tools such as RVIZ.

**CrazyChoir** has been tested on Webots R2022a. It is possible to download it at [this link](https://github.com/cyberbotics/webots/releases/tag/R2022a).

The following packages are also required to run Webots simulations:

```bash
sudo apt install ros-foxy-vision-msgs
sudo apt install ros-foxy-hardware-interface
sudo apt install ros-foxy-controller-manager
```

If you do not have a ROS 2 workspace run on a terminal:

```bash
mkdir -p ~/crazychoir_ws/src
cd ~/crazychoir_ws/src
```

To install the toolbox, navigate inside the `src` directory and run:

```bash
git clone --recursive https://github.com/OPT4SMART/crazychoir.git .
git submodule update --remote --merge ChoiRbot/
```

Then, install the Vicon required libraries

```bash
cd ros2-vicon-receiver
./install_libs.sh
```

Then, simply build the workspace:
```bash
cd ~/crazychoir_ws
colcon build --symlink-install
```

### Installation of required Python packages

**CrazyChoir** requires a set of Python packages that can be installed by running:
```  bash
cd ~/crazychoir_ws/src/
pip3 install -r requirements.txt
```

**CrazyChoir** requires a set of API from the [DISROPT package](https://github.com/OPT4SMART/disropt).
You can install them by running:
```bash
pip3 install --no-deps disropt
```
You could also install disropt by directly running ``pip install disropt``. However,
this would automatically install additional packages (such as mpi4py) that are
not required by **CrazyChoir**.

### Setup of Crazyflie Firmware Bindings
In order to use some functionalities of the package, it is necessary to build the Crazyflie firmware and create Python bindings of firmware functions.
We follow the guide at [this link](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/building-and-flashing/build.md).

In your favorite directory, run
```bash
sudo apt-get install make gcc-arm-none-eabi
sudo apt install swig
```
Then, clone the Crazyflie firmware repository
```bash
git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
cd crazyflie-firmware
git submodule init
git submodule update
```

Compile and create the bindings
```bash
make cf2_defconfig
make -j 12
make bindings_python
```

Now, you should have a ```build``` directory containing the file ```cffirmware.py```.
Copy the path of ```cffirmware.py``` into the file ```~/dev_ws/src/crazychoir/crazychoir/utils/__init__.py``` at Line 3.
