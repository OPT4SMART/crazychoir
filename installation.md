# Installation
In this page, we provide the installation procedure for the **CrazyChoir** package

## Requirements and Installation

**CrazyChoir** currently requires ROS 2 Foxy to be installed on your system.
Please refer to the [ROS 2 website](https://index.ros.org/doc/ros2/) for a comprehensive
tutorial on how to install ROS 2. We suggest to perform the *Desktop Install* of ROS 2,
which provides useful tools such as RVIZ.

If you do not have a ROS 2 workspace run on a terminal:

	mkdir -p ~/dev_ws/src
	cd ~/dev_ws/src

To install the toolbox, navigate inside the `src` directory and run:
```
git clone --recursive https://github.com/OPT4SMART/crazychoir.git .
```

Then, simply build the workspace:
```
cd ~/dev_ws
colcon build --symlink-install
```

### Installation of required Python packages

**CrazyChoir** requires a set of Python packages that can be installed by running:
```  
cd ~/dev_ws/src/ChoiRbot
pip3 install -r requirements.txt
```

If you are interested in running distributed optimization algorithms, you also need
the [DISROPT package](https://github.com/OPT4SMART/disropt).
You can install it by running:
```
pip3 install -r requirements_disropt.txt
pip3 install --no-deps disropt
```
You could also install disropt by directly running ``pip install disropt``. However,
this would automatically install additional packages (such as mpi4py) that are
not required by **CrazyChoir**.
