| :warning: Information for end users |
|:------------------------------------|
| Documentation pages are currently being uploaded. The latest version will be available soon. |

**CrazyChoir** is a ROS 2 toolbox allowing users to run simulations and experiments on swarms of Crazyflie nano-quadrotors.
**CrazyChoiR** implements several tools to model swarms of Crazyflie nano-quadrotors and to run distributed, complex tasks both in simulation and experiments. Examples include formation control, and task assignment and trajectory tracking settings.

## Requirements and Installation
**CrazyChoir** requires ROS 2 Foxy to be installed on your system.

To install the toolbox, create a ROS 2 workspace and, inside the `src` directory, run:
```
git clone https://github.com/OPT4SMART/crazychoir.git .
```

Then, from the parent directory execute:
```
colcon build --symlink-install
```


## Constributors
**ChoiRbot** is developed by
[Lorenzo Pichierri](https://www.unibo.it/lorenzo.pichierri/),
[Andrea Testa](https://www.unibo.it/sitoweb/a.testa) and
[Giuseppe Notarstefano](https://www.unibo.it/sitoweb/giuseppe.notarstefano)
