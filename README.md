# CrazyChoir 
[**Website**](https://opt4smart.github.io/crazychoir/)
| [**Installation**](https://opt4smart.github.io/crazychoir/installation)


| :warning: Information for end users |
|:------------------------------------|
| Documentation pages are currently being uploaded. The latest version will be available soon. |

**CrazyChoir** is a ROS 2 toolbox allowing users to run simulations and experiments on swarms of Crazyflie nano-quadrotors.
**CrazyChoiR** implements several tools to model swarms of Crazyflie nano-quadrotors and to run distributed, complex tasks both in simulation and experiments. Examples include task assignment, formation control and trajectory tracking settings.

## Requirements and Installation
**CrazyChoir** requires ROS 2 Foxy to be installed on your system.

Some preliminary steps are required to succesfully complete the installation. We refer the reader to [this guide](https://opt4smart.github.io/crazychoir/installation).

## Examples
In order to check the installation and start to use **CrazyChoir**, you can run

```
ros2 launch crazychoir_examples formation_webots.launch.py 
```

This will start a Webots simulation with 11 Crazyflie nano-quadrotor performing a formation control task.


## Citing **CrazyChoir**
If you are you using **CrazyChoir** in research work to be published, please cite the accompanying paper

```
@ARTICLE{pichierri2023crazychoir,
  title={CrazyChoir: Flying Swarms of Crazyflie Quadrotors in ROS 2},
  author={Pichierri, Lorenzo and Testa, Andrea and Notarstefano, Giuseppe},
  journal={IEEE Robotics and Automation Letters},  
  volume={8},
  number={8},
  pages={4713-4720},
  year={2023},
  publisher={IEEE}
```


## Contributors
**CrazyChoir** is developed by
[Lorenzo Pichierri](https://www.unibo.it/sitoweb/lorenzo.pichierri),
[Andrea Testa](https://www.unibo.it/sitoweb/a.testa) and
[Giuseppe Notarstefano](https://www.unibo.it/sitoweb/giuseppe.notarstefano)
