# Applications and Videos
With **CrazyChoir**, you can easily run your experiments on teams of cooperating Crazyflie. 
We hope these videos help you understand and utilize the package effectively. If you have any further questions or need assistance, please don't hesitate to reach out.

Now, let's dive into these implementations and explore the boundless possibilities! 

## Trajectory Tracking & Pickup-and-Delivery

[![Trajectory Tracking & Pickup-and-Delivery](https://img.youtube.com/vi/wX2r55WdZ9g/0.jpg)](https://www.youtube.com/watch?v=wX2r55WdZ9g)

Watch this [video](https://www.youtube.com/watch?v=wX2r55WdZ9g) to see a Crazyflie tracking a hand-written trajectory drawn on our GUI. 

Try this example on your own! Follow the installation procedure [here](/installation) and run

    source install/setup.bash
    ros2 launch crazychoir_examples tracking_webots.launch.py

Instead, if you are ready to fly, try this

    ros2 launch crazychoir_examples tracking_vicon.launch.py

Then, look at multiple Crazyflies that solves a Pickup-and-delivery vehicle routing problem via distributed optimization leveraging CrazyChoir and its decentralized framework. 


## Bearing-based Formation Control

[![Multi-Robot Mapping Example](https://img.youtube.com/vi/mJ1HOquR-vE/0.jpg)](https://www.youtube.com/watch?v=mJ1HOquR-vE)

In this [video](https://www.youtube.com/watch?v=mJ1HOquR-vE), you'll witness an example of formation control using CrazyChoir. Multiple nano-quadrotors cooperate together to keep the formation tight, demonstrating how the package facilitates decentralized tasks.

If you want to run this experiment type in your terminal this line

    ros2 launch crazychoir_examples formation_webots.launch.py

## Cooperative Robotic Surveillance, ICRA 2023

[![A Distributed Online Optimization Strategy for Cooperative Robotic Surveillance](https://img.youtube.com/vi/5bFFdURhTYs/0.jpg)](https://www.youtube.com/watch?v=5bFFdURhTYs)

This [video](https://www.youtube.com/watch?v=5bFFdURhTYs) shows three nano quadrotors simulate a basketball match against three virtual, attacking players. The defensive strategy, which handles the online changes in the attacking team behavior, is performed according to the proposed distributed optimization scheme. The whole experimet was handled by CrazyChoir!
If you are interested, take a look at the [accompaining paper](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=10160700).


