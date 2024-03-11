# Distributed Real Time Control for multiple mobile robots in an Adversarial environment
_currently a work in progress_
the practical aspect

# The aim of this project
Our goal is to design and implement a system that consists of multiple mobile robots in a capture the flag game. We aim to create an LP-based multi-robot path planning algorithm that aims to defend a "defence zone" (read: flag)




# Progress 
Here is a working demonstration of our algorithm as of yet:
<video src="https://github.com/synonymous01/multiagent_control/assets/40025239/7b3aa2ab-4ce5-4d47-b2b3-5522906577e5" />

For the sake of our research, we consider the collision of a defender robot with the attacker to be considered as capturing. You can see that our defenders not only create walls to inhibit attacker's access to the defence zone, they eventually capture it once it tries to make a run for it.

# My contributions
Of course, I am working in a team for this project in the iMaSS lab so it's important to mention what my contributions to this project are.

Firstly, I contributed by doing a summer project on formation control for robotic swarms. This included initially creating MATLAB simulations, then using the Robomaster SDK to run them on our robot platform: the Robomaster EP Core.
You can explore my work in this [link](https://github.com/synonymous01/multiagent_formation_control).

Then, before moving on to starting this project, we needed to interface our robot platform with ROS so that sensor integration was possible. There were challenges linked to this which you can read more on [here](https://github.com/synonymous01/robomaster_interface)

Now that our platform is successfully integrated, we are now devising a distributed control algorithm and simulating it in MATLAB before we move on to implementing it on ROS.

More updates on this project to come soon.
