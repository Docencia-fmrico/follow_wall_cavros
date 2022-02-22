[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6883687&assignment_repo_type=AssignmentRepo)

[![GitHub Action
Status](https://github.com/Docencia-fmrico/follow_wall_cavros/workflows/main/badge.svg)](https://github.com/Docencia-fmrico/follow_wall_cavros)

# Follow Wall
## Introduction
In this project we will develop a follow wall behavior using tiago and kobuki robots.

To do it, we will use the laser, and the motors for velocities.

The concept behind is simple, first we get the nearest wall to make an approach, then we follow the wall side at the right, correcting the trayectory per cycle.
The code is organized in 2 different nodes: LaserNode , that covers perception , and LifeCyle which uses LaserNode's information to perform different movements depending on the robots position respect to the nearest wall.

![esquema](https://user-images.githubusercontent.com/79047431/155038093-0bd58565-921d-4eb3-bd65-f1e484ed2f17.png)

## Laser
LaserNode's job is to capture the laser perception of the robot and choose its relevant information that later will be published in /laser_info topic.

These relevant information consists in the minimum distance to a wall , its angle respect to the robot and a particular reading that will help the movement node to detect if the robot is aproaching to an open door.

Depending on the robot we are using , we will have to make some changes about the laser's interpretation , due to the difference in the capture of this sensor.
In Tiago robot we have angle 0 in front of the robot (first circle) but Kobuki's laser puts angle 0 at the back of the robot (second circle). For the first robot the node that uses this information works with its innitial angles , but with Kobuki we changed the angle representation so that it starts at 0 - front of the robot - and clockwise finishes at 360 - also at the front (third circle).


![grados](https://user-images.githubusercontent.com/79047431/155038665-ada412eb-a856-4bc9-b6d6-3057aec9a574.png)

## Motors
The motors are controlled inside LifeCycle's implementation. This node receives information via /laser_info topic and makes decissions about the robot's behaviour depending on these perceptions. Also , as LifeCycle is a lifecycle node , it has to be configured and activated to enable the robot's execution , and can also be deactivated , unconfigured etc.

In this node the robot's behavior is separated into two states : aproaching the nearest wall and following it , with all the problematics that each state overrides. 

## Conclusions
In conclusion, Tiago and Kobuki are able to follow a wall using simple reactive behaviour using simple sensors and actuators.

## Video
Tiago's simulation video:

[Video_follow_wall](https://urjc-my.sharepoint.com/:v:/g/personal/v_delatorre_2019_alumnos_urjc_es/EfGfeaxClGBBiwwTuLXH_w0BTLgBT-TUQoRV9uln_RQ5NA?e=EVxZuO)

Kobuki's real robot test:



## Contributors
* Rubén Montilla Fernández
* Blanca Soria Rubio
* Victor de la Torre Rosa
* Cristian Sánchez Rodríguez 
