[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6883687&assignment_repo_type=AssignmentRepo)

[![GitHub Action
Status](https://github.com/Docencia-fmrico/follow_wall_cavros/workflows/main/badge.svg)](https://github.com/Docencia-fmrico/follow_wall_cavros)

# Follow Wall
## Introduction
In this project we will develop a follow wall behavior using tiago robot.

To do it, we will use the laser, and the motors for velocities.

The concept behind is simple, first we get the nearest wall to make an approach, then we follow the wall side at the right, correcting the trayectory per cycle.

## Laser
LaserNode's job is to capture the laser perception of the robot and choose its relevant information that later will be published in /laser_info topic.

These relevant information consists in the minimum distance to a wall , its angle respect to the robot and a particular reading that will help the movement node to detect if the robot is aproaching to an open door.

## Motors
The motors are controlled inside MoveNode's implementation. This node receives information via /laser_info topic and makes decissions about the robot's behaviour depending on these perceptions.

In this node tiago's behavior is separated into two states : aproaching the nearest wall and following it , with all the problematics that each state overrides. 

## Conclusions
In conclusion, Tiago is able to follow a wall using simple sensor/actuators.

For extensibility reasons, we capsuled everything in a lifecycle node, where we can switch between 3 states, unconfigure, inactive and active. This last one is where the job is done.

## Videos

Video of the robot working in Gazebo:
[Video_gazebo_follow_wall](https://urjc-my.sharepoint.com/:v:/g/personal/v_delatorre_2019_alumnos_urjc_es/EfGfeaxClGBBiwwTuLXH_w0BTLgBT-TUQoRV9uln_RQ5NA?e=EVxZuO)
Videos of the real robot running:
[Video_follow_corners](https://urjc-my.sharepoint.com/:v:/g/personal/r_montillaf_2019_alumnos_urjc_es/EYs1AoZE1-xKqDASg9e0-84BJwlyLanlmBEnYzI_P4EwcA?e=doxDcn)
[Video_follow_turns](https://urjc-my.sharepoint.com/:v:/g/personal/r_montillaf_2019_alumnos_urjc_es/EXmkvGisdyxDqYQs6fM6loQB1oCNjSdUZrWV8r9qWzjlAQ?e=zkUxEo)



## Contributors
* Rubén Montilla Fernández
* Blanca Soria Rubio
* Victor de la Torre Rosa
* Cristian Sánchez Rodríguez 
