# NIST Taskboard 1 Assembly/Disassembly
This project is an attempt to NIST Taskboard 1 Benchmark. While the benchmark itself is out of the reach for this project. The assembly/disassembly task is an amazing source of learning. This project were built with `ROS2`, `MoveIt2`, utilizing `MTC`(MoveIt Task Constructor). All the `CAD` materials were included in the assembly task. Few adjustment and missing piece were then collected and designed using `FreeCAD`.

## Preliminary
I was looking for a suitable project to attempt on assembly/disassembly and come up to this NIST Taskboard 1. At that time, I'm learning the robotics stack of `ROS2` and all of it's library. This project were meant to further hone my skills by practice. This project got me to learn `CAD`, Math behind the robotics, and the software side. This project wasn't easy for a beginner like me. Since I lacks of fundamentals to even attempt this. So I was learning by doing. 

## System Architecture & Stacks
- ROS 2 (Jazzy)
- MoveIt 2
- OS Environment Ubuntu 22
- Robot model (IRB120, IRB1200)
- Gripper model (Onrobot 2fg7, Robotiq 2f-80/2f-140)

## CAD & Custom adjustment

## Installation
You have to complete the preequisite installation of `ROS2` project so its build ready. Read the documentation [documentation] here.

Build the project
```
colcon build --packages-select project_moveit project_bringup project_description project_perception
```
Open 3 terminal on project directory
```
source install/setup.bash
```

Terminal 1: Run the robot rviz and all preequisite
```
ros2 launch project_description project irb120_rviz_taskboard.launch.py
```

Terminal 2: Run scene loader
```
ros2 run project_perception scene_loader
```

Terminal 3: Run task constructor
```
ros2 launch project_moveit task_constructor.launch.py
```

## Progress
On assembly, I had some physical issue on reach and layout. This going to need more study to solve. For example, the robot can't physically reach some place with some pose. Gripper cannot physically reach the middle of the board approaching sideways. So, task constructor cannot do it in one go (Pick->Rearrange->Pick->Assemble). This parts introduce more challenges.

| No | Name | Description | `partname_link` | Checklist |
| --- | --- | --- | --- | --- |
| **M-Series Nuts** |  |  |  |  |
| 1 | M4 Nut |  | `m4_nut_link` | [x] |
| 2 | M8 Nut |  | `m8_nut_link` | [x] |
| 3 | M12 Nut |  | `m12_nut_link` | [x] |
| 4 | M16 Nut |  | `m16_nut_link` | [ ] |
| **KET Series (Square Bars)** |  |  |  |  |
| 5 | KET4 Bar |  | `ket4_link` | [ ] |
| 6 | KET8 Bar |  | `ket8_link` | [ ] |
| 7 | KET12 Bar |  | `ket12_link` | [ ] |
| 8 | KET16 Bar |  | `ket16_link` | [ ] |
| **RGOCG Series (Round Rods)** |  |  |  |  |
| 9 | RGOCG4 Rod |  | `rgocg4_50_link` | [ ] |
| 10 | RGOCG8 Rod |  | `rgocg8_50_link` | [x] |
| 11 | RGOCG12 Rod |  | `rgocg12_50_link` | [x] |
| 12 | RGOCG16 Rod |  | `rgocg16_50_link` | [x] |
| **Gear Series** |  |  |  |  |
| 13 | Small Gear |  | `gear_small_link` | [ ] |
| 14 | Medium Gear |  | `gear_medium_link` | [ ] |
| 15 | Large Gear |  | `gear_large_link` | [ ] |
| **Connectors (Plugs & Jacks)** |  |  |  |  |
| 16 | BNC Male Connector |  | `bnc_male_link` | [ ] |
| 17 | Male Waterproof Connector |  | `mcon-310-sp_link` | [ ] |
| 18 | DSUB Male Connector |  | `dsub_male__link` | [ ] |
| 19 | USB Male Cable |  | `usb_male_cable_link` | [ ] |
| 20 | RJ45 Male Connector |  | `rj45_male__link` | [ ] |

## Known Issue & Workaround
- [ ] M8 Bolt on taskboard wasn't arranged properly
- [ ] Gripper coudn't physically reach middle of the board
- [ ] Lots of naming issue discrepancy
