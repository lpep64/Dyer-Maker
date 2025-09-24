# Prompt for an MVP Digital Twin of a Resilient Manufacturing Testbed

### Role and Goal
You are an expert Robotics and AI Engineer. Your task is to act as a project manager and technical lead to design a Minimum Viable Product (MVP) for a digital twin of a manufacturing testbed. This MVP will serve as the foundational environment for future research on Dynamic Job Shop Scheduling (DJSS), resiliency, and decentralized systems.

### Project Context
The project is to develop a mini factory testbed that simulates Amazon-style warehouse operations. The physical testbed uses Niryo Ned robots and various conveyors. [cite_start]The core problem is to solve complex DJSS problems in real-time, which includes handling dynamic challenges like unpredictable orders and equipment failures[cite: 264, 279, 281]. [cite_start]The final system will include a physical testbed and a digital twin (a high-fidelity simulation) to test scenarios that are too risky or expensive to run on the physical system[cite: 265, 362].

[cite_start]The system architecture is based on ROS2, with individual hardware components (robots, conveyors, sensors) represented as ROS2 nodes[cite: 36, 37, 84, 85, 86]. The project has already completed the initial setup to access the robot's URDF and STL files.

### MVP Scope & Objective
The highest priority for this MVP is to create a functional, modular digital twin environment in ROS2 and Gazebo that acts as an advanced model of the physical testbed.

The specific, first-use case for this MVP is the following:
1.  **Identify Part:** A vision sensor node (`vision_node`) detects a part on a conveyor and publishes its color.
2.  **Pick & Place:** A single Niryo Ned robot (R1) picks up the colored part from one conveyor.
3.  **Transfer:** The robot places the part onto a second conveyor.

### Technical Requirements
The final MVP solution must adhere to the following technical constraints:
* **Simulation Platform:** ROS2 and Gazebo.
* **Modularity:** The solution must be designed as a modular environment. [cite_start]Each component (robots, conveyors, sensors) should be a separate, reusable asset that can be easily rearranged and combined to replicate different factory layouts[cite: 365, 307].
* [cite_start]**Architecture:** The solution must be based on a ROS2 node architecture, as described in the project's documentation[cite: 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57].
* **Assets:** The solution must use the provided Niryo Ned robot URDF and STL files as the foundation for the robot models.

### Deliverables
The final response should be a comprehensive MVP Proposal formatted as a single large markdown file. It must include the following sections:

1.  **Project Summary:** A brief overview of the MVP's purpose and its role in the larger project.
2.  **MVP Scope & Use Case:** A detailed description of the MVP's functionality, including the specific pick-and-place scenario.
3.  **System Architecture Design:** A clear, step-by-step breakdown of the proposed ROS2/Gazebo architecture. This should include:
    * A list of the required ROS2 nodes (`vision_node`, `sensor_node`, `pick_place_node`, etc.) and their specific responsibilities.
    * A description of the ROS2 topics and messages used for communication between nodes (`object_ready`, `object_color`, `pick_place_command`).
4.  **Implementation Plan (Step-by-Step Guide):** A detailed, numbered list of the technical steps required to build the MVP. This should be a guide for a developer to follow, from creating the Gazebo world and URDF/SDF models to writing the ROS2 nodes and the main launch file.
5.  **Future Work:** A section outlining how this MVP can be expanded to achieve the other advanced project goals (e.g., adding more robots, implementing dispatching algorithms, simulating failures, or exploring decentralized control).

### Reasoning Instructions
Let's think step by step to ensure a robust and logical solution. First, define the core components and their interactions in the Gazebo simulation environment. Then, map these to the required ROS2 nodes and topics. Finally, lay out a clear, sequential plan for implementation, starting with the simplest elements and building up to the complete MVP.

logic testbed original: 
# Gazebo Logistic Testbed

## Step 1

Follow the first 4 steps from the following github repo: `https://github.com/NiryoRobotics/ned-ros2-driver` to create the ros2_driver_ws. Upon completion of this step you should be able to access the URDF and STL files of the robots.

## Software:

The testbed has Ubuntu 24.04, ROS2 Jazzy and Gazebo 8.9.0. Gazebo 8.9 is old version and is no longer supported by the gazebo community. So you don’t have to use this. You can go for the latest version. 

## Snapshot of the 4 steps that you are required to do from the above git repository.

<img width="869" height="790" alt="Screenshot from 2025-09-16 18-02-31" src="https://github.com/romeshprasad/gazebo_logistic_testbed/blob/main/image.png" />

Once you have complete the 4 steps, you should be able to navigate to the following folder and see if the files are present.

As you can see the mesh and urdf folders are inside the directory ~/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_description$

```bash
(venv) labfab@labfab-Alienware-Area-51-R5:~/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_description$ ls
CMakeLists.txt  launch  meshes  package.xml  rviz  urdf
```

All the urdf files are xacro. Inside my code I convert them into urdf. So no need to worry about the conversion

```bash
(venv) labfab@labfab-Alienware-Area-51-R5:~/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_description/urdf/ned2$ ls
niryo_ned2_camera.urdf.xacro  niryo_ned2_gripper1_n_camera.urdf.xacro  niryo_ned2_param.urdf.xacro  niryo_ned2.urdf.xacro  without_mesh_niryo_ned2.urdf.xacro
```

This will give you access to all the stl files.

```bash
venv) labfab@labfab-Alienware-Area-51-R5:~/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_description/meshes/ned2/stl$ ls
arm_link.stl  base_link.stl  elbow_link.stl  forearm_link.stl  hand_link.stl  shoulder_link.stl  wrist_link.stl
```

## Step 2

Make sure to add the following lines to your bash script.

```bash
nano ~/.bashrc

# Add the below files at the bottom of the bash script
source ~/ros2_drivers_ws/venv/bin/activate
source /opt/ros/jazzy/setup.bash
source ~/ros2_drivers_ws/install/setup.bash
 
# Close and open the terminal

```

## Step 3

Add the below line to the environment. This is required to let the gazebo know the directory for the files

```bash
export GZ_SIM_RESOURCE_PATH=/home/labfab/ros2_drivers_ws/install/niryo_ned_description/share:$GZ_SIM_RESOURCE_PATH
```

## Step 4

Run the following script 

```bash
ros2 launch niryo_gazebo_multi_launch.py # multiple robot spawn
ros2 launch niryo_gazebo_single_launch.py #single robot spawn
```
