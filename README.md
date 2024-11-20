# Cognitive Robotics Project Repository

This repository contains the simulation environments, controllers, and planning algorithms developed for a robotics project using [Webots](https://cyberbotics.com/).
The project focuses on exploring robotic task planning, object manipulation, and dynamic exploration in simulated environments while solving the Pick and Place problem. 

## Directory Structure

- **`colored cubes/`**: 
  - Contains all files necessary for executing simulations involving **colored cubes**. 
  - Includes the planner, Webots controller code, and room configuration files tailored for scenarios requiring robots to identify and interact with cubes of different colors.

- **`controllers/`**: 
  - Houses the Webots controller files specific to each part of the project. Each subfolder contains code and configurations necessary to control and test the robot's behavior:

    - **supervisor_controller_colored_cubes/**:
      - Code and configurations for handling colored cubes.

    - **supervisor_controller_dynamic_planning/**:
      - Code and configurations for handling dynamic planning

    - **supervisor_controller_identical_cubes/**:
      - Code and configurations for handling identical cubes.
  
- **`dynamic planning/`**: 
  - Contains all files related to the **dynamic planning** aspect of the project. This includes:
  - Includes the planner, Webots controller code, and room configuration files tailored for scenarios requiring robots to plan dynamically and explore the room efficiently.
    
- **`identical cubes/`**: 
  - Contains files for running simulations focused on handling **identical cubes**. This includes:
  - Includes the planner, Webots controller code, and room configuration files tailored for scenarios requiring robots to identify and interact with identical cubes.

- **`protos/`**: 
  - Includes **custom Webots prototype files**. These define the physical and behavioral properties of objects (e.g., cubes).

- **`worlds/`**: 
  - Contains the **Webots world files**, which define the layout of the simulation environments.
