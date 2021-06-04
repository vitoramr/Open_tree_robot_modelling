# Model and simulation of an open tree robot
## CentraleSupélec - Elective course - 2020
### Interactive Robotic Systems

Professor: Mathieu Grossard
Students:
- João Victor EVANGELISTA MATOSO
- Vítor ALBUQUERQUE MARANHÃO RIBEIRO

### Introduction 
This code was developed as a final project for the "Interactive Robotic Systems" course at CentraleSupélec. The project consisted of the modelling of an open tree robot with revolute joints developed by the CEA Laboratory.

### MATLAB Code ([Matlab_model.m](https://github.com/vitoramr/Open_tree_robot_modelling/blob/main/Matlab_model.m))
The code's structure is divided into 5 sections:

1. Parameters Initialization
- Parameters for displaying the plots (can be set to false)
- Geometric parameters of the robot
- Inertial parameters of the robot

2. Resolution of the questions
Each question is divided into the same topic as described in the TDs
Summary:
- Direct geometric model (questions 4 to 7)
- Inverse geometric model (question 8)
- Inverse kinematic model (questions 9 to 11)
- Dynamic model (questions 12 to 17)
- Trajectory generation in the joint space (question 18)
- The questions 17, 19 and 20 are solved in the Simulink file also attached

3. TD's Functions
- It contains all the functions that were developed for computing the questions

4. Plotting functions
- It contains all the functions that are used to plot results

5. Auxiliary functions
- It contains all mathematical functions used to help during calculations


### SIMULINK File ([Simulink_model_and_control.slx](https://github.com/vitoramr/Open_tree_robot_modelling/blob/main/Simulink_model_and_control.slx))
The Simulink file performs the modelling and the control of the robot,
Solving the questions 17, 19 and 20 of TD2.

Before running the Simulink file, run the [init.m](https://github.com/vitoramr/Open_tree_robot_modelling/blob/main/init.m) Matlab script to initialize the model's parameters
