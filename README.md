# MPC-with-obstacle-avoidance
## Description

The MPC (Model Predictive Control) implementation uses the Casadi Python Package for numerical optimization and {numpy + matplotlib} for visualization. 
A holonomic mobile robot (Mecanum wheeled omnnidirectional mobile robot (MWOMR)) is used as a system for the implementation. 
Also, it is provided by an obstacle avoidance by adding inequality constraints according to the obstacle's parameters.
The repo is aided by animated examples for better visualization.

Casadi: https://web.casadi.org/


## Content
1. `mpc_code.py` &rightarrow; The main Python script for MPC
2. `simulation_code.py` &rightarrow; A helper file implementing the visualization used in MPC code

## Requirements
Before running the codes, make sure that you have Python3.5+ installed on your computer alongside the following packages:
- [CasADi](https://web.casadi.org/get/)
- [NumPy](https://numpy.org/install/)
- [Matplotlib](https://matplotlib.org/users/installing.html)


