from time import time
import matplotlib.pyplot as plt
import casadi as ca
from simulation_code import animate_mpc    
from mpc_code import *
import numpy as np

# Initialize Current state
current_x = 0
current_y = 0
yaw = 0
state = ca.DM([current_x, current_y, yaw])  
# Initialize Target state
target_x = 0
target_y = 0
target_theta = 0
state_target = ca.DM([target_x, target_y, target_theta])

p = ca.vertcat(state, state_target)

obstacles_sim = DM2Arr(obstacles)

def mpc_update(X0, U0, Xt, args=args):
    solution = solver(
            x0=ca.vertcat(X0, U0),
            p=ca.vertcat(X0[:n_states], Xt),
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg']
        )

    X0n = solution['x'][:n_states*(N+1)]
    U0n = solution['x'][n_states*(N+1):]

    return X0n, U0n

def update_initial_variables(xi, X0n, U0n):
    # optimization variable current state
    # 1. initial state 
    # 2. initial next states are extracted from previous optimized state
    # Same procedure for all next steps
    # For the last state, copy the previous cell in the state vector "X0".
    X0 = X0n
    X0[:n_states] = xi  
    X0[n_states:-n_states] = X0n[2*n_states:]
    X0[-n_states:] = X0n[-2*n_states:-n_states]

    # Same procedure as the states.
    U0 = U0n
    U0[:-n_controls] = U0n[n_controls:]
    U0[-n_controls:] = U0n[-2*n_controls:-n_controls]

    return X0, U0

def get_target():
    global state_target
    return state_target

def set_target(x,y,theta):
    global state_target
    state_target = ca.DM([x, y, theta])

# animate_mpc
animate_mpc(
    mpc_update, 
    update_initial_variables, 
    get_target, 
    p,
    obstacles_sim
)


 
