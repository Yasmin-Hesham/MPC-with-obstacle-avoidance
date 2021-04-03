import numpy as np 
from numpy import sin, cos, pi
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.widgets import TextBox
from time import time
import casadi as ca
from mpc_code import *


def animate_mpc(mpc_update, update_initial_variables, get_target, p, obstacles_sim, save=False):
    global X0, U0, Xt
    def create_triangle(state=[0,0,0], h=1, w=0.5, update=False):
        x, y, th = state
        triangle = np.array([
            [h,  0  ],
            [0,  w/2],
            [0, -w/2],
            [h,  0  ]
        ]).T
        rotation_matrix = np.array([
            [cos(th), -sin(th)],
            [sin(th),  cos(th)]
        ])

        coords = np.array([[x, y]]) + (rotation_matrix @ triangle).T
        if update == True:
            return coords
        else:
            return coords[:n_states, :]

    def submit(expression):
        global Xt
        if expression[0] == '[':
            Xt = eval('ca.DM(' + expression + ')')
    
    def init():
        return path, horizon, current_state, target_state,

    def animate(i):
        global X0, U0

        # get variables
        update_initial_variables(X0[n_states:2*n_states], X0, U0)
        x = np.array(X0[0, 0]).flatten()[0]
        y = np.array(X0[1, 0]).flatten()[0]
        theta = np.array(X0[2, 0]).flatten()[0]
        target = Xt

        # update limits
        # all_x = [np.array(Xt[0]).flatten()] + list(x_new) + list(x_new_path)
        # all_y = [np.array(Xt[1])[0]] + list(y_new) + list(y_new_path)
        x_target = np.array(Xt[0, 0]).flatten()[0]
        y_target = np.array(Xt[1, 0]).flatten()[0]
        min_lim = min(min(x, x_target, y, y_target) * 0.9, ax.get_xlim()[0])
        max_lim = max(max(x, x_target, y, y_target) * 1.1, ax.get_xlim()[1])
        ax.set_xlim(
            left=min_lim,
            right=max_lim,
            auto=True,
            emit=True
        )
        ax.set_ylim(
            bottom=min_lim,
            top=max_lim,
            auto=True,
            emit=True
        )

        # do mpc
        X0, U0 = mpc_update(X0, U0, target)

        # update path
        x_new_path = np.hstack((path.get_xdata(), x))
        y_new_path = np.hstack((path.get_ydata(), y))
        path.set_data(x_new_path, y_new_path)

        # update horizon
        x_new = np.array(X0[n_states::n_states])
        y_new = np.array(X0[n_states+1::n_states])
        horizon.set_data(x_new, y_new)

        # update current_state
        current_state.set_xy(create_triangle([x, y, theta], update=True))
        current_state_patch.set_center((x, y))

        # update target_state
        target_state.set_xy(create_triangle(np.array(target).flatten(), update=True))

        c0 = obstacle_patches[0]
        c1 = obstacle_patches[1]
        c2 = obstacle_patches[2]


        return path, horizon, current_state, target_state, current_state_patch, c0, c1, c2, 

    ''' main '''
    X0 = ca.repmat(p[:n_states], 1, N+1)         # initial state full
    U0 = ca.DM.zeros(n_controls, N)              # initial control
    X0 = ca.reshape(X0, -1, 1)
    U0 = ca.reshape(U0, -1, 1)
    Xt = get_target()

    p = np.array(p).flatten()


    # create figure and axes
    fig, ax = plt.subplots()
    ax.set_aspect(1)
     
    # prepare animation objects:
    #   path
    path, = ax.plot([], [], 'k', linewidth=2)
    #   horizon
    horizon, = ax.plot([], [], 'x-g', alpha=0.5)
    #   current_state
    current_triangle = create_triangle(p[:3])
    current_state = ax.fill(current_triangle[:, 0], current_triangle[:, 1], color='r')
    current_state = current_state[0]
    current_state_circle = plt.Circle((p[0], p[1]), rob_radius, color='g', fill=False)
    current_state_patch = ax.add_patch(current_state_circle)
    #   target_state
    target_triangle = create_triangle(p[3:])
    target_state = ax.fill(target_triangle[:, 0], target_triangle[:, 1], color='b')
    target_state = target_state[0]
    #   obstacles
    obstacle_patches = []
    for i in range(len(obstacles_sim)):
        obstacles_circle = plt.Circle((obstacles_sim[i,0], obstacles_sim[i,1]), 
                                obstacles_sim[i,2], fill=False) 
        ax.set_aspect(1)
        ax.add_patch(obstacles_circle) 
        obstacle_patches.append(ax.add_patch(obstacles_circle))

    txt_ax = fig.add_axes([0.85, 0.45, 0.1, 0.05])
    text_box = TextBox(txt_ax, "")
    text_box.on_submit(submit)


    # create animation
    sim = animation.FuncAnimation(
        fig=fig,
        func=animate,
        init_func=init,
        interval=step_horizon*100,
        blit=True
    )
    plt.show()

    # if save == True:
    #     sim.save('./animation' + str(time()) +'.gif', writer='ffmpeg', fps=30)

    return

