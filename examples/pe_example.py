import numpy as np

# Utility functions to initialize the problem
from odp.Grid import Grid
from odp.Shapes import *

# Specify the  file that includes dynamic systems
from odp.dynamics import DubinsCapture, Plane2D, Plane1D, DubinsCar4D, PursuitEvasion

# Plot options
from odp.Plots import PlotOptions
from odp.Plots import plot_isosurface, plot_valuefunction

# Solver core
from odp.solver import HJSolver, computeSpatDerivArray

import math
import os

""" USER INTERFACES
- Define grid
- Generate initial values for grid using shape functions
- Time length for computations
- System dynamics for computation
- Initialize plotting option
- Call HJSolver function

Note: If run on the server, please save the result and use the plot function on your local machine
"""

if os.path.exists("plots") == False:
    os.mkdir("plots")

##################################################### TWINPMPE EXAMPLE #####################################################
# STEP 1: Define grid
grid_min = np.array([-10.0, -10.0, -5.0, -5.0])
grid_max = np.array([10.0, 10.0, 5.0, 5.0])
dims = 4
N = np.array([25, 25, 25, 25])
pd = []
g = Grid(grid_min, grid_max, dims, N, pd)

# STEP 2: Generate initial values for grid using shape functions
radius = 2.0
ignore_dims = [2, 3]
Initial_value_f = CylinderShape(g, ignore_dims, np.zeros(dims), radius)
# STEP 3: Time length for computations
Lookback_length = 10.0
t_step = 0.1

small_number = 1e-5
tau = np.arange(start=0, stop=Lookback_length + small_number, step=t_step)

# STEP 4: System dynamics for computation
sys = PursuitEvasion(uMode="min", dMode="max")

# STEP 5: Initialize plotting option
po = PlotOptions(
    do_plot=False,
    plot_type="set",
    plotDims=[0, 1, 2],
    slicesCut=[10],
    colorscale="Bluered",
    save_fig=True,
    filename="plots/TwinPMPE_0_sublevel_set",
    interactive_html=True,
)

# STEP 6: Call HJSolver function
compMethod = {"TargetSetMode": "minVWithV0"}
result = HJSolver(sys, g, Initial_value_f, tau, compMethod, po, saveAllTimeSteps=True)


# plot_valuefunction(g, result, po)

last_time_step_result = result[..., 0]

# Compute spatial derivatives at every state
x_derivative = computeSpatDerivArray(
    g, last_time_step_result, deriv_dim=1, accuracy="medium"
)
y_derivative = computeSpatDerivArray(
    g, last_time_step_result, deriv_dim=2, accuracy="medium"
)
vx_derivative = computeSpatDerivArray(
    g, last_time_step_result, deriv_dim=3, accuracy="medium"
)
vy_derivative = computeSpatDerivArray(
    g, last_time_step_result, deriv_dim=4, accuracy="medium"
)
state = np.array([-1.0, 1.0, 0.0, 0.0])
corresponding_grid_idxs = []
for state_dim in range(state.shape[0]):
    corresponding_grid_idxs.append(
        np.argmin(np.abs(g.grid_points[state_dim] - state[state_dim]))
    )


# Let's compute optimal control at some random idices
spat_deriv_vector = (
    x_derivative[
        corresponding_grid_idxs[0],
        corresponding_grid_idxs[1],
        corresponding_grid_idxs[2],
        corresponding_grid_idxs[3],
    ],
    y_derivative[
        corresponding_grid_idxs[0],
        corresponding_grid_idxs[1],
        corresponding_grid_idxs[2],
        corresponding_grid_idxs[3],
    ],
    vx_derivative[
        corresponding_grid_idxs[0],
        corresponding_grid_idxs[1],
        corresponding_grid_idxs[2],
        corresponding_grid_idxs[3],
    ],
    vy_derivative[
        corresponding_grid_idxs[0],
        corresponding_grid_idxs[1],
        corresponding_grid_idxs[2],
        corresponding_grid_idxs[3],
    ],
)

# Compute the optimal control
opt_ax, opt_ay = sys.optCtrl_inPython(spat_deriv_vector)
print("Optimal x accel is {}\n".format(opt_ax))
print("Optimal y accel is {}\n".format(opt_ay))

opt_dx, opt_dy = sys.optDstb_inPython(spat_deriv_vector)
print("Optimal x disturbance is {}\n".format(opt_dx))
print("Optimal y disturbance is {}\n".format(opt_dy))
