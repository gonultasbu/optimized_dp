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
grid_min = np.array([-20.0, -20.0, -4.5, -4.5])
grid_max = np.array([20.0, 20.0, 4.5, 4.5])
dims = 4
N = np.array([80, 80, 80, 80])
pd = []
g = Grid(grid_min, grid_max, dims, N, pd)

# STEP 2: Generate initial values for grid using shape functions
radius = 2.0
ignore_dims = [2, 3]
Initial_value_f = CylinderShape(g, ignore_dims, np.zeros(dims), radius)
# STEP 3: Time length for computations
Lookback_length = 0.5
t_step = 0.05

small_number = 1e-5
tau = np.arange(start=0, stop=Lookback_length + small_number, step=t_step)

# STEP 4: System dynamics for computation
sys = PursuitEvasion(uMode="max", dMode="min")

# STEP 5: Initialize plotting option
po = PlotOptions(
    do_plot=True,
    plot_type="set",
    plotDims=[0, 1, 2],
    slicesCut=[],
    colorscale="Bluered",
    save_fig=False,
    filename="plots/TwinPMPE_0_sublevel_set",
    interactive_html=True,
)

# STEP 6: Call HJSolver function
compMethod = {"TargetSetMode": "minVWithV0"}
result = HJSolver(sys, g, Initial_value_f, tau, compMethod, po, saveAllTimeSteps=True)
