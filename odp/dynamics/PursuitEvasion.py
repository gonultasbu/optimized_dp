# File: odp/dynamics/PursuitEvasion.py

import heterocl as hcl
import numpy as np


class PursuitEvasion:
    def __init__(
        self,
        x=[0, 0, 0, 0],
        uMin=[-2.45, -2.45],
        uMax=[2.45, 2.45],
        dMin=[-9.81, -9.81],
        dMax=[9.81, 9.81],
        uMode="min",
        dMode="max",
    ):
        self.x = x
        self.uMax = uMax
        self.uMin = uMin
        self.dMax = dMax
        self.dMin = dMin
        assert uMode in ["min", "max"]
        self.uMode = uMode
        if uMode == "min":
            assert dMode == "max"
        else:
            assert dMode == "min"
        self.dMode = dMode

    def dynamics(self, t, state, uOpt, dOpt):
        x_dot = hcl.scalar(0, "x_r_dot")
        y_dot = hcl.scalar(0, "y_r_dot")
        vx_dot = hcl.scalar(0, "vx_r_dot")
        vy_dot = hcl.scalar(0, "vy_r_dot")
        # Relative dynamics
        x_dot[0] = state[2]
        y_dot[0] = state[3]
        vx_dot[0] = dOpt[0] - uOpt[0]
        vy_dot[0] = dOpt[1] - uOpt[1]

        return (
            x_dot[0],
            y_dot[0],
            vx_dot[0],
            vy_dot[0],
        )

    def opt_ctrl(self, t, state, spat_deriv):
        opt_ax = hcl.scalar(self.uMin[0], "opt_ax")
        opt_ay = hcl.scalar(self.uMin[1], "opt_ay")
        in3 = hcl.scalar(0, "in3")
        in4 = hcl.scalar(0, "in4")
        with hcl.if_(spat_deriv[2] > 0):
            with hcl.if_(self.uMode == "min"):
                opt_ax[0] = self.uMax[0]
        with hcl.elif_(spat_deriv[2] < 0):
            with hcl.if_(self.uMode == "max"):
                opt_ax[0] = self.uMax[0]

        with hcl.if_(spat_deriv[3] > 0):
            with hcl.if_(self.uMode == "min"):
                opt_ay[0] = self.uMax[1]
        with hcl.elif_(spat_deriv[3] < 0):
            with hcl.if_(self.uMode == "max"):
                opt_ay[0] = self.uMax[1]

        return (opt_ax[0], opt_ay[0], in3[0], in4[0])

    def opt_dstb(self, t, state, spat_deriv):
        opt_dx = hcl.scalar(self.dMax[0], "opt_dx")
        opt_dy = hcl.scalar(self.dMax[1], "opt_dy")
        d3 = hcl.scalar(0, "d3")
        d4 = hcl.scalar(0, "d4")

        with hcl.if_(spat_deriv[2] > 0):
            with hcl.if_(self.dMode == "min"):
                opt_dx[0] = self.dMin[0]
        with hcl.elif_(spat_deriv[2] < 0):
            with hcl.if_(self.dMode == "max"):
                opt_dx[0] = self.dMin[0]

        with hcl.if_(spat_deriv[3] > 0):
            with hcl.if_(self.dMode == "min"):
                opt_dy[0] = self.dMin[1]
        with hcl.elif_(spat_deriv[3] < 0):
            with hcl.if_(self.dMode == "max"):
                opt_dy[0] = self.dMin[1]

        return (opt_dx[0], opt_dy[0], d3[0], d4[0])

    # Optional: Python implementation for testing or post-processing
    def optCtrl_inPython(self, spat_deriv):
        opt_ax = self.uMin[0]
        opt_ay = self.uMin[1]

        if spat_deriv[2] > 0:
            if self.uMode == "min":
                opt_ax = self.uMax[0]
        elif spat_deriv[2] < 0:
            if self.uMode == "max":
                opt_ax = self.uMax[0]
        elif np.isclose(spat_deriv[2], 0):
            opt_ax = 0
        if spat_deriv[3] > 0:
            if self.uMode == "min":
                opt_ay = self.uMax[1]
        elif spat_deriv[3] < 0:
            if self.uMode == "max":
                opt_ay = self.uMax[1]
        elif np.isclose(spat_deriv[3], 0):
            opt_ay = 0

        return (opt_ax, opt_ay)

    def optDstb_inPython(self, spat_deriv):
        opt_dx = self.dMax[0]
        opt_dy = self.dMax[1]

        if spat_deriv[2] > 0:
            if self.dMode == "min":
                opt_dx = self.dMin[0]
        elif spat_deriv[2] < 0:
            if self.dMode == "max":
                opt_dx = self.dMin[0]

        if spat_deriv[3] > 0:
            if self.dMode == "min":
                opt_dy = self.dMin[1]
        elif spat_deriv[3] < 0:
            if self.dMode == "max":
                opt_dy = self.dMin[1]

        return (opt_dx, opt_dy)
