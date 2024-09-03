# File: odp/dynamics/PursuitEvasion.py

import heterocl as hcl


class PursuitEvasion:
    def __init__(
        self,
        x=[0, 0, 0, 0],
        uMin=[-1, -1],
        uMax=[1, 1],
        dMin=[-0.25, -0.25],
        dMax=[0.25, 0.25],
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

    def opt_ctrl(self, t, state, spat_deriv):
        opt_ax = hcl.scalar(self.uMax[1], "opt_ax")
        opt_ay = hcl.scalar(self.uMax[1], "opt_ay")
        in3 = hcl.scalar(0, "in3")
        in4 = hcl.scalar(0, "in4")
        with hcl.if_(spat_deriv[2] > 0):
            with hcl.if_(self.uMode == "min"):
                opt_ax[0] = self.uMin[0]
        with hcl.elif_(spat_deriv[2] < 0):
            with hcl.if_(self.uMode == "max"):
                opt_ax[0] = self.uMin[0]

        with hcl.if_(spat_deriv[3] > 0):
            with hcl.if_(self.uMode == "min"):
                opt_ay[0] = self.uMin[0]
        with hcl.elif_(spat_deriv[3] < 0):
            with hcl.if_(self.uMode == "max"):
                opt_ay[0] = self.uMin[0]

        return (opt_ax[0], opt_ay[0], in3[0], in4[0])

    def opt_dstb(self, t, state, spat_deriv):
        opt_dx = hcl.scalar(self.uMin[1], "opt_dx")
        opt_dy = hcl.scalar(self.uMin[1], "opt_dy")

        d3 = hcl.scalar(0, "d3")
        d4 = hcl.scalar(0, "d4")

        with hcl.if_(spat_deriv[2] > 0):
            with hcl.if_(self.dMode == "min"):
                opt_dx[0] = self.uMax[0]
        with hcl.elif_(spat_deriv[2] < 0):
            with hcl.if_(self.dMode == "max"):
                opt_dx[0] = self.uMax[0]

        with hcl.if_(spat_deriv[3] > 0):
            with hcl.if_(self.dMode == "min"):
                opt_dy[0] = self.uMax[0]
        with hcl.elif_(spat_deriv[3] < 0):
            with hcl.if_(self.dMode == "max"):
                opt_dy[0] = self.uMax[0]

        return (opt_dx[0], opt_dy[0], d3[0], d4[0])

    def dynamics(self, t, state, uOpt, dOpt):
        x_dot = hcl.scalar(0, "x_p_dot")
        y_dot = hcl.scalar(0, "y_p_dot")
        vx_dot = hcl.scalar(0, "vx_p_dot")
        vy_dot = hcl.scalar(0, "vy_p_dot")
        # Pursuer dynamics
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

    # Optional: Python implementation for testing or post-processing
    def optCtrl_inPython(self, state, spat_deriv):
        opt_ax = self.uMin[1]
        opt_ay = self.uMin[1]

        if spat_deriv[2] > 0:
            if self.uMode == "min":
                opt_ax = self.uMin[0]
        elif spat_deriv[2] < 0:
            if self.uMode == "max":
                opt_ax = self.uMin[0]

        if spat_deriv[3] > 0:
            if self.uMode == "min":
                opt_ay = self.uMin[0]
        elif spat_deriv[3] < 0:
            if self.uMode == "max":
                opt_ay = self.uMin[0]

        return (opt_ax, opt_ay)

    def optDstb_inPython(self, state, spat_deriv):
        opt_dx = self.uMin[1]
        opt_dy = self.uMin[1]

        if spat_deriv[2] > 0:
            if self.dMode == "min":
                opt_dx = self.uMin[0]
        elif spat_deriv[2] < 0:
            if self.dMode == "max":
                opt_dx = self.uMin[0]

        if spat_deriv[3] > 0:
            if self.dMode == "min":
                opt_dy = self.uMin[0]
        elif spat_deriv[3] < 0:
            if self.dMode == "max":
                opt_dy = self.uMin[0]

        return (opt_dx, opt_dy)
