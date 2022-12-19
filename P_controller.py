import csv

import cvxpy

from prm_plannerV2 import *
from E160_state import *
from E160_robot import *
from Path import *
import math
import time
import cubic_spline_planner

# MPC and Vehicle Variables
NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 2  # horizon length - higher horizon causes program to slow - 5 is okay - 9 is slow  - 3 is best so far - 2
# Higher better overall path. Just takes forever. In this case it isnt needed. Low is performing well
# 2 did superb. 1 is terrible.
# 2 or 3. 2 is faster 3 is okay on speed. Ill go with 2

# mpc parameters
R = np.diag([0.001,
             0.001])  # input cost matrix
Rd = np.diag([0.01,
              1.0])  # input difference cost matrix
Q = np.diag([2,
             2,
             .5,
             0.5])  # state cost matrix
Qf = Q  # state final matrix

# Ending Conditions
GOAL_DIS = 1  # goal distance
STOP_SPEED = 0.5  # stop speed

# iterative parameter
MAX_ITER = 3  # Max iteration - number of optimization attempts - higher = slower - lower = faster - 3 is okay
DU_TH = 0.1  # iteration finish param
N_IND_SEARCH = 1000 # Search index number - for some reason low numbers cause the controller to go crazy
# - 1000 seems to be good.
TARGET_SPEED = 20.0  # [m/s] target speed - higher speeds causes car to differ from path - slower is better for tracking
# possibly because horizon has to be low. Controller cant plan ahead for faster speeds - 20 is good for tracking - 40 has overshoot
# 30 is good medium
DT = 0.1  # [s] time tick

# Vehicle parameters
WB = 20  # [m]
L2 = 16
r = 3

# Constraints
MAX_STEER = np.deg2rad(40.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 40.0  # maximum speed [m/s]
MIN_SPEED = -MAX_SPEED  # minimum speed [m/s]
MAX_ACCEL = 10.0  # maximum accel [m/ss]


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None

    # MPC constants


def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        # if dx != 0.0 and dy != 0.0:
        #     dangle = abs(pi_2_pi(move_direction - cyaw[i]))
        #     if dangle >= math.pi / 4.0:
        #         direction = -1.0
        #     else:
        #         direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def pi_2_pi(angle):
    while (angle > math.pi):
        angle = angle - 2.0 * math.pi

    while (angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def get_linear_model_matrix(v, phi, delta, ):
    NX = 4  # x = x, y, v, yaw
    NU = 2  # a = [accel, steer]
    DT = 0.2  # [s] time tick
    WB = 2.5  # [m]

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C


def update_state(state, a, delta):
    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + state.v * math.cos(state.yaw) * DT
    state.y = state.y + state.v * math.sin(state.yaw) * DT
    state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT
    state.v = state.v + a * DT

    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    return state


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):
    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def predict_motion(x0, oa, od, xref):
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
    for (ai, di, i) in zip(oa, od, range(1, T + 1)):
        state = update_state(state, ai, di)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.v
        xbar[3, i] = state.yaw

    return xbar


def iterative_linear_mpc_control(xref, x0, dref, oa, od):
    """
    MPC contorl with updating operational point iteraitvely
    """

    if oa is None or od is None:
        oa = [0.0] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, oa, od, xref)
        poa, pod = oa[:], od[:]
        oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
        du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")

    return oa, od, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, dref):
    """
    linear mpc control
    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(xbar[2, t], xbar[3, t], dref[0, t])
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]  # First state is the current state
    constraints += [x[2, :] <= MAX_SPEED]
    constraints += [x[2, :] >= MIN_SPEED]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        ov = get_nparray_from_matrix(x.value[2, :])
        oyaw = get_nparray_from_matrix(x.value[3, :])
        oa = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

    return oa, odelta, ox, oy, oyaw, ov


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)
    # print(ind)
    # print(pind)
    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        travel += abs(state.v) * DT
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref


def smooth_yaw(yaw):
    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


class P_controller:

    def __init__(self, robot, logging=True):
        self.robot = robot  # do not delete this line
        self.logging = logging

        if (logging == True):
            self.robot.make_headers(['pos_X', 'posY', 'posZ', 'vix', 'viy', 'wi', 'vr', 'wr'])
        self.state = State()
        # self.set_goal_points()

    # Edit goal point list below, if you click a point using mouse, the points programmed
    # will be washed out

    def initpath(self):  # Used to initialize goal point and initial parameters for control
        self.dl = 1  # m

        # Create Reference Trajectory from Planned Path
        # self.create_spline()

        # Initialize Yaw for first step
        self.init_yaw()

        self.cyaw = smooth_yaw(self.cyaw)
        # Final Goal Location
        # self.robot.state_des.add_destination(x=self.cx[-1], y=self.cy[-1], theta=self.cyaw[-1])

        # Get speed Profile
        self.sp = calc_speed_profile(self.cx, self.cy, self.cyaw, TARGET_SPEED)

        # choose closes point
        self.target_ind, _ = calc_nearest_index(self.state, self.cx, self.cy, self.cyaw, 0)

        # Initial state
        (c_posX, c_posY, c_theta) = self.robot.state.get_pos_state()  # get current position configuration
        (c_v, c_w) = self.robot.state.get_local_vel_state()  # Velocity in Robot Frame
        self.state = State(x=c_posX, y=c_posY, yaw=c_theta, v=c_v)  # Initialize state

        self.odelta, self.oa = None, None

    def track_point(self):
        # -------------------------------------
        self.initpath()
        # ______________________________
        # All c_ means current
        (c_posX, c_posY, c_theta) = self.robot.state.get_pos_state()  # get current position configuration
        (c_v, c_w) = self.robot.state.get_local_vel_state()  # get current local velocity configuration
        # ----------------------------------
        # Main MPC Program

        # Get state for controller
        xref, self.target_ind, dref = calc_ref_trajectory(
            self.state, self.cx, self.cy, self.cyaw, self.ck,
            self.sp, self.dl, self.target_ind)
        print("xref = ", xref)
        x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]
        # print(x0)

        # MPC Plan Time

        # t = time.time()
        self.oa, self.odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, x0, dref, self.oa, self.odelta)
        # print(time.time()-t)
        if self.odelta is not None:
            di, ai = self.odelta[0], self.oa[0]
        print("ai = ", ai)
        print("di = ", di)

        # Update State per controls
        self.state = update_state(self.state, ai, di)
        # -------------------------------------------------
        # Pass Linear and Angular speed to simulation
        c_v = self.state.v
        c_w = c_v * np.tan(di) / WB
        # print(self.state.v)
        # print(c_w)

        # self.robot.set_motor_control(linear velocity (cm), angular velocity (rad))
        self.robot.set_motor_control(c_v, c_w)  # use this command to set robot's speed in local frame
        self.robot.send_wheel_speed(self.wheel_spd(di), math.degrees(di))  # unit rad/s

        # use the following to log the variables, use [] to bracket all variables you want to store
        # stored values are in log folder
        # if self.logging == True:
        #     self.robot.log_data([self.state.x, c_posY, c_theta, c_v, c_w])

        if abs(c_posX - self.cx[-1]) < 10 and abs(c_posY - self.cy[-1]) < 10:  # you need to modify the reach way point criteria
            if (self.robot.state_des.reach_destination()):
                print("final goal reached")
                self.robot.set_motor_control(.0, .0)  # stop the motor
                return True
        return False

    def create_spline(self, x, y):  # Used to set ref path in path planner

        # Create spline to track to goal location
        self.cx, self.cy, self.cyaw, self.ck, self.s = cubic_spline_planner.calc_spline_course(x, y, ds=1)

        # Round to integers
        for i in range(0, len(self.cx)):
            self.cx[i] = int(self.cx[i])
            self.cy[i] = int(self.cy[i])
            self.cyaw[i] = round(self.cyaw[i],3)

    def init_yaw(self):
        # initial yaw compensation
        if self.state.yaw - self.cyaw[0] >= math.pi:
            self.state.yaw -= math.pi * 2.0
        elif self.state.yaw - self.cyaw[0] <= -math.pi:
            self.state.yaw += math.pi * 2.0

    def wheel_spd(self, di):

        X = WB * math.tan(math.radians(90 - di))
        Y = WB * math.tan(math.radians(90 - di))
        R_icr = X + L2 / 2
        L_icr = Y - L2 / 2
        icr_rear = X + L2 / 2
        Rrr = X
        Rrl = X + L2
        Rfr = math.sqrt(X ** 2 + L2 ** 2)
        Rfl = math.sqrt((X + L2) ** 2 + L2 ** 2)

        wfr = self.state.v * (Rfr) / (R_icr * r)
        wfl = self.state.v * (Rfl) / (L_icr * r)
        wrr = self.state.v * (Rrr) / (icr_rear * r)
        wrl = self.state.v * (Rrl) / (icr_rear * r)

        max_speed = max(wfr, wfl, wrr, wrl)
        return max_speed
