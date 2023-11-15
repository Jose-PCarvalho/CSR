import copy

import numpy as np
from math import sin, cos, pi
from scipy.optimize import fsolve
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse as EllipsePlot
from matplotlib.path import Path


class Simulation:
    def __init__(self, cfg):

        self.n_agents = len(cfg['agent_position'])
        # Parameters of range
        self.MAX_RANGE = cfg['max_dist_between_agents']
        self.MAX_R_PICK = cfg['pickup_dist']
        self.MAX_R_DIS = cfg['max_dist_with_resource']
        self.verbose = cfg['verbose']
        self.leader = cfg['leader']

        # Resource
        self.initial_resource = cfg['resource_position']
        self.resource = copy.deepcopy(self.initial_resource)
        self.num_R = len(cfg['resource_position'])
        self.R_Caught = np.zeros(self.num_R,dtype=np.bool)

        self.arena = ConvexPolygon(cfg['map_boundaries'])
        self.world = World(self.arena)
        if 'obstacles' in cfg:
            for obs in cfg['obstacles']:
                if 'Ellipse' in obs:
                    obs_param = obs['Ellipse']
                    o = Ellipse(center=obs_param['center'], angle=np.radians(obs_param['angle']), axes=obs_param['axes'])
                    self.world.add_obstacle(o)

        # Time of simulation
        self.sim_time = cfg['sim_time']
        self.dt = cfg['dt']

        self.t_signal = np.arange(0, self.sim_time, self.dt)  # time samples
        x_signal = np.zeros((2, len(self.t_signal)))
        x_signal = np.expand_dims(x_signal, axis=0)
        self.x_signal = np.repeat(x_signal, self.n_agents, axis=0)
        u_signal = np.zeros((2, len(self.t_signal)))
        u_signal = np.expand_dims(u_signal, axis=0)
        self.u_signal = np.repeat(u_signal, self.n_agents, axis=0)

        # Initial conditions of each drone
        for i in range(self.n_agents):

            self.x_signal[i, :, 0] = cfg['agent_position'][i]
        self.cpx = [[] for _ in range(self.n_agents)]
        self.cpy = [[] for _ in range(self.n_agents)]

        self.do = np.zeros(self.n_agents)  # Distance to obstacle
        self.d_mtx = np.zeros(self.n_agents)  # Damaged matrix
        self.R_delivered = np.zeros(self.num_R, dtype=np.bool)  # Indicates if Resource was delivered at the warehouse
        self.w_R_mtx = np.zeros(self.n_agents)  # Matrix that tells if a pair of drones caught some R
        self.time_score = 999.9 * np.ones(
            self.num_R * 2)  # 0 - caught R_1, 1 - caught R_2, 2 - drop R_1 in warehouse, 3 - drop R_2 in warehouse
        self.position_mtx = np.zeros((self.n_agents, self.n_agents))  # matrix that indicates which drones are near
        self.t = 0

        self.state_dict = {
            "Resource Position": self.resource,
            "Resource Caught": self.R_Caught,
            "Resource Delivered": self.R_delivered,
            "Distance to Obstacle": self.do,
            "Leader State": self.x_signal[self.leader, :, self.t],
        }

    def get_relative_positions(self):

        xij = np.zeros((self.n_agents, self.n_agents, 2))
        angle = []
        for i in range(self.n_agents):
            for j in range(self.n_agents):
                xij[i][j] = self.x_signal[i, :, self.t] - self.x_signal[j, :, self.t]
                if i==j:
                    self.position_mtx[i][j] = 0
                    self.position_mtx[j][i] = 0
                elif np.linalg.norm(xij[i][j], 2) > self.MAX_RANGE:
                    xij[i][j][0] = 0
                    xij[i][j][1] = 0
                    self.position_mtx[i][j] = 0
                    self.position_mtx[j][i] = 0
                else:
                    self.position_mtx[i][j] = 1
                    self.position_mtx[j][i] = 1

        for i in range(self.n_agents):
            closest_pt, self.do[i] = self.world.compute_closest(self.x_signal[i, :, self.t])

            angle.append(
                np.arctan2(closest_pt[1] - self.x_signal[i, 1, self.t], closest_pt[0] - self.x_signal[i, 0, self.t]))
            self.cpx[i].append(closest_pt[0])
            self.cpy[i].append(closest_pt[1])

        return xij, angle

    def step(self, u_control):
        x1 = self.x_signal[0, :, self.t]
        x2 = self.x_signal[1, :, self.t]
        x3 = self.x_signal[2, :, self.t]
        x4 = self.x_signal[3, :, self.t]
        x5 = self.x_signal[4, :, self.t]
        points_vec = np.array([x1, x2, x3, x4, x5])

        # Check collisions
        self.check_collisions(points_vec)

        # compute a proximity table to check what are the drones that are closer to the Resources
        prox_mtx = self.proximity(points_vec)

        # Send actuation commands
        for i in range(len(self.d_mtx)):
            if self.d_mtx[i] == 1:
                u_control[i] = [0, 0]
                prox_mtx[0][i] = 99
                prox_mtx[1][i] = 99
            if all(self.position_mtx[i] == np.zeros(self.n_agents)) and i != self.leader:

                u_control[i] = [0, 0]

        self.closest_drones(prox_mtx)


        for i in range(self.num_R):
            if i+1 in self.w_R_mtx:
                self.R_Caught[i] = True
            else:
                self.R_Caught[i] = False


        # Saturate the control inputs between -1 and 1
        for a, u in enumerate(u_control):
            self.u_signal[a, :, self.t] = np.clip(u, -1, 1)
            self.x_signal[a, :, self.t + 1] = step_f(self.x_signal[a, :, self.t], self.u_signal[a, :, self.t], self.dt)

        self.R_warehouse_check(points_vec)

        self.t += 1

        self.state_dict = {
            "Resource Position": self.resource,
            "Resource Caught": self.R_Caught,
            "Resource Delivered": self.R_delivered,
            "Distance to Obstacle": self.do,
            "Leader State": self.x_signal[self.leader, :, self.t],
        }

    def proximity(self, points_vec):
        prox_mtx = []
        for r in self.resource:
            temp = np.zeros(self.n_agents)
            for a in range(self.n_agents):
                temp[a] = np.sqrt((points_vec[a][0] - r[0]) ** 2 + (points_vec[a][1] - r[1]) ** 2)
            prox_mtx.append(temp)

        return prox_mtx

    def R_warehouse_check(self, points_vec):
        t = self.t_signal[self.t]
        for i in range(self.num_R):
            if i + 1 in self.w_R_mtx:

                result = np.where(self.w_R_mtx == i + 1)
                index = result[0][0]
                index_2 = result[0][1]

                if points_vec[index][0] < -5 and points_vec[index][1] < -5 and points_vec[index_2][0] < -5 and \
                        points_vec[index_2][1] < -5:  # Arriving to base

                    self.resource[i][0] = points_vec[index][0]
                    self.resource[i][1] = points_vec[index][1]
                    self.time_score[i + self.num_R] = t
                    self.w_R_mtx[index] = 0
                    self.w_R_mtx[index_2] = 0
                    self.R_delivered[i] = True

                    print('Resource ' + str(i + 1) + ' delivered  at time t=', t)

                elif np.sqrt((points_vec[index][0] - points_vec[index_2][0]) ** 2 + (
                        points_vec[index][1] - points_vec[index_2][
                    1]) ** 2) > self.MAX_R_DIS:  # 1.5: # Loosing Resource

                    self.w_R_mtx[index] = 0
                    self.w_R_mtx[index_2] = 0
                    self.time_score[i] = 999.9

                    print('WARNING: Resource ' + str(i + 1) + ' lost!')

    def closest_drones(self, prox_mtx):
        # Find the closest drones to the Resource
        closer = 99
        pre_candidate = -1
        candidate = -1
        for r in range(self.num_R):
            for i in range(0, 5):

                if prox_mtx[r][i] < closer and self.w_R_mtx[i] == 0:  # Finds the closest drone without Resource
                    closer = prox_mtx[r][i]
                    pre_candidate = i

            closer = 99

            if pre_candidate != -1:  # If a drone without Resource was found and is the closest

                if prox_mtx[r][pre_candidate] < self.MAX_R_PICK and r + 1 not in self.w_R_mtx and not self.R_delivered[
                    r]:  # If distance to the Resource is smaller than dist_R and Resource 1 is not delivered and isn't with any other drone

                    if self.verbose == 1:
                        print('Drone', pre_candidate + 1, 'found the Resource ', r + 1, ' at time t=',
                              self.t_signal[self.t])

                    for i in range(0, 5):

                        if i != pre_candidate and self.w_R_mtx[
                            i] == 0:  # Verifies all drones except the closest one, and the ones with Resource

                            if prox_mtx[r][i] < self.MAX_R_PICK and prox_mtx[0][
                                i] < closer:  # If closer than dist_R, closer than closer (to get the minimum) and without Resource
                                closer = prox_mtx[0][i]
                                candidate = i

                    if candidate != -1:

                        if self.verbose == 1:
                            print('Drone', candidate + 1, 'found the Resource ', r + 1, ' at time t=',
                                  self.t_signal[self.t])

                        self.w_R_mtx[pre_candidate] = r + 1
                        self.w_R_mtx[candidate] = r + 1
                        self.time_score[r] = self.t_signal[self.t]

                    else:
                        if self.verbose == 1:
                            print('But no other drone found the Resource ', r + 1, '...')

    def check_collisions(self, points_vec):
        for i in range(self.n_agents):
            if self.d_mtx[i] == 0:
                if points_vec[i][0] > 10 or points_vec[i][0] < -10 or points_vec[i][1] > 10 or points_vec[i][
                    1] < -10:  # Verify if the drone left the arena
                    self.d_mtx[i] = 1
                    print('WARNING: Drone', i + 1, 'left the arena!!')
                    if self.w_R_mtx[i] != 0:
                        temp = self.w_R_mtx[i]
                        self.w_R_mtx[i] = 0
                        self.w_R_mtx[np.where(self.w_R_mtx == temp)[0][0]] = 0
                    continue
                elif self.do[i] < 0.01:  # If distance to nearest object is smaller than 0.01, the drone is damaged
                    self.d_mtx[i] = 1
                    print('WARNING: Collision with obstacle! Drone:', i + 1)
                    if self.w_R_mtx[i] != 0:
                        temp = self.w_R_mtx[i]
                        self.w_R_mtx[i] = 0
                        self.w_R_mtx[np.where(self.w_R_mtx == temp)[0][0]] = 0
                    continue
                # Verify collisions between drones
                for j in range(self.n_agents):
                    if i != j:
                        dist = np.linalg.norm(points_vec[i] - points_vec[j], 2)
                        if dist < 0.1:
                            self.d_mtx[i] = 1
                            self.d_mtx[j] = 1
                            print('WARNING: Collision between drone', i + 1, 'and drone', j + 1, '!!', ' -> dist =',
                                  dist)
                            if self.w_R_mtx[i] != 0:
                                temp = self.w_R_mtx[i]
                                self.w_R_mtx[i] = 0
                                self.w_R_mtx[np.where(self.w_R_mtx == temp)[0][0]] = 0
                                self.time_score[temp - 1] = 999.9
                                print('WARNING: Resource ', str(temp), ' lost!')

                            if self.w_R_mtx[j] != 0:
                                temp = self.w_R_mtx[j]
                                self.w_R_mtx[j] = 0
                                self.w_R_mtx[np.where(self.w_R_mtx == temp)[0][0]] = 0
                                self.time_score[temp - 1] = 999.9
                                print('WARNING: Resource ', str(temp), ' lost!')


def step_f(x, u, h):
    return x + u * h

class Ellipse():
    '''

    Implementation of elliptical obstacle class.

    '''

    def __init__(self, center=[0.0, 0.0], angle=0.0, axes=[1.0, 1.0]):

        self.center = np.array(center)

        self.angle = angle

        self.axes = np.array(axes)

        self.point = np.zeros(2)

        s = np.sin(self.angle)

        c = np.cos(self.angle)

        self.R = np.array([[c, s], [-s, c]])

    def equations(self, vars):

        '''

        Equation for solving the 'nearest point to ellipse' problem.

        '''

        l, gamma = vars

        a = self.axes[0]

        b = self.axes[1]

        term = self.R @ (self.point - self.center)

        eq1 = (a + l * b) * cos(gamma) - term[0]

        eq2 = (b + l * a) * sin(gamma) - term[1]

        return [eq1, eq2]

    def compute_closest(self, point):

        '''

        Computes the closest point on the ellipse with respect to a given point.

        '''

        if self.isInside(point):
            return point, 0.0

        self.point = np.array(point)

        initial_l = np.random.rand()

        initial_gamma = pi * np.random.rand()

        l_sol, gamma_sol = fsolve(self.equations, (initial_l, initial_gamma))

        while l_sol < 0:
            initial_l = np.random.rand()

            initial_gamma = pi * np.random.rand()

            l_sol, gamma_sol = fsolve(self.equations, (initial_l, initial_gamma))

        a = self.axes[0]

        b = self.axes[1]

        v = np.array([a * cos(gamma_sol), b * sin(gamma_sol)])

        n = np.array([b * cos(gamma_sol), a * sin(gamma_sol)])

        closest_point = self.R.T @ v + self.center

        # distance = l_sol*np.linalg.norm(n)

        distance = np.linalg.norm(closest_point - point)

        return closest_point, distance

    def plot(self):

        ellipse = EllipsePlot(self.center, 2 * self.axes[0], 2 * self.axes[1], angle=np.degrees(self.angle), color='g',
                              label='obstacle')

        plt.gca().add_patch(ellipse)

    def isInside(self, point):

        '''

        Checks if point is inside of ellipse.

        '''

        a = self.axes[0]

        b = self.axes[1]

        v = self.R @ (np.array(point) - self.center)

        Lambda = np.diag([1 / a ** 2, 1 / b ** 2])

        expr = v.T @ Lambda @ v - 1

        return expr < 0


class Line():
    '''

    Implementation of simple line.

    '''

    def __init__(self, p1=[0.0, 0.0], p2=[1.0, 1.0]):

        self.p1 = np.array(p1)

        self.p2 = np.array(p2)

        self.line_vector = self.p2 - self.p1

        self.angle = np.arctan2(self.line_vector[1], self.line_vector[0])

        self.normal = np.array([-sin(self.angle), cos(self.angle)])

        self.point = np.zeros(2)

    def equations(self, vars):

        '''

        Equation for solving the 'nearest point to wall' problem.

        '''

        l, gamma = vars

        eq1 = self.p1[0] * (1 - gamma) + self.p2[0] * gamma + l * self.normal[0] - self.point[0]

        eq2 = self.p1[1] * (1 - gamma) + self.p2[1] * gamma + l * self.normal[1] - self.point[1]

        return [eq1, eq2]

    def compute_closest(self, point):

        '''

        Computes the closest point on the line with respect to a given point.

        '''

        self.point = np.array(point)

        initial_l = np.random.rand()

        initial_gamma = np.random.rand()

        l_sol, gamma_sol = fsolve(self.equations, (initial_l, initial_gamma))

        distance = np.abs(l_sol)

        if gamma_sol > 1:

            gamma_sol = 1

            distance = np.linalg.norm(self.point - self.p2)

        elif gamma_sol < 0:

            gamma_sol = 0

            distance = np.linalg.norm(self.point - self.p1)

        closest_point = self.p1 * (1 - gamma_sol) + self.p2 * gamma_sol

        return closest_point, distance

    def plot(self):

        '''

        Plot line.

        '''

        plt.plot(self.p1[0], self.p1[1], color='r', marker='*')

        plt.plot(self.p2[0], self.p2[1], color='r', marker='*')

        plt.plot([self.p1[0], self.p2[0]], [self.p1[1], self.p2[1]], color='b', linewidth=2.0)


class ConvexPolygon():
    '''

    Implementation of a convex polygon class. Vertices must be the vertices of a convex polygon.

    '''

    def __init__(self, vertices):

        self.vertices = np.array(vertices)

        self.num_vertices = len(self.vertices)

        self.lines = []

        self.create_lines()

        self.hull = ConvexHull(self.vertices)

        self.hull_path = Path(self.vertices[self.hull.vertices])

    def create_lines(self):

        '''

        Create lines from vertice points.

        '''

        for k in range(self.num_vertices - 1):
            self.lines.append(Line(p1=self.vertices[k, :], p2=self.vertices[k + 1, :]))

        self.lines.append(Line(p1=self.vertices[-1, :], p2=self.vertices[0, :]))

    def isInside(self, point):

        '''

        Checks whether the point is inside the ConvexPolygon or not.

        '''

        return self.hull_path.contains_point(point)

    def compute_closest(self, point):

        '''

        Computes the closest point on the polygon to point.

        '''

        if self.isInside(point):
            return point, 0.0

        num_lines = len(self.lines)

        line_distances = np.zeros(num_lines)

        line_pts = np.zeros([2, num_lines])

        for k in range(num_lines):
            line_pts[:, k], line_distances[k] = self.lines[k].compute_closest(point)

        closest_line_index = np.argmin(line_distances)

        closest_line_pt = line_pts[:, closest_line_index]

        closest_line_dist = line_distances[closest_line_index]

        return closest_line_pt, closest_line_dist

    def plot(self):

        '''

        Plot polygon.

        '''

        for line in self.lines:
            line.plot()


class World():
    '''

    Implementation of simple class for a world with a polygonal arena and obstacles (elliptical and polygonal)

    '''

    def __init__(self, arena):

        self.arena = arena

        self.obstacles = []

        self.closest_object = None

    def add_obstacle(self, obs):

        '''

        Adds obstacles to the simulated world.

        '''

        self.obstacles.append(obs)

    def compute_closest(self, point):

        '''

        Computes the closest point on the obstacles or walls, with respect to point.

        '''

        # First, checks the arena walls

        closest_wall_dist = float('inf')

        num_walls = len(self.arena.lines)

        if num_walls > 0:

            wall_distances = np.zeros(num_walls)

            wall_pts = np.zeros([2, num_walls])

            for k in range(num_walls):
                wall_pts[:, k], wall_distances[k] = self.arena.lines[k].compute_closest(point)

            closest_wall_index = np.argmin(wall_distances)

            closest_wall_pt = wall_pts[:, closest_wall_index]

            closest_wall_dist = wall_distances[closest_wall_index]

        # Then, checks the obstacles

        closest_obs_dist = float('inf')

        num_obs = len(self.obstacles)

        if num_obs > 0:

            obs_distances = np.zeros(num_obs)

            obs_pts = np.zeros([2, num_obs])

            for k in range(num_obs):
                obs_pts[:, k], obs_distances[k] = self.obstacles[k].compute_closest(point)

            closest_obs_index = np.argmin(obs_distances)

            closest_obs_pt = obs_pts[:, closest_obs_index]

            closest_obs_dist = obs_distances[closest_obs_index]

        if closest_obs_dist < closest_wall_dist:

            closest_pt = closest_obs_pt

            closest_dist = closest_obs_dist

            self.closest_obstacle = self.obstacles[closest_obs_index]

        else:

            closest_pt = closest_wall_pt

            closest_dist = closest_wall_dist

            self.closest_obstacle = self.arena.lines[closest_wall_index]

        return closest_pt, closest_dist

    def plot(self):

        '''

        Plot the whole world.

        '''

        self.arena.plot()

        for obs in self.obstacles:
            obs.plot()
