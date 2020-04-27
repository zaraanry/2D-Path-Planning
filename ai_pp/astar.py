import math
from random import random, randint
from config import FIELD_MAX


planned_path = None
previous_obstacles_hash = None
drawable_lines = None
target = None


def hash_obstacle_map(obstacles):
    sh = 0
    for o in obstacles:
        sh += 5.2 * o.x + 3.7 * o.y + 1.2
    return sh


def a_star_path_planning(self, objectives, obstacles, force_repath):
    global previous_obstacles_hash, planned_path, drawable_lines, target

    obstacles_hash = hash_obstacle_map(obstacles)
    if planned_path is None or force_repath or previous_obstacles_hash != obstacles_hash:
        previous_obstacles_hash = obstacles_hash
        planned_path, drawables = main_function(self, objectives[0], obstacles)
        drawable_lines = [line_to(**l) for l in drawables]
        if len(planned_path) <= 0:
            return {'vx': 0, 'vy': 0, 'vr': 0}, drawable_lines, True
        target = planned_path.pop()

    dx = float(target['x'] - self.x)
    dy = float(target['y'] - self.y)
    max_delta = max(math.fabs(dx), math.fabs(dy))

    if math.fabs(dx) < objectives[0].r*.6 and math.fabs(dy) < objectives[0].r*.6:
        if len(planned_path) > 0:
            target = planned_path.pop()
        else:
            print 'reached'
            return {'vx': 0, 'vy': 0, 'vr': 0}, [], True

    dx /= max_delta
    dy /= max_delta
    return {'vx': dx, 'vy': dy, 'vr': 0}, drawable_lines, False


# class RRTPP:
#     def __init__(self):
#         self.selected_objective = None  # {'x': 0, 'y': 0}
#         self.obstacles = []
#         self.rrt_graph = []
#         self.planned_path = []
#         self.avoidance_resolution = 10
#         self.initial_position = None
#
#     def calculate_path(self, initial_position, objective, obstacles, avoidance_resolution=None):
#         self.obstacles = obstacles
#         self.selected_objective = {'x': objective.x, 'y': objective.y}
#         if avoidance_resolution is not None:
#             self.avoidance_resolution = avoidance_resolution
#         self.initial_position = initial_position
#
#         return self.rrt_shot(self.avoidance_resolution) or self.rrt_shot(self.avoidance_resolution * 5) or self.rrt_shot(self.avoidance_resolution * 10)
#
#     def rrt_shot(self, success_threshold):
#         self.rrt_graph = [{'x': self.initial_position['x'], 'y': self.initial_position['y'], 'parent_id': -1}]
#         for i in range(800):
#             rnd_node = self.get_random_node()
#             nearest_node_index = self.get_nearest_node_index(rnd_node)
#             near_goal_node = self.steer(nearest_node_index, rnd_node, success_threshold)
#
#             if near_goal_node is not None:
#                 self.generate_final_course(len(self.rrt_graph) - 1)
#                 return True
#         return False
#
#     def get_nearest_node_index(self, rnd_node):
#         min_dist = None
#         min_id = None
#
#         for i, p in enumerate(self.rrt_graph):
#             d = self.dist_to(p, rnd_node)
#             if min_dist is None or d < min_dist:
#                 min_dist = d
#                 min_id = i
#         return min_id
#
#     def steer(self, parent_id, to_node, success_threshold):
#         from_node = self.rrt_graph[parent_id]
#         extend_length = min(self.dist_to(from_node, to_node), 5*self.avoidance_resolution)
#         n_expand = int(extend_length / self.avoidance_resolution)
#         theta = self.angle_to(to_node, from_node)
#
#         x = from_node['x']
#         y = from_node['y']
#
#         for i in range(n_expand):
#             x += self.avoidance_resolution * math.cos(theta)
#             y += self.avoidance_resolution * math.sin(theta)
#             new_node = {
#                 'x': x,  # //from_node.t1 + Math.cos(theta) * rrt_res,
#                 'y': y,  # //from_node.t2 + Math.sin(theta) * rrt_res,
#                 'parent_id': parent_id,
#             }
#
#             if self.check_point_inside_obstacle(new_node):
#                 break
#
#             self.rrt_graph.append(new_node)
#             if self.calc_dist_to_goal(new_node) <= success_threshold:
#                 return new_node
#         return None
#
#     def check_point_inside_obstacle(self, point):
#         for o in self.obstacles:
#             if self.intersect_circle_and_rect(point['x'], point['y'], self.avoidance_resolution, o.x+o.r/2, o.y+o.r/2, o.r, o.r):
#                 return True
#         return False
#
#     @staticmethod
#     def intersect_circle_and_rect(cx, cy, cr, rx, ry, rw, rh):
#         dx = math.fabs(cx - rx)
#         dy = math.fabs(cy - ry)
#
#         if dx > (rw / 2 + cr):
#             return False
#         if dy > (rh / 2 + cr):
#             return False
#
#         if dx <= (rw / 2):
#             return True
#         if dy <= (rh / 2):
#             return True
#
#         corner_distance = (dx - rw / 2) ** 2 + (dy - rh / 2) ** 2
#         return corner_distance <= (cr ** 2)
#
#     def get_random_node(self):
#         if random() > 0.1:
#             p = None
#             while p is None or self.check_point_inside_obstacle(p):
#                 p = {
#                     'x': randint(-FIELD_MAX, +FIELD_MAX),
#                     'y': randint(-FIELD_MAX, +FIELD_MAX),
#                 }
#             return p
#         else:  # goal point sampling
#             return self.selected_objective
#
#     def calc_dist_to_goal(self, p):
#         return self.dist_to(p, self.selected_objective)
#
#     @staticmethod
#     def dist_to(p1, p2):
#         dt1 = p1['x'] - p2['x']
#         dt2 = p1['y'] - p2['y']
#         return (dt1 ** 2 + dt2 ** 2) ** .5
#
#     @staticmethod
#     def angle_to(p1, p2):
#         dt1 = p1['x'] - p2['x']
#         dt2 = p1['y'] - p2['y']
#         return math.atan2(dt2, dt1)
#
#     def generate_final_course(self, goal_ind):
#         self.planned_path = [self.selected_objective]
#         node = self.rrt_graph[goal_ind]
#         while node['parent_id'] != -1:
#             self.planned_path.append({
#                 'x': node['x'],
#                 'y': node['y']
#             })
#             node = self.rrt_graph[node['parent_id']]
#         self.planned_path.append({
#             'x': node['x'],
#             'y': node['y']
#         })
#
#     def get_drawable_lines(self):
#         lines = []
#         for p in self.rrt_graph:
#             p_parent = self.rrt_graph[p['parent_id']] if p['parent_id'] >= 0 else self.selected_objective
#             lines.append({
#                 'x1': p['x'],
#                 'y1': p['y'],
#                 'x2': p_parent['x'],
#                 'y2': p_parent['y'],
#                 'is_red': p['parent_id'] >= 0
#             })
#         return lines


def line_to(x1, y1, x2, y2, is_red):
    return {
        'shape': 'lines',
        'color': '#f00' if is_red else '#0f0',
        'fill': False,
        'points': [
            {'x': x1, 'y': y1},
            {'x': x2, 'y': y2},
        ]
    }


class AStarPlanner:
    def __init__(self, ox, oy, reso, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        nstart = self.Node(self.calc_xyindex(sx, self.minx), self.calc_xyindex(sy, self.miny), 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx), self.calc_xyindex(gy, self.miny), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))
            current = open_set[c_id]

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Find goal")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(ngoal, closed_set)

        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # generate final course
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [self.calc_grid_position(ngoal.y, self.miny)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            pind = n.pind

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, minp):
        """
        calc grid position
        :param index:
        :param minp:
        :return:
        """
        pos = index * self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.reso)

    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        try:
            if self.obmap[int(node.x)][int(node.y)]:
                return False
        except:
            return True
        return True

    def calc_obstacle_map(self, ox, oy):
        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))
        print("minx:", self.minx)
        print("miny:", self.miny)
        print("maxx:", self.maxx)
        print("maxy:", self.maxy)

        self.xwidth = int(round((self.maxx - self.minx) / self.reso))
        self.ywidth = int(round((self.maxy - self.miny) / self.reso))
        print("xwidth:", self.xwidth)
        print("ywidth:", self.ywidth)

        # obstacle map generation
        self.obmap = [[False for i in range(self.ywidth)] for i in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main_function(initial_position, objective, obstacles):
    print '\tcalculating obstacles...'
    grid_size = 10
    padding = initial_position.r * 2

    ox, oy = [], []
    for o in obstacles:
        o_r = o.r * 2
        o_x = o.x - o_r - padding
        while o_x < o.x + o_r + padding:
            o_y = o.y - o_r - padding
            while o_y < o.y + o_r + padding:
                ox.append(o_x)
                oy.append(o_y)
                o_y += grid_size/2
            o_x += grid_size/2

    print '\tcalculating path...'
    a_star = AStarPlanner(ox, oy, grid_size, initial_position.r)
    rx, ry = a_star.planning(initial_position.x, initial_position.y, objective.x, objective.y)

    prev_px = objective.x
    prev_py = objective.y
    lines = []
    planned_path = []
    for px, py in zip(rx, ry):
        planned_path.append({
            'x': px,
            'y': py,
        })

        lines.append({
            'x1': px,
            'y1': py,
            'x2': prev_px,
            'y2': prev_py,
            'is_red': True
        })
        prev_px = px
        prev_py = py

    print '\t[Done]'
    return planned_path, lines
