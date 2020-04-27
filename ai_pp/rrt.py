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


def rrt_path_planning(self, objectives, obstacles, force_repath):
    global previous_obstacles_hash, planned_path, drawable_lines, target

    obstacles_hash = hash_obstacle_map(obstacles)
    if planned_path is None or force_repath or previous_obstacles_hash != obstacles_hash:
        previous_obstacles_hash = obstacles_hash
        planner = RRTPP()
        res = planner.calculate_path({'x': self.x, 'y': self.y}, objectives[0], obstacles)
        if not res:
            print 'not possible'

            if len(planner.rrt_graph) > 0:
                drawable_lines = [line_to(**l) for l in planner.get_drawable_lines()]
            else:
                drawable_lines = []

            return {'vx': 0, 'vy': 0, 'vr': 0}, drawable_lines, True
        planned_path = planner.planned_path
        drawable_lines = [line_to(**l) for l in planner.get_drawable_lines()]
        planned_path.pop()
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


class RRTPP:
    def __init__(self):
        self.selected_objective = None  # {'x': 0, 'y': 0}
        self.obstacles = []
        self.rrt_graph = []
        self.planned_path = []
        self.avoidance_resolution = 10
        self.initial_position = None

    def calculate_path(self, initial_position, objective, obstacles, avoidance_resolution=None):
        self.obstacles = obstacles
        self.selected_objective = {'x': objective.x, 'y': objective.y}
        if avoidance_resolution is not None:
            self.avoidance_resolution = avoidance_resolution
        self.initial_position = initial_position

        return self.rrt_shot(self.avoidance_resolution) or self.rrt_shot(self.avoidance_resolution * 5) or self.rrt_shot(self.avoidance_resolution * 10)

    def rrt_shot(self, success_threshold):
        self.rrt_graph = [{'x': self.initial_position['x'], 'y': self.initial_position['y'], 'parent_id': -1}]
        for i in range(800):
            rnd_node = self.get_random_node()
            nearest_node_index = self.get_nearest_node_index(rnd_node)
            near_goal_node = self.steer(nearest_node_index, rnd_node, success_threshold)

            if near_goal_node is not None:
                self.generate_final_course(len(self.rrt_graph) - 1)
                return True
        return False

    def get_nearest_node_index(self, rnd_node):
        min_dist = None
        min_id = None

        for i, p in enumerate(self.rrt_graph):
            d = self.dist_to(p, rnd_node)
            if min_dist is None or d < min_dist:
                min_dist = d
                min_id = i
        return min_id

    def steer(self, parent_id, to_node, success_threshold):
        from_node = self.rrt_graph[parent_id]
        extend_length = min(self.dist_to(from_node, to_node), 5*self.avoidance_resolution)
        n_expand = int(extend_length / self.avoidance_resolution)
        theta = self.angle_to(to_node, from_node)

        x = from_node['x']
        y = from_node['y']

        for i in range(n_expand):
            x += self.avoidance_resolution * math.cos(theta)
            y += self.avoidance_resolution * math.sin(theta)
            new_node = {
                'x': x,  # //from_node.t1 + Math.cos(theta) * rrt_res,
                'y': y,  # //from_node.t2 + Math.sin(theta) * rrt_res,
                'parent_id': parent_id,
            }

            if self.check_point_inside_obstacle(new_node):
                break

            self.rrt_graph.append(new_node)
            if self.calc_dist_to_goal(new_node) <= success_threshold:
                return new_node
        return None

    def check_point_inside_obstacle(self, point):
        for o in self.obstacles:
            if self.intersect_circle_and_rect(point['x'], point['y'], self.avoidance_resolution, o.x, o.y, o.r*2, o.r*2):
                return True
        return False

    @staticmethod
    def intersect_circle_and_rect(cx, cy, cr, rx, ry, rw, rh):
        dx = math.fabs(cx - rx)
        dy = math.fabs(cy - ry)

        if dx > (rw / 2 + cr):
            return False
        if dy > (rh / 2 + cr):
            return False

        if dx <= (rw / 2):
            return True
        if dy <= (rh / 2):
            return True

        corner_distance = (dx - rw / 2) ** 2 + (dy - rh / 2) ** 2
        return corner_distance <= (cr ** 2)

    def get_random_node(self):
        if random() > 0.1:
            p = None
            while p is None or self.check_point_inside_obstacle(p):
                p = {
                    'x': randint(-FIELD_MAX, +FIELD_MAX),
                    'y': randint(-FIELD_MAX, +FIELD_MAX),
                }
            return p
        else:  # goal point sampling
            return self.selected_objective

    def calc_dist_to_goal(self, p):
        return self.dist_to(p, self.selected_objective)

    @staticmethod
    def dist_to(p1, p2):
        dt1 = p1['x'] - p2['x']
        dt2 = p1['y'] - p2['y']
        return (dt1 ** 2 + dt2 ** 2) ** .5

    @staticmethod
    def angle_to(p1, p2):
        dt1 = p1['x'] - p2['x']
        dt2 = p1['y'] - p2['y']
        return math.atan2(dt2, dt1)

    def generate_final_course(self, goal_ind):
        self.planned_path = [self.selected_objective]
        node = self.rrt_graph[goal_ind]
        while node['parent_id'] != -1:
            self.planned_path.append({
                'x': node['x'],
                'y': node['y']
            })
            node = self.rrt_graph[node['parent_id']]
        self.planned_path.append({
            'x': node['x'],
            'y': node['y']
        })

    def get_drawable_lines(self):
        lines = []
        for p in self.rrt_graph:
            p_parent = self.rrt_graph[p['parent_id']] if p['parent_id'] >= 0 else self.selected_objective
            lines.append({
                'x1': p['x'],
                'y1': p['y'],
                'x2': p_parent['x'],
                'y2': p_parent['y'],
                'is_red': p['parent_id'] >= 0
            })
        return lines


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
