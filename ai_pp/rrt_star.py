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


def rrt_star_path_planning(self, objectives, obstacles, force_repath):
    global previous_obstacles_hash, planned_path, drawable_lines, target

    obstacles_hash = hash_obstacle_map(obstacles)
    if planned_path is None or force_repath or previous_obstacles_hash != obstacles_hash:
        previous_obstacles_hash = obstacles_hash
        planner = RRTs()
        res = planner.calculate_path({'x': self.x, 'y': self.y}, objectives[0], obstacles)
        if not res:
            print 'not possible'

            if len(planner.nodes) > 0:
                drawable_lines = [line_to(**l) for l in planner.get_drawable_lines()]
            else:
                drawable_lines = []

            return {'vx': 0, 'vy': 0, 'vr': 0}, drawable_lines, True
        planned_path = planner.planned_path
        drawable_lines = [line_to(**l) for l in planner.get_drawable_lines()]
        planned_path.pop()
        target = planned_path.pop()

    dx = float(target.coord['x'] - self.x)
    dy = float(target.coord['y'] - self.y)
    max_delta = max(math.fabs(dx), math.fabs(dy))

    if math.fabs(dx) < objectives[0].r * .6 and math.fabs(dy) < objectives[0].r * .6:
        if len(planned_path) > 0:
            target = planned_path.pop()
        else:
            print 'reached'
            return {'vx': 0, 'vy': 0, 'vr': 0}, [], True

    dx /= max_delta
    dy /= max_delta
    return {'vx': dx, 'vy': dy, 'vr': 0}, drawable_lines, False


class Node:
    def __init__(self, coord=None, cost=0, parent=None):
        self.coord = coord
        self.cost = cost
        self.parent = parent


class RRTs:
    def __init__(self):
        self.selected_objective = None  # {'x': 0, 'y': 0}
        self.obstacles = []
        self.planned_path = []
        self.avoidance_resolution = 10
        self.initial_position = None
        self.nodes = []

    def calculate_path(self, initial_position, objective, obstacles):
        print '\tcalculating path...'
        self.obstacles = obstacles
        self.selected_objective = Node({'x': objective.x, 'y': objective.y})
        self.initial_position = Node(initial_position)
        self.run()
        print '\t[done]'
        return True

    def get_random_node(self):
        if random() > 0.1:
            p = None
            while p is None or self.check_point_inside_obstacle(p.coord):
                p = Node({
                    'x': randint(-FIELD_MAX, +FIELD_MAX),
                    'y': randint(-FIELD_MAX, +FIELD_MAX),
                })
            return p
        else:
            return self.selected_objective

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

    def check_if_reached(self):
        pass

    def find_nearest_point_from_nodes(self, point):
        min_dist = None
        min_id = None
        for idx, n in enumerate(self.nodes):
            dist = self.node_dist(point, n)
            if min_dist is None or min_dist > dist:
                min_dist = dist
                min_id = idx
        return self.nodes[min_id], min_id, min_dist

    def steer(self, qr, qn, val, eps):
        new_coord = {'x': 0, 'y': 0}
        if val >= eps:
            new_coord['x'] = qn.coord['x'] + ((qr.coord['x'] - qn.coord['x']) * eps) / self.node_dist(qr, qn)
            new_coord['y'] = qn.coord['y'] + ((qr.coord['y'] - qn.coord['y']) * eps) / self.node_dist(qr, qn)
        else:
            new_coord['x'] = qr.coord['x']
            new_coord['y'] = qr.coord['y']

        cost = self.node_dist(Node(new_coord), qn) + qn.cost
        return Node(new_coord, cost)

    def collision(self, p1, p2):
        def ccw(a, b, c):
            return (c['y'] - a['y']) * (b['x'] - a['x']) > (b['y'] - a['y']) * (c['x'] - a['x'])

        for o in self.obstacles:
            obs_x1 = o.x - o.r
            obs_y1 = o.y - o.r
            obs_x2 = o.x + o.r
            obs_y2 = o.y + o.r
            c1 = {'x': obs_x1, 'y': obs_y1}
            d1 = {'x': obs_x1, 'y': obs_y2}
            c2 = {'x': obs_x1, 'y': obs_y1}
            d2 = {'x': obs_x2, 'y': obs_y1}
            c3 = {'x': obs_x2, 'y': obs_y2}
            d3 = {'x': obs_x2, 'y': obs_y1}
            c4 = {'x': obs_x2, 'y': obs_y2}
            d4 = {'x': obs_x1, 'y': obs_y2}

            ints1 = ccw(p1.coord, c1, d1) != ccw(p2.coord, c1, d1) and ccw(p1.coord, p2.coord, c1) != ccw(p1.coord, p2.coord, d1)
            ints2 = ccw(p1.coord, c2, d2) != ccw(p2.coord, c2, d2) and ccw(p1.coord, p2.coord, c2) != ccw(p1.coord, p2.coord, d2)
            ints3 = ccw(p1.coord, c3, d3) != ccw(p2.coord, c3, d3) and ccw(p1.coord, p2.coord, c3) != ccw(p1.coord, p2.coord, d3)
            ints4 = ccw(p1.coord, c4, d4) != ccw(p2.coord, c4, d4) and ccw(p1.coord, p2.coord, c4) != ccw(p1.coord, p2.coord, d4)
            if ints1 != 0 or ints2 != 0 or ints3 != 0 or ints4 != 0:
                return True
        return False

    @staticmethod
    def node_dist(n1, n2):
        dx = n1.coord['x'] - n2.coord['x']
        dy = n1.coord['y'] - n2.coord['y']
        return (dx ** 2 + dy ** 2) ** .5

    def run(self, iter=600):
        self.nodes = [self.initial_position]
        for i in range(iter):
            q_rand = self.get_random_node()
            if self.check_if_reached():
                break

            q_near, _, near_dist = self.find_nearest_point_from_nodes(q_rand)
            q_new = self.steer(q_rand, q_near, near_dist, 20)

            if self.collision(q_rand, q_near):
                continue

            # find parent -------------------
            r = 60
            q_nearest = [n for n in self.nodes if not self.collision(n, q_new) and self.node_dist(n, q_new) <= r]
            q_min = q_near
            c_min = q_new.cost
            for n in q_nearest:
                if not self.collision(n, q_new) and n.cost + self.node_dist(n, q_new) < c_min:
                    q_min = n
                    c_min = n.cost + self.node_dist(n, q_new)
            for j, n in enumerate(self.nodes):
                if n.coord == q_min.coord:
                    q_new.parent = j
            # -------------------------------

            self.nodes.append(q_new)
        self.generate_final_course()

    def generate_final_course(self):
        q_final, idx, _ = self.find_nearest_point_from_nodes(self.selected_objective)
        self.selected_objective.parent = idx
        self.nodes.append(self.selected_objective)

        self.planned_path = []
        current = self.selected_objective
        while current.parent != 0:
            self.planned_path.append(current)
            current = self.nodes[current.parent]
        self.planned_path.append(current)

    def get_drawable_lines(self):
        lines = []
        for p in self.nodes:
            p_parent = self.nodes[p.parent] if p.parent is not None and p.parent >= 0 else self.selected_objective
            lines.append({
                'x1': p.coord['x'],
                'y1': p.coord['y'],
                'x2': p_parent.coord['x'],
                'y2': p_parent.coord['y'],
                'is_red': p.parent is not None and p.parent >= 0
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
