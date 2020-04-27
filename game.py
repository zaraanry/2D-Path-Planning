
from orbiter import Orbiter
from random import randint
from config import *


class Objective:
    def __init__(self, x, y, r=20):
        self.x = x
        self.y = y
        self.r = r

    @staticmethod
    def gen_random_list(n=1):
        return [Objective(randint(-FIELD_MAX, FIELD_MAX), randint(-FIELD_MAX, FIELD_MAX)) for _ in range(n)]
    
    def to_json(self):
        return [{
            'shape': 'ellipse',
            'color': '#eee',
            'fill': True,

            'x': self.x,
            'y': self.y,
            'rx': self.r,
            'ry': self.r,
        }]


class Obstacle:
    def __init__(self, x, y, r=10):
        self.x = x
        self.y = y
        self.r = r

    @staticmethod
    def gen_random_list(n=10, size=None, invalid_places=None):
        def is_invalid_position(_x, _y, _r):
            for p in invalid_places:
                dx = _x - p.x
                dy = _y - p.y
                dist = (dx ** 2 + dy ** 2) ** .5
                pr = p.r if p.r > 10 else 10
                if dist < _r + pr:
                    return True
            return False

        if invalid_places is None:
            invalid_places = []
        result = []
        while len(result) < n:
            r = randint(10, 20) if size is None else size
            x, y = randint(-FIELD_MAX, FIELD_MAX), randint(-FIELD_MAX, FIELD_MAX)

            if is_invalid_position(x, y, r):
                continue

            result.append(Obstacle(x, y, r))
        return result

    def to_json(self):
        return [{
            'shape': 'rect',
            'color': '#000',
            'fill': True,

            'x': self.x-self.r,
            'y': self.y-self.r,
            'w': self.r*2,
            'h': self.r*2,
        }]


class GameEngine:
    def __init__(self):
        self.debug_draw = []
        self.frame = 0
        self.reset_count_down = -1
        self.objectives = []
        self.obstacles = []
        self.orbiters = []
        self.num_of_obstacles = 20
        self.path_planning_method = 'simple'
        self.obstacles_recreation_countdown = -1
        self.test_number = 0

        self.init()

    def set_params(self, num_of_obstacles, path_planning_method):
        # return
        if num_of_obstacles != self.num_of_obstacles:
            self.obstacles_recreation_countdown = 4
        elif self.obstacles_recreation_countdown > -1:
            self.obstacles_recreation_countdown -= 1
        if self.obstacles_recreation_countdown == 0:
            if len(self.obstacles) < num_of_obstacles:
                self.obstacles.extend(Obstacle.gen_random_list(n=num_of_obstacles-len(self.obstacles), invalid_places=self.objectives + self.orbiters))
            while len(self.obstacles) > num_of_obstacles:
                self.obstacles.pop(randint(0, len(self.obstacles)-1))

        if self.path_planning_method != path_planning_method:
            for o in self.orbiters:
                o.change_path_planning_method(path_planning_method)
                o.cycle(objectives=self.objectives, obstacles=self.obstacles, force_repath=True, run_physics=False)

        self.num_of_obstacles = num_of_obstacles
        self.path_planning_method = path_planning_method

    def init(self):
        self.frame = 0
        self.obstacles_recreation_countdown = -1
        self.objectives = Objective.gen_random_list()
        self.orbiters = [Orbiter(method=self.path_planning_method)]
        self.obstacles = Obstacle.gen_random_list(n=self.num_of_obstacles, invalid_places=self.objectives + self.orbiters)

    def test_prep(self):
        return
        import math
        obj_r = self.objectives[0].r
        dx = self.objectives[0].x - self.orbiters[0].x
        dy = self.objectives[0].y - self.orbiters[0].y
        is_failed = not(math.fabs(dx) < obj_r*.6 and math.fabs(dy) < obj_r*.6)
        print 'TTT, %d, %s, %d, %d, %d' % (self.test_number, self.path_planning_method, self.num_of_obstacles, self.frame, (0 if is_failed else 1))
        print '----------------------------------'
        if self.test_number == 0:
            self.num_of_obstacles = 20
            self.path_planning_method = 'simple'
            self.test_number += 1
        elif self.test_number == 1:
            self.num_of_obstacles = 20
            self.path_planning_method = 'rrt'
            self.test_number += 1
        elif self.test_number == 2:
            self.num_of_obstacles = 20
            self.path_planning_method = 'rrts'
            self.test_number += 1
        elif self.test_number == 3:
            self.num_of_obstacles = 20
            self.path_planning_method = 'astar'
            self.test_number += 1

        elif self.test_number == 4:
            self.num_of_obstacles = 30
            self.path_planning_method = 'simple'
            self.test_number += 1
        elif self.test_number == 5:
            self.num_of_obstacles = 30
            self.path_planning_method = 'rrt'
            self.test_number += 1
        elif self.test_number == 6:
            self.num_of_obstacles = 30
            self.path_planning_method = 'rrts'
            self.test_number += 1
        elif self.test_number == 7:
            self.num_of_obstacles = 30
            self.path_planning_method = 'astar'
            self.test_number += 1

        elif self.test_number == 8:
            self.num_of_obstacles = 40
            self.path_planning_method = 'simple'
            self.test_number += 1
        elif self.test_number == 9:
            self.num_of_obstacles = 40
            self.path_planning_method = 'rrt'
            self.test_number += 1
        elif self.test_number == 10:
            self.num_of_obstacles = 40
            self.path_planning_method = 'rrts'
            self.test_number += 1
        elif self.test_number == 11:
            self.num_of_obstacles = 40
            self.path_planning_method = 'astar'
            self.test_number += 1

        elif self.test_number == 12:
            self.num_of_obstacles = 10
            self.path_planning_method = 'simple'
            self.test_number += 1
        elif self.test_number == 13:
            self.num_of_obstacles = 10
            self.path_planning_method = 'rrt'
            self.test_number += 1
        elif self.test_number == 14:
            self.num_of_obstacles = 10
            self.path_planning_method = 'rrts'
            self.test_number += 1
        elif self.test_number == 15:
            self.num_of_obstacles = 10
            self.path_planning_method = 'astar'
            self.test_number = 0

    def cycle(self):
        if self.reset_count_down > 0:
            self.reset_count_down -= 1
            return
        elif self.reset_count_down == 0:
            self.test_prep()
            self.init()
            self.reset_count_down -= 1

        self.frame += 1
        all_reached = True
        for o in self.orbiters:
            reached = o.cycle(objectives=self.objectives, obstacles=self.obstacles)
            all_reached = all_reached and reached

        if all_reached:
            self.reset_count_down = 5

    def to_json(self):
        drawable = []
        for o in self.objectives:
            drawable.extend(o.to_json())
        for o in self.obstacles:
            drawable.extend(o.to_json())
        for o in self.orbiters:
            drawable.extend(o.ai_map())
        for o in self.orbiters:
            drawable.extend(o.to_json())

        drawable.extend(self.debug_draw)

        return {
            'center_draw': True,
            'extra_info': {'counter': self.frame},
            'drawable': drawable,
        }
