from orbiter_physics import OrbiterDrawablePhysic
from ai_pp import *


class Orbiter(OrbiterDrawablePhysic):
    def __init__(self, method, init_pos=None):
        OrbiterDrawablePhysic.__init__(self, init_pos)
        self.error_prior = 0
        self.integral = 0
        self._ai_map = []
        self.path_planning_function = None
        self.change_path_planning_method(method)

    def change_path_planning_method(self, method):
        if method == 'rrt':
            self.path_planning_function = rrt_path_planning
        elif method == 'rrts':
            self.path_planning_function = rrt_star_path_planning
        elif method == 'astar':
            self.path_planning_function = a_star_path_planning
        else:
            self.path_planning_function = simple_path_planning

    def cycle(self, objectives, obstacles, force_repath=False, run_physics=True):
        self.actions, self._ai_map, reached = self.path_planning_function(self, objectives, obstacles, force_repath)

        if not run_physics:
            return False

        self.cycle_physic()
        return reached or self.collision_check(objectives, obstacles)

    def ai_map(self):
        return self._ai_map
