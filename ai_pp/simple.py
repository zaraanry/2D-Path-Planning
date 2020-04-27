import math


def simple_path_planning(self, objectives, obstacles, force_repath):
    x = self.x
    y = self.y

    obj_x = objectives[0].x
    obj_y = objectives[0].y
    obj_r = objectives[0].r

    dx = obj_x - x
    dy = obj_y - y
    max_delta = max(math.fabs(dx), math.fabs(dy))

    if math.fabs(dx) < obj_r*.6 and math.fabs(dy) < obj_r*.6:
        return {'vx': 0, 'vy': 0, 'vr': 0}, [], True

    dx /= max_delta
    dy /= max_delta

    return {'vx': dx, 'vy': dy, 'vr': 0}, [line_to(x, y, obj_x, obj_y)], False


def line_to(x1, y1, x2, y2):
    return {
        'shape': 'lines',
        'color': '#f00',
        'fill': False,
        'points': [
            {'x': x1, 'y': y1},
            {'x': x2, 'y': y2},
        ]
    }
