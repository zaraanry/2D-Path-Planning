import math
from random import random
from config import *

PI = math.pi


class OrbiterPhysic:
    def __init__(self, init_pos):
        if init_pos is None:
            self.x = random() * 400 - 200
            self.y = random() * 400 - 200
            self.r = random() * 6.
            self.radius = 5
        else:
            self.x = init_pos.get('x', 0)
            self.y = init_pos.get('y', 0)
            self.r = init_pos.get('r', 0)
            self.radius = init_pos.get('radius', 5)

        self.actions = {'vx': 0, 'vy': 0, 'vr': 0}

    @staticmethod
    def clip(val, mx, mn=0):
        if val > mx:
            return mx
        if val < mn:
            return mn
        return val

    # @staticmethod
    # def angular_diff(t1, t2, thr):
    #     d = OrbiterPhysic.angle(OrbiterPhysic.angle(t1) - OrbiterPhysic.angle(t2))
    #     return (d < thr) or (d > (pi2 - thr))
    #
    # @staticmethod
    # def angle(val):
    #     if val > PI*2:
    #         return val - PI*2
    #     if val < 0:
    #         return val + PI*2
    #     return val

    def cycle_physic(self):
        vx = self.clip(self.actions['vx'], mn=-ORBITER_VX_MAX, mx=ORBITER_VX_MAX) * ORBITER_MAX_SPEED
        vy = self.clip(self.actions['vy'], mn=-ORBITER_VY_MAX, mx=ORBITER_VY_MAX) * ORBITER_MAX_SPEED
        vr = self.clip(self.actions['vr'], mn=-ORBITER_VR_MAX, mx=ORBITER_VR_MAX)

        # TODO implement VR

        self.x += vx
        self.y += vy
        self.x = self.clip(self.x, mn=-FIELD_MAX, mx=FIELD_MAX)
        self.y = self.clip(self.y, mn=-FIELD_MAX, mx=FIELD_MAX)

    def collision_check(self, objectives, obstacles):
        # TODO implement other robot collision
        for o in obstacles:
            if self.intersect_circle_and_rect(self.x, self.y, self.radius, o.x, o.y, o.r*2, o.r*2):
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


class OrbiterDrawablePhysic(OrbiterPhysic):
    def __init__(self, init_pos):
        OrbiterPhysic.__init__(self, init_pos)
    
    def to_json(self):
        color = '#ff0'
        radius = self.radius

        # m1p1x = radius * math.cos(self.r + PI / 2);
        # m1p1y = radius * math.sin(self.r + PI / 2);
        # m1p2x = m1p1x + 8. * math.cos(self.r);
        # m1p2y = m1p1y + 8. * math.sin(self.r);
        #
        # m2p1x = radius * math.cos(self.r - PI / 2);
        # m2p1y = radius * math.sin(self.r - PI / 2);
        # m2p2x = m2p1x + 8. * math.cos(self.r);
        # m2p2y = m2p1y + 8. * math.sin(self.r);
        #
        # m3p1x = 3. * math.cos(self.r + PI);
        # m3p1y = 3. * math.sin(self.r + PI);
        # m3p2x = m3p1x + 8. * math.cos(self.r + PI);
        # m3p2y = m3p1y + 8. * math.sin(self.r + PI);

        return [
            # Body
            {
                'shape': 'ellipse',
                'color': color,
                'fill': True,

                'x': self.x,
                'y': self.y,
                'rx': radius,
                'ry': radius,
            },
            {
                'shape': 'ellipse',
                'color': '#000',
                'fill': False,

                'x': self.x,
                'y': self.y,
                'rx': radius+1,
                'ry': radius+1,
            },

            # # Motor 1
            # {
            #     'shape': 'lines',
            #     'color': color,
            #     'fill': False,
            #     'points': [
            #         {'x': self.x + m1p1x, 'y': self.y + m1p1y},
            #         {'x': self.x + m1p2x, 'y': self.y + m1p2y},
            #     ]
            # },
            #
            # # Motor 2
            # {
            #     'shape': 'lines',
            #     'color': color,
            #     'fill': False,
            #     'points': [
            #         {'x': self.x + m2p1x, 'y': self.y + m2p1y},
            #         {'x': self.x + m2p2x, 'y': self.y + m2p2y},
            #     ]
            # },
            #
            # # Motor 3
            # {
            #     'shape': 'lines',
            #     'color': color,
            #     'fill': False,
            #     'points': [
            #         {'x': self.x + m3p1x, 'y': self.y + m3p1y},
            #         {'x': self.x + m3p2x, 'y': self.y + m3p2y},
            #     ]
            # },
        ]
