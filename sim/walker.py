#!/usr/bin/env python
import time
from threading import Thread
import math
import pybullet as p
from op3 import OP3


class WJFunc:
    """
    Walk Joint Function CPG style
    Provides parameterized sine wave functions as y=offset+scale*(in_offset+in_scale*x)
    """

    def __init__(self):
        self.offset = 0
        self.scale = 1
        self.in_offset = 0
        self.in_scale = 1

    def get(self, x):
        """ x between 0 and 1"""
        f = math.sin(self.in_offset + self.in_scale * x)
        return self.offset + self.scale * f

    def clone(self):
        z = WJFunc()
        z.offset = self.offset
        z.scale = self.scale
        z.in_offset = self.in_offset
        z.in_scale = self.in_scale
        return z

    def mirror(self):
        z = self.clone()
        z.offset *= -1
        z.scale *= -1
        return z

    def update_param_th(self):
        def _update_param():
            while True:
                for k, v in self.param_sliders.items():
                    self.parameters[k] = -p.readUserDebugParameter(v)
                for k, v in self.ang_sliders.items():
                    offset = p.readUserDebugParameter(v)
                    self.ang_offsets["l_" + k] = offset
                    self.ang_offsets["r_" + k] = -offset

                time.sleep(0.001)
        Thread(target=_update_param).start()


    def __str__(self):
        return "y=%f+%f*sin(%f+%f*x)" % (self.offset, self.scale, self.in_offset, self.in_scale)


class WFunc:
    """
    Multi-joint walk function for Darwin
    """

    def __init__(self, **kwargs):
        walk_offset = {'hip_pitch': -0.063,
                       'hip_roll': 0.0,
                       'hip_yaw': 0.0,
                       'ank_pitch': 0.0,
                       'ank_roll': 0.0,
                       'knee': 0.0}

        self.parameters = {"swing_scale": 0.0,
                            "step_scale": 0.1,
                            "step_offset": 0.25,
                            "ankle_offset": 0.0,
                            "vx_scale": 0.2,
                            "vy_scale": 0.2,
                            "vt_scale": 0.1}
        self.ang_offsets = {}
        self.param_sliders = {}
        self.ang_sliders = {}
        for k, v in self.parameters.items():
            self.param_sliders[k] = p.addUserDebugParameter(k, -3, 3, v)
        for k, v in walk_offset.items():
            self.ang_sliders[k] = p.addUserDebugParameter(k, -3, 3, v)
        self.update_param_th()
        while not self.ang_offsets:
            print("wait parameters to propagated")
        for k, v in kwargs.items():
            self.parameters[k] = v
        self.generate_init()

    def update_param_th(self):
        def _update_param():
            while True:
                for k, v in self.param_sliders.items():
                    self.parameters[k] = -p.readUserDebugParameter(v)
                for k, v in self.ang_sliders.items():
                    offset = p.readUserDebugParameter(v)
                    self.ang_offsets["l_" + k] = offset
                    self.ang_offsets["r_" + k] = -offset

                time.sleep(0.001)
        Thread(target=_update_param).start()

    def generate_init(self):
        """
        Build CPG functions for walk-on-spot (no translation or rotation, only legs up/down)
        """
        # f1=THIGH1=ANKLE1=L=R in phase
        self.pfn = {}  # phase joint functions
        self.afn = {}  # anti phase joint functions

        # ~ print f
        f1 = WJFunc()
        f1.in_scale = math.pi
        f1.scale = -self.parameters["swing_scale"]
        self.pfn["l_ank_roll"] = f1
        self.pfn["l_hip_roll"] = f1

        # f2=mirror f1 in antiphase
        f2 = f1.mirror()
        # ~ f2=WJFunc()
        self.afn["l_ank_roll"] = f2
        self.afn["l_hip_roll"] = f2

        f3 = WJFunc()
        f3.in_scale = math.pi
        f3.scale = self.parameters["step_scale"]
        f3.offset = self.parameters["step_offset"]
        self.pfn["l_hip_pitch"] = f3
        f33 = f3.mirror()
        f33.offset += self.parameters["ankle_offset"]
        self.pfn["l_ank_pitch"] = f33

        f4 = f3.mirror()
        f4.offset *= 2
        f4.scale *= 2
        self.pfn["l_knee"] = f4

        f5 = f3.clone()
        f5.in_scale *= 2
        f5.scale = 0
        self.afn["l_hip_pitch"] = f5

        f6 = f3.mirror()
        f6.in_scale *= 2
        f6.scale = f5.scale
        f6.offset += self.parameters["ankle_offset"]
        self.afn["l_ank_pitch"] = f6

        f7 = f4.clone()
        f7.scale = 0
        self.afn["l_knee"] = f7

        self.forward = [f5, f6]

        self.generate_right_init()
        self.joints = self.pfn.keys()

        self.show()

    def generate_right_init(self):
        """
        Mirror CPG functions from left to right and antiphase right
        """
        l = [v[2:] for v in self.pfn.keys()]
        for j in l:
            self.pfn["r_" + j] = self.afn["l_" + j].mirror()
            self.afn["r_" + j] = self.pfn["l_" + j].mirror()

    def generate(self):
        # ~ print f
        f1 = self.pfn["l_ank_roll"]
        f1.scale = -self.parameters["swing_scale"]
        self.pfn["l_hip_roll"] = f1

        # f2=mirror f1 in antiphase
        f1.mirror_to(self.afn["l_ank_roll"])
        f1.mirror_to(self.afn["l_hip_roll"])
        # ~ f2=WJFunc()

        f3 = self.pfn["l_hip_pitch"]
        f3.scale = self.parameters["step_scale"]
        f3.offset = self.parameters["step_offset"]

        f33 = self.pfn["l_ank_pitch"]
        f3.mirror_to(f33)
        f33.offset += self.parameters["ankle_offset"]

        f4 = self.pfn["l_knee"]
        f3.mirror_to(f4)
        f4.offset *= 2
        f4.scale *= 2

        f5 = self.afn["l_hip_pitch"]
        f3.copy_to(f5)
        f5.in_scale *= 2
        f5.scale = 0

        f6 = self.afn["l_ank_pitch"]
        f3.mirror_to(f6)
        f6.in_scale *= 2
        f6.scale = f5.scale
        f6.offset += self.parameters["ankle_offset"]

        f7 = self.afn["l_knee"]
        f4.copy_to(f7)
        f7.scale = 0

        self.generate_right()

    def generate_right(self):
        """
        Mirror CPG functions from left to right and antiphase right
        """
        l = [v[2:] for v in self.pfn.keys()]
        for j in l:
            self.afn["l_" + j].mirror_to(self.pfn["r_" + j])
            self.pfn["l_" + j].mirror_to(self.afn["r_" + j])

    def get(self, phase, x, velocity):
        """ Obtain the joint angles for a given phase, position in cycle (x 0,1)) and velocity parameters """
        self.generate()
        angles = {}
        for j in self.pfn.keys():
            if phase:
                angles[j] = self.ang_offsets[j] + self.pfn[j].get(x)
            else:
                angles[j] = self.ang_offsets[j] + self.afn[j].get(x)
        self.apply_velocity(angles, velocity, phase, x)
        return angles

    def show(self):
        """
        Display the CPG functions used
        """
        for j in self.pfn.keys():
            print(j, "p", self.pfn[j], "a", self.afn[j])

    def apply_velocity(self, angles, velocity, phase, x):
        """ Modify on the walk-on-spot joint angles to apply the velocity vector"""

        # VX
        v = velocity[0] * self.parameters["vx_scale"]
        # print(velocity)
        d = (x * 2 - 1) * v
        if phase:
            angles["l_hip_pitch"] += d
            angles["l_ank_pitch"] += d
            angles["r_hip_pitch"] += d
            angles["r_ank_pitch"] += d
        else:
            angles["l_hip_pitch"] -= d
            angles["l_ank_pitch"] -= d
            angles["r_hip_pitch"] -= d
            angles["r_ank_pitch"] -= d

        # VY
        v = velocity[1] * self.parameters["vy_scale"]
        d = (x) * v
        d2 = (1 - x) * v
        if v >= 0:
            if phase:
                angles["l_hip_roll"] -= d
                angles["l_ank_roll"] -= d
                angles["r_hip_roll"] += d
                angles["r_ank_roll"] += d
            else:
                angles["l_hip_roll"] -= d2
                angles["l_ank_roll"] -= d2
                angles["r_hip_roll"] += d2
                angles["r_ank_roll"] += d2
        else:
            if phase:
                angles["l_hip_roll"] += d2
                angles["l_ank_roll"] += d2
                angles["r_hip_roll"] -= d2
                angles["r_ank_roll"] -= d2
            else:
                angles["l_hip_roll"] += d
                angles["l_ank_roll"] += d
                angles["r_hip_roll"] -= d
                angles["r_ank_roll"] -= d

        # VT
        v = velocity[2] * self.parameters["vt_scale"]
        d = (x) * v
        d2 = (1 - x) * v
        if v >= 0:
            if phase:
                angles["l_hip_yaw"] = -d
                angles["r_hip_yaw"] = d
            else:
                angles["l_hip_yaw"] = -d2
                angles["r_hip_yaw"] = d2
        else:
            if phase:
                angles["l_hip_yaw"] = d2
                angles["r_hip_yaw"] = -d2
            else:
                angles["l_hip_yaw"] = d
                angles["r_hip_yaw"] = -d

class Walker:
    """
    Class for making Darwin walk
    """

    def __init__(self, op3):
        self.op3 = op3
        self.running = False

        self.velocity = [0, 0, 0]
        self.walking = False
        self.func = WFunc()

        # ~ self.ready_pos=get_walk_angles(10)
        self.ready_pos = self.func.get(True, 0, [0, 0, 0])
        self._th_walk = None

    def cmd_vel(self, vx, vy, vt):
        print("cmdvel", (vx, vy, vt))
        self.start()
        self.set_velocity(vx, vy, vt)

    def init_walk(self):
        """
        If not there yet, go to initial walk position
        """
        if self.get_dist_to_ready() > 0.02:
            self.updateready_pos()
            self.op3.set_angles_slow(self.ready_pos)


    def start(self):
        if not self.running:
            self.running = True
            self.init_walk()
            self._th_walk = Thread(target=self._do_walk)
            self._th_walk.start()
            self.walking = True

    def stop(self):
        if self.running:
            self.walking = False
            self.running = False

    def set_velocity(self, x, y, t):
        self.velocity = [x, y, t]

    def _do_walk(self):
        """
        Main walking loop, smoothly update velocity vectors and apply corresponding angles
        """
        func = self.func

        # Global walk loop
        n = 50
        p = True
        i = 0
        self.current_velocity = [0, 0, 0]
        while self.walking or i < n or self.is_walking():
            if not self.walking:
                self.velocity = [0, 0, 0]
            if not self.is_walking() and i == 0:  # Do not move if nothing to do and already at 0
                self.update_velocity(self.velocity, n)
                time.sleep(1. / 240.)
                continue
            x = float(i) / n
            angles = func.get(p, x, self.current_velocity)
            self.update_velocity(self.velocity, n)
            self.op3.set_angles(angles)
            i += 1
            if i > n:
                i = 0
                p = not p
            time.sleep(1. / 240.)

        self._th_walk = None

    def updateready_pos(self):
        self.ready_pos['l_sho_roll'] = 0.8
        self.ready_pos['r_sho_roll'] = -0.8
        self.ready_pos['l_el'] = -0.7
        self.ready_pos['r_el'] = 0.7

    def is_walking(self):
        e = 0.02
        for v in self.current_velocity:
            if abs(v) > e: return True
        return False

    def rescale(self, angles, coef):
        z = {}
        for j, v in angles.items():
            offset = self.ready_pos[j]
            v -= offset
            v *= coef
            v += offset
            z[j] = v
        return z

    def update_velocity(self, target, n):
        a = 3 / float(n)
        b = 1 - a
        self.current_velocity = [a * t + b * v for (t, v) in zip(target, self.current_velocity)]

    def get_dist_to_ready(self):
        angles = self.op3.get_angles()
        return get_distance(self.ready_pos, angles)


def interpolate(anglesa, anglesb, coefa):
    z = {}
    joints = anglesa.keys()
    for j in joints:
        z[j] = anglesa[j] * coefa + anglesb[j] * (1 - coefa)
    return z


def get_distance(anglesa, anglesb):
    d = 0
    joints = anglesa.keys()
    if len(joints) == 0: return 0
    for j in joints:
        d += abs(anglesb[j] - anglesa[j])
    d /= len(joints)
    return d


if __name__ == "__main__":

    op3 = OP3()
    walker = Walker(op3)
    time.sleep(1)
    walker.start()
    walker.set_velocity(1, 0, 0)

    while True:
        time.sleep(0.5)
