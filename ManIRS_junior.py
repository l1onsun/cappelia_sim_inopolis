import os

os.add_dll_directory(r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu')
import b0RemoteApi
from RoboFunctions import ManRobot
from typing import Optional
from math import isclose
import time


class SyncSimulation:
    def __init__(self):
        self.robot: Optional[ManRobot] = None
        self._do_next_step = True
        self.init = self.simulation_step_started = self.simulation_step_done = self.cleanup = None
        self.on_init(lambda: None)
        self.on_step_started(lambda: None)
        self.on_step_done(lambda: None)
        self.on_cleanup(lambda: None)

    def on_init(self, callback):
        self.init = callback

    def on_step_started(self, callback):
        self.simulation_step_started = lambda msg: callback()

    def on_step_done(self, callback):
        def callback_and_sync(msg):
            callback()
            self._do_next_step = True

        self.simulation_step_done = callback_and_sync

    def on_cleanup(self, callback):
        self.cleanup = callback

    def start(self):
        with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient', 'b0RemoteApi_manirs') as client:
            self.robot = ManRobot(client)

            client.simxSynchronous(True)
            client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(self.simulation_step_started))
            client.simxGetSimulationStepDone(client.simxDefaultSubscriber(self.simulation_step_done))
            client.simxStartSimulation(client.simxDefaultPublisher())
            self.init()
            while not self.robot.disconnect and self.robot.simTime < 120:
                if self._do_next_step:
                    self._do_next_step = False
                    client.simxSynchronousTrigger()
                client.simxSpinOnce()
            self.cleanup()
            if not self.robot.disconnect:
                client.simxStopSimulation(client.simxDefaultPublisher())
            else:
                print('Simulation was stopped and client was disconnected!')


class RobotController:
    def __init__(self):
        self.wait_for_completion = None
        self.mission_generator = None

    def start_mission(self, mission_generator):
        self.mission_generator = mission_generator
        self.wait_for_completion = next(self.mission_generator)

    def next_mission(self):
        result = next(self.mission_generator)
        if result == "complete":
            self.wait_for_completion = self.mission_generator = None
        else:
            self.wait_for_completion = result

    def update(self):
        if self.wait_for_completion is not None:
            if self.wait_for_completion():
                self.wait_for_completion = next(self.mission_generator)


class Evangelion:
    def __init__(self, robot: ManRobot):
        self.robot = robot
        self._x_pos = 0
        self._y_pos = 0
        self._z_pos = 0

    def wait_one_step(self):
        return lambda: True

    def wait_time(self, seconds):
        start = time.time()
        return lambda: (time.time() >= start + seconds)

    def release(self):
        self.robot.releaseObject()
        yield self.wait_one_step()

    def release_and_grab(self):
        yield from self.release()
        self.robot.grapObject()
        yield self.wait_one_step()

    def move_pos(self):
        self.robot.setPositions(self._x_pos, self._y_pos, self._z_pos)

        def wait_for_completion():
            if isclose(self.robot.X_enc, self._x_pos, abs_tol=1) and \
                    isclose(self.robot.Y_enc, self._y_pos, abs_tol=1) and \
                    isclose(self.robot.Z_enc, self._z_pos, abs_tol=1):
                return True
            return False

        return wait_for_completion

    def shift(self, x, z):
        self._x_pos = 21 + 37 * x
        self._z_pos = 21 + 37 * z
        return self.move_pos()

    def rise(self, y):
        self._y_pos = y
        return self.move_pos()

    def move_to(self, x, y):
        yield self.rise(50)
        yield self.shift(x, y)

    def grab_to(self, x, y, height = 40):
        yield self.rise(height)
        yield from self.release_and_grab()
        yield self.rise(80)
        yield self.shift(x, y)
        yield self.rise(height)
        yield from self.release()

    def get_color(self):
        r, g, b = self.img_pix(128, 200)
        if r > g and r > b:
            return "red"
        elif g > r and g > b:
            return "green"
        return "blue"

    def get_shape(self):
        brightness = sum(self.img_pix(64, 170))
        if brightness > 750:
            return "ball"
        return "cube"

    def get_cube_type(self):
        start_color = None
        for i in range(256):
            r, g, b = self.img_pix(i, 255)
            if sum([r, g, b]) > 750:
                continue
            if not start_color:
                start_color = (r, g, b)
            if (r, g, b) != start_color:
                return "hollow"

        return "normal"

    def get_full_info(self):
        color = self.get_color()
        shape = self.get_shape()
        if shape == "cube":
            cube_type = self.get_cube_type()
            if cube_type == "hollow":
                shape = "hollow_cube"
        return color, shape

    def print_color(self, aditional_info=""):
        print("info: ", aditional_info)
        print(self.get_full_info())
        return self.wait_time(0.1)
        # self.robot.

    def img_pix(self, x, y):
        i = 3 * (256 * y + x)
        return self.robot.cam_image[i: i + 3]

    def complete(self):
        yield self.rise(50)
        self._x_pos = 0
        self._z_pos = 0
        yield self.move_pos()
        self._y_pos = 0
        yield self.move_pos()
        yield "complete"

def color_shape_to_position(color, shape):
    x = z = height = None
    if color == "red":
        x = 1
    elif color == "green":
        x = 2
    elif color == "blue":
        x = 3
    if shape == "cube":
        z = 1
        height = 40
    elif shape == "hollow_cube":
        z = 2
        height = 32
    elif shape == "ball":
        z = 3
        height = 35
    return x, z, height

# 40, 35(ball), 32(hollow)

def mission_y(robot: ManRobot):
    ev = Evangelion(robot)
    for i in range(1, 4):
        yield from ev.move_to(i, 4)
        color, shape = ev.get_full_info()
        required_x, required_z, height = color_shape_to_position(color, shape)
        yield from ev.grab_to(required_x, required_z, height)

    yield from ev.complete()


if __name__ == '__main__':
    sim = SyncSimulation()
    autobot = RobotController()


    @sim.on_init
    def init():
        autobot.start_mission(
            mission_y(sim.robot)
        )


    @sim.on_step_started
    def step_start():
        autobot.update()


    sim.start()