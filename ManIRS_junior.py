import os

os.add_dll_directory(r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu')
import b0RemoteApi
from RoboFunctions import ManRobot
import time
from typing import Optional
from collections import deque
from math import isclose


class Simulation:
    def __init__(self):
        self.robot: Optional[ManRobot] = None
        self._do_next_step = True
        self.init = self.simulation_step_started = self.simulation_step_done = self.cleanup = None
        self.on_init(lambda: None)
        self.on_step_started(lambda msg: None)
        self.on_step_done(lambda msg: None)
        self.on_cleanup(lambda: None)

    def on_init(self, callback):
        self.init = callback

    def on_step_started(self, callback):
        self.simulation_step_started = callback

    def on_step_done(self, callback):
        def callback_and_sync():
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
    def __init__(self, robot: ManRobot):
        self._x = self._z = 0
        self.tasks = deque()

    def receive_mission(self, mission):
        self.mission = mission

    def update(self):
        pass


def task_move():
    def mission():
        task_move()

    def complete():
        pass


def move_pos(robot: ManRobot, x, y, z):
    robot.setPositions(x, y, z)

    def wait():
        if isclose(robot.X_enc, x) and \
                isclose(robot.Y_enc, y) and \
                isclose(robot.Z_enc, z):
            return True
        return False

    return wait


def mission(robot: ManRobot):
    yield move_pos(robot, 100, 100, 100)
    yield move_pos(robot, 100, 100, 0)
    yield move_pos(robot, 0, 100, 100)
    yield move_pos(robot, 0, 100, 0)


def main():
    sim = Simulation()
    autobot = RobotController(sim.robot)

    @sim.on_init
    def init():
        autobot.receive_mission(mission)
        sim.robot.setPositions(100, 100, 100)

    @sim.on_step_started
    def step_start(msg):
        pass

    sim.start()


if __name__ == '__main__':
    main()
