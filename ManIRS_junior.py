import os

os.add_dll_directory(r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu')
import b0RemoteApi
from RoboFunctions import ManRobot
from typing import Optional
from math import isclose


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


def move_pos(robot: ManRobot, x, y, z):
    robot.setPositions(x, y, z)

    def wait_for_completion():
        if isclose(robot.X_enc, x, abs_tol=1) and \
                isclose(robot.Y_enc, y, abs_tol=1) and \
                isclose(robot.Z_enc, z, abs_tol=1):
            return True
        return False

    return wait_for_completion


def mission_x(robot: ManRobot):
    yield move_pos(robot, 100, 100, 100)
    yield move_pos(robot, 100, 100, 0)
    yield move_pos(robot, 0, 100, 100)
    yield move_pos(robot, 0, 100, 0)


if __name__ == '__main__':
    sim = SyncSimulation()
    autobot = RobotController()


    @sim.on_init
    def init():
        autobot.start_mission(
            mission_x(sim.robot)
        )


    @sim.on_step_started
    def step_start():
        autobot.update()


    sim.start()
