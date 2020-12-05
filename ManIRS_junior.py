import os

os.add_dll_directory(r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu')
import b0RemoteApi
from RoboFunctions import ManRobot
from typing import Optional


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


sim = SyncSimulation()


@sim.on_init
def init():
    pass


@sim.on_step_started
def step_start():
    pass


@sim.on_step_done
def step_end():
    pass


@sim.on_cleanup
def cleanup():
    pass


sim.start()
