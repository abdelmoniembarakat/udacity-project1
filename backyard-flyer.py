from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
import time
import numpy as np
from enum import Enum


class phases(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class up_and_down(Drone):
    def __init__(self, connection):
        super().__init__(connection)
        self.in_mission = True
        self.flight_phase = phases.MANUAL
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.local_velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_phase == phases.TAKEOFF:
            altitude = -1 * self.local_position[2]
            if altitude > 0.95 * self.target_position[2]:
                self.waypoint_transition()

    def local_velocity_callback(self):
        if self.flight_phase == phases.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and abs(self.local_position[2] < 0.01)):
                self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_phase == phases.MANUAL:
            self.arming_transition()
        elif self.flight_phase == phases.ARMING:
            self.takeoff_transition()
        elif self.flight_phase == phases.WAYPOINT:
            self.landing_transition()
        elif self.flight_phase == phases.DISARMING:
            self.manual_transition()


    def arming_transition(self):
        print('Arming transition')
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0], self.global_position[1], self.global_position[2])
        self.flight_phase = phases.ARMING

    def takeoff_transition(self):
        print('Takeoff transition')
        target_altitude = 5.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_phase = phases.TAKEOFF

    def waypoint_transition(self):
        print('Waypoint transition')
        square = [(5,0,3,0), (5,5,3,0), (0,5,3,0), (0,0,3,0)]
        for corner in square:
            self.cmd_position(*corner)
            time.sleep(2)
        self.flight_phase = phases.WAYPOINT



    def landing_transition(self):
        print('Landing transition')
        self.land()
        self.flight_phase = phases.LANDING

    def disarming_transition(self):
        print('Disarming transition')
        self.disarm()
        self.flight_phase = phases.DISARMING

    def manual_transition(self):
        print('Manual transition')
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_phase = phases.MANUAL



    def start(self):
        self.start_log('Logs', 'NavLog.txt')
        print('Connecting')
        super().start()
        self.stop_log()

if __name__ == '__main__':
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded = False, PX4 = False)
    drone = up_and_down(conn)
    time.sleep(4)
    drone.start()
