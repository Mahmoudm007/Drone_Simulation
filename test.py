import time
from enum import Enum
import numpy as np
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
import pandas as pd
import A_star


class Phases(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    LANDING = 3
    DISARMING = 4


class UpAndDownFlyer(Drone):

    def __init__(self, connection, path):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.in_mission = True
        self.flight_phase = Phases.MANUAL
        self.path = path
        self.current_waypoint = 0

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_phase == Phases.TAKEOFF or self.flight_phase == Phases.WAYPOINT:
            self.follow_path()

    def follow_path(self):
        if self.current_waypoint < len(self.path):
            target = self.path[self.current_waypoint]
            self.target_position = np.array([target[0], target[1], 3.0])  # Assuming constant altitude of 3.0

            distance = np.linalg.norm(self.local_position[:2] - self.target_position[:2])
            if distance < 0.5:  # Check if close to the target
                self.current_waypoint += 1
        else:
            self.landing_transition()

    # Existing methods...

    def takeoff_transition(self):
        print("takeoff transition")
        self.flight_phase = Phases.TAKEOFF
        self.follow_path()

    def velocity_callback(self):
        if self.flight_phase == Phases.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and
                    abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_phase == Phases.MANUAL:
            self.arming_transition()
        elif self.flight_phase == Phases.ARMING:
            if self.armed:
                self.takeoff_transition()
        elif self.flight_phase == Phases.DISARMING:
            if not self.armed:
                self.manual_transition()

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()

        # set the current location to be the home position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])

        self.flight_phase = Phases.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        self.flight_phase = Phases.TAKEOFF
        self.follow_path()

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_phase = Phases.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_phase = Phases.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_phase = Phases.MANUAL

    def start(self):
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        super().start()
        self.stop_log()


if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760',
                             threaded=False,
                             PX4=False)

    colliders_file_path = 'colliders.csv'

    # Extract relevant columns from the colliders DataFrame
    colliders_df = pd.read_csv(colliders_file_path, skiprows=1)

    # Convert relevant columns to float
    colliders_df = colliders_df.astype(float)

    # Define the size of the grid
    min_posX = colliders_df['posX'].min()
    max_posX = colliders_df['posX'].max()
    min_posY = colliders_df['posY'].min()
    max_posY = colliders_df['posY'].max()

    grid_size = (int(max_posX - min_posX) + 1, int(max_posY - min_posY) + 1)
    grid = np.zeros(grid_size)

    # Fill the grid with obstacles
    for index, obstacle in colliders_df.iterrows():
        x = int(obstacle['posX'] - min_posX)
        y = int(obstacle['posY'] - min_posY)
        z = int(obstacle['posZ'])  # Use original Z value for height
        half_size_x = int(obstacle['halfSizeX'])
        half_size_y = int(obstacle['halfSizeY'])

        grid[max(0, x - half_size_x):min(grid_size[0], x + half_size_x + 1),
             max(0, y - half_size_y):min(grid_size[1], y + half_size_y + 1)] = z

    # Scale the grid to 300x300 using nearest-neighbor interpolation
    scaled_grid_size = 100
    scale_x = scaled_grid_size / grid.shape[0]
    scale_y = scaled_grid_size / grid.shape[1]

    scaled_grid = np.zeros((scaled_grid_size, scaled_grid_size))

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            scaled_x = int(i * scale_x)
            scaled_y = int(j * scale_y)
            scaled_grid[scaled_x, scaled_y] = grid[i, j]

    a_star = A_star.AStarApp(scaled_grid, height_cost_weight=0.3)  # You can adjust the weight here

    drone = UpAndDownFlyer(conn, )
    time.sleep(2)
    drone.start()
