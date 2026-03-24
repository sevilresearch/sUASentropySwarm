"""
Created by Jason Carnahan  - 5/14/2021

Edited by Tara Moore - 2025

Edited by Juan José Molina - March 2026

Handles meta data manipulation and analytics. Keeps track of the drones in the system,
and handles interaction for the user
"""
import csv
import numpy as np
import os
from UAV import UAV
import pandas as pd
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.pyplot as plt
import time


class CentralControl:
    """
    Central Controler that
    """

    def __init__(self, strategy):
        print("[CC] CentralControl init")
        self.strategy = strategy  # Strategy to use
        self.uavs = []  # Storage for the UAVs
        self.Vmax = 1.25   # Max Speed of the UAVs
        self.maxDistance = 50  # Max Survey Distance between UAVs
        self.minDistance = 5  # Min physical distance between UAVs
        self.waypoints = [[25, 100], [75, 100], [75, 50], [25, 50]] # Arbitrary goal for the swarm
        #self.waypoints = [[25, 100], [75, 100], [75, 50], [25, 50]]
        # Stable base dir next to this file, regardless of CWD
        self._base_dir = os.path.dirname(__file__)
        self._runs_dir = os.path.join(self._base_dir, "data-files")
        os.makedirs(self._runs_dir, exist_ok=True)
        self._csv_path = os.path.join(self._runs_dir, "entropySave.csv")
       

        # For the structure of the UAV init
        self.num_of_uav = None
        self.sep_distance = None
        self.row_length = None


    def init_system(self, num_of_uavs, separating_distance, row_length):
        """
        Since all that is needed to add UAVs to the simulation is num_of_uavs, separating_distance, and row_length;
        the UAV positions will need to be calculated when added to the system
        """

        # Save the Structure
        self.num_of_uav = num_of_uavs
        self.sep_distance = separating_distance
        self.row_length = row_length

        # Init the structure
        i = 1
        row = 1
        col = 1

        while i <= num_of_uavs:
            if row > row_length:
                row = 1
                col += 1

            # Create the UAV
            self.add_uav(f"Drone{i}", separating_distance * -row + 100, 25 * col, -2)

            i += 1
            row += 1
        print(f"[CC] init_system: N={num_of_uavs}  sep={separating_distance}  rows={row_length}")


    def add_uav(self, ID, x, y, z):
        """
        Used apart of the initialization to add UAVs to the internal system.
        :param ID: Name of the UAV
        :param x: x coordinate
        :param y: y coordinate
        :param z: z coordinate
        """
        # Create a new UAV object with a new strategy class
        strategy_class = self.strategy.__class__
        copied_strategy = strategy_class()

        new_uav = UAV(ID, x, y, z, self.minDistance, self.maxDistance, self.Vmax, copied_strategy)
        new_uav.strategy.set_parent(new_uav)

        # Create a connection to the UAV
        new_uav.strategy.establish_connection(self)

        # Add it to the internal data
        self.uavs.append(new_uav)

    def connect_to_environment(self):
        """
        Establishes connection the environment of the UAVs
        """
        for uav in self.uavs:
            uav.strategy.connect_to_environment()

        return True

    def close_connection(self):
        """
        Disable connection to the UAVs. Typically done on shut down.
        """
        for uav in self.uavs:
            uav.strategy.close_connection()

    def call_uav_update(self):
        """
        Calls to the UAV to update its own Kinematics.
        This should not be done here with the idea that the UAV handles the calculations.
        """
        for uav in self.uavs:
            uav.x, uav.y, uav.z, uav.theta = uav.strategy.get_self_position()

            if uav.theta > 2 * np.pi:
                uav.theta -= 2 * np.pi
            elif uav.theta < 0:
                uav.theta += 2 * np.pi

    def uav_init(self):
        """
        Sends a 'takeoff' command to get each UAV ready for formation
        """
        for uav in self.uavs:
            move = uav.strategy.move_by_heading(0, 0, -2, 1, 0)

    def entropyFormation(self):
        """
        Provides commands for the Entropy formation
        """
        import os, csv
        import time
        #print(f"[CC] entropyFormation: len(uavs)={len(self.uavs)} waypoints={self.waypoints}")
        atFinalWaypoint = False  # Flag to check if the formation is complete
        commandDuration = 0.1 # How long each movement action takes (in s?) changes from 1 to 0.1 for smoother movement
        iterationCount = 0  # User statements to see something happening
        waypoints = self.waypoints.copy()  # copy of the waypoints
        nextWaypoint = waypoints.pop(0)  # [x, y] of the next waypoint
        z = -25  # Consistent height

        # MOVED THIS HERE Project 3
        waypoint_offset = 2 * self.minDistance  # The uav will seek the waypoint with this off set so they dont collide into the same point
        # In the future, may one to try and calculate the center of mass of the swarm and use that???
        closeEnough = 2  # offset from waypoint so the uav does not have to be exactly on it
        #print(f"[CC] entropyFormation: len(uavs)={len(self.uavs)} waypoints={self.waypoints}")

        # ADDED Project 3
        # Initial assignment of the first waypoint to all UAVs
        for i, uav in enumerate(self.uavs):
            uav.waypoint = [nextWaypoint[0] - (i * waypoint_offset),
                            nextWaypoint[1] + (i * waypoint_offset)]
            print(f"[CC] initial assign Drone{i+1} -> waypoint {uav.waypoint}")


        print("Starting Formation Control")
        # --- Create folder-safe CSV path ---
       

        #base_dir = os.path.join(os.path.dirname(__file__), "data-files")
        #os.makedirs(base_dir, exist_ok=True)            # Ensure folder exists

        #csv_path = os.path.join(base_dir, "entropySave.csv")

        # Print the first line to the save file
        os.makedirs(self._runs_dir, exist_ok=True)  # ensure data-files folder exists
        with open(self._csv_path, "w", newline="") as csvfile:
            fileWrite = csv.writer(csvfile, delimiter=' ', quotechar='|')
            header = []

            for i in range(1, len(self.uavs) + 1):
                header += [f"X{i}", f"Y{i}", f"EntropyDrone{i}"]
            header.append("TimeDifference")
            fileWrite.writerow(header)

            #i = 1

            #while i <= len(self.uavs):
                #writeOut.append(f"X{i}")
                #writeOut.append(f"Y{i}")
                #writeOut.append(f"EntropyDrone{i}")

                #if i == len(self.uavs):
                    #writeOut.append("TimeDifference")
                #i += 1

            #fileWrite.writerow(writeOut)

        # Capture the start time
        beginningTime = time.time()

        # Main loop until all UAVs have reached the final waypoint
        while not atFinalWaypoint:
            iterationCount = iterationCount + 1

            # Capture the time difference
            currentTime = time.time()
            # TOOK OFF /1000000000
            timeDiff = currentTime - beginningTime
            # choose when to print to console and file
            if iterationCount % 5 == 0:
                self.write_data(self.uavs, timeDiff)
            # ONLY print occasionally (every 500 iterations)
            if iterationCount % 500 == 0:
                print(f"Iteration = {iterationCount}")
                # COMMENTED OUT
                #if iterationCount % 500 == 0:
                   #self.write_data(self.uavs, timeDiff)
            
            # MOVED THIS TO ABOVE 
            #waypoint_offset = 2 * self.minDistance  # The uav will seek the waypoint with this off set so they dont collide into the same point
            # In the future, may one to try and calculate the center of mass of the swarm and use that???
            #closeEnough = 2  # offset from waypoint so the uav does not have to be exactly on it

            # COMMENTED OUT OLD LOGIC Project3
            #targetReached = len(self.uavs)  # flag for the amount of uavs on target. Bound between 0 and Max Number of UAVs
            targetReached = 0
            # Check UAV and waypoint relationship
            i = 0

            # for uav in self.uavs:
            for i, uav in enumerate(self.uavs):

                if uav.waypoint is not None:
                    # Check if the UAV has made it to its target
                    dx = uav.waypoint[0] - uav.x
                    dy = uav.waypoint[1] - uav.y
                    if np.sqrt(dx*dx + dy*dy) < closeEnough:
                        print(f"{uav.ID} has reached its target")
                        uav.waypoint = None       # freeze, no more target for this UAV
                        uav.strategy.move_by_heading(0, 0, z, commandDuration, 0)
                        continue

                # TOOK OUT Project 3
                # else:
                #     # UAV had no waypoint yet, assign the current one
                #     print(f"[CC] assign Drone{i+1} -> waypoint {nextWaypoint}")
                #     uav.waypoint = [nextWaypoint[0] - (i * waypoint_offset),
                #                     nextWaypoint[1] + (i * waypoint_offset)]

                # COMMENTED OUT OLD LOGIC Project3
                # check if the UAV has a target
                # if uav.waypoint is not None:
                #     # Check if the uav has made it to its target
                #     if np.sqrt(((uav.waypoint[0] - uav.x)**2) + ((uav.waypoint[1] - uav.y)**2)) < closeEnough:
                #         if targetReached < len(self.uavs):
                #             targetReached = targetReached + 1
                #         print(f"{uav.ID} has reached its target")
                #         uav.waypoint =None #freeze no more target, others ignore it 
                #         uav.strategy.move_by_heading(0, 0, z, commandDuration, 0)
                #         continue
                #     else:
                #         if targetReached > 0:
                #             targetReached = targetReached - 1
                # else:
                #     print(f"[CC] assign Drone{i+1} -> {uav.waypoint}")
                #     uav.waypoint = [nextWaypoint[0] - (i * waypoint_offset), nextWaypoint[1] + (i * waypoint_offset)]
                #     targetReached = targetReached - 1

                # Tell the UAV to update its position values
                self.call_uav_update()

                # Get the UAV's position status
                uavStatus = [uav.x, uav.y, uav.theta]

                # Get the desired V and theta of the uav
                otherUAVs = uav.strategy.get_other_uav_positions()
                desired = uav.calculator.calculateNextMove(otherUAVs)  # [speed, angle]
                uavDesires = np.array([round(desired[0], 4), round(desired[1], 4)])  # create numpy array

                # Smooth init transition
                if iterationCount != 1:
                    transition_angle = uavStatus[2] + uavDesires[1] * 0.5
                else:
                    transition_angle = uavStatus[2]

                # Convert from 
                transition_angle = np.where(transition_angle < 0, 2 * np.pi + transition_angle, transition_angle)

                # Account for multiples of pi due to spinning
                if transition_angle > 2 * np.pi:
                    transition_angle = transition_angle % (2 * np.pi)

                # Calculate the desired velocity XY vectors
                veloVector = np.array([round(uavDesires[0] * np.cos(transition_angle), 4), round(uavDesires[0] * np.sin(transition_angle), 4)])

                # Call to move the uav
                uav.strategy.move_by_heading(veloVector[0], veloVector[1], z, commandDuration, transition_angle)
                i = i + 1

            # Get the next waypoint to move
            # Check if all UAVs are done with their current waypoint
            all_done = all(uav.waypoint is None for uav in self.uavs)

            if all_done:
                if len(waypoints) != 0:
                    # There are more waypoints: give everyone the next one
                    nextWaypoint = waypoints.pop(0)
                    for i, uav in enumerate(self.uavs):
                        uav.waypoint = [nextWaypoint[0] - (i * waypoint_offset),
                                        nextWaypoint[1] + (i * waypoint_offset)]
                    print(f"[CC] Assigned new waypoint {nextWaypoint} to all UAVs")
                else:
                    # No more waypoints, finish
                    atFinalWaypoint = True
                    print("[CC] All UAVs reached the final waypoint. Ending simulation.")
                    # ADDED: DEBUG: summarize behaviour counts over all UAVs 
                    total_too_close = 0
                    total_entropy_lt = 0
                    total_entropy_ge = 0

                    for uav in self.uavs:
                        m = uav.manager
                        total_too_close += m.count_too_close
                        total_entropy_lt += m.count_entropy_lt
                        total_entropy_ge += m.count_entropy_ge

                    print("[STATS] Behaviour counts over entire run:")
                    print(f"    too_close branch        : {total_too_close}")
                    print(f"    entropy < threshold     : {total_entropy_lt}")
                    print(f"    entropy >= threshold    : {total_entropy_ge}")




            # COMMENTED OUT OLD LOGIC Project 3
            # if targetReached == len(self.uavs):
            #     # All UAVs have reach their target
            #     if len(waypoints) != 0:
            #         # There are more waypoints to reach
            #         targetReached = 0  # reset flag
            #         nextWaypoint = waypoints.pop(0)  # get next waypoint

            #         # assign new waypoint to the UAVs
            #         i = 0
            #         for uav in self.uavs:
            #             uav.waypoint = [nextWaypoint[0] - (i * waypoint_offset), nextWaypoint[1] + (i * waypoint_offset)]
            #             i = i + 1
            #     else:
            #         # There were no more waypoints, finished mission
            #         atFinalWaypoint = True
            #         #csvfile.close()
                    

    def orbit(self):
        """

        """
        target = "TemplateCube_Rounded_117"
        iterationCount = 0
        z = -2
        commandDuration = 1

        while True:
            iterationCount = iterationCount + 1

            for uav in self.uavs:
                # Tell the UAV to update its position values
                self.call_uav_update()

                # Get the UAV's position status
                uavStatus = [uav.x, uav.y, uav.theta]

                # get speed and direction
                desired = uav.strategy.orbit(target)
                uavDesires = np.array([round(desired[0], 4), round(desired[1], 4)])

                # Smooth init transition
                if iterationCount != 1:
                    if desired[2] is not None:
                        # Orbiting
                        facing_angle = uavStatus[2] + desired[2] * 0.5
                        moving_angle = uavStatus[2] + uavDesires[1] * 0.5
                    else:
                        # Not orbiting
                        moving_angle = uavStatus[2] + uavDesires[1] * 0.5
                        facing_angle = moving_angle
                else:
                    # Initial command
                    moving_angle = uavStatus[2]
                    facing_angle = moving_angle

                # Convert from [-pi, pi] to [0, 2pi] -
                moving_angle = np.where(moving_angle < 0, 2 * np.pi + moving_angle, moving_angle)
                facing_angle = np.where(facing_angle < 0, 2 * np.pi + facing_angle, facing_angle)

                # Calculate the desired velocity XY vectors
                veloVector = np.array([round(uavDesires[0] * np.cos(moving_angle), 4), round(uavDesires[0] * np.sin(moving_angle), 4)])

                # Call to move the uav
                uav.strategy.move_by_heading(veloVector[0], veloVector[1], z, commandDuration, facing_angle)

    def write_data(self, uavs, timeDiff):
        """
        Writes information out to the save file
        """
        #with open('data-files/entropySave.csv', 'a') as csvfile:
        with open(self._csv_path, 'a', newline='') as csvfile:
            fileWrite = csv.writer(csvfile, delimiter=' ', quotechar='|')
            writeOut = []

            for uav in uavs:
                writeOut.append(uav.x)
                writeOut.append(uav.y)
                writeOut.append(uav.entropy)
            writeOut.append(timeDiff)

            fileWrite.writerow(writeOut)

    def plotter(self):
        """
        Creates the gif for data analysis
        """
        # Array placeholders for all the data
        uavX = []
        uavXB = []
        uavXFull = []

        uavY = []
        uavYB = []
        uavYFull = []

        uavE = []
        uavEFull = []

        time = []
        timeFull = []

        # Init the plot
        plt.style.use('fivethirtyeight')
        fig = plt.figure(figsize=(15, 10))
        ax1 = fig.add_subplot(2, 1, 1)
        ax2 = fig.add_subplot(2, 1, 2)
        ax1.set(xlim=(-250, 160), ylim=(-5, 450))
        ax2.set(xlim=(-250, 160), ylim=(-5, 450))

        # Get all the data from the file
        data = pd.read_csv('data-files/entropySave.csv', delimiter=" ")

        i = 0
        while i < len(self.uavs):
            # Fill the second dimension with the number of UAVs
            uavX.append([])
            uavXB.append([])
            uavXFull.append([])
            uavY.append([])
            uavYB.append([])
            uavYFull.append([])
            uavE.append([])
            uavEFull.append([])
            i += 1

        # Live Data Setting
        i = 0
        for uav in self.uavs:
            # Grab the X,Y, Entropy, and time data
             uavXFull[i].extend(data[f"X{i+1}"])
             uavYFull[i].extend(data[f"Y{i+1}"])
             uavEFull[i].extend(data[f"EntropyDrone{i+1}"])
             i += 1
        timeFull = data["TimeDifference"]

        global iterator
        iterator = 0
        colors = ['blue', 'red', 'green', 'yellow', 'brown', 'cyan', 'deepPink']

        # This function will be called for each frame creation of the gif
        def animate(iterator):
            print(f"Timer {iterator}")
            i = 0
            while i < len(self.uavs):
                # Get the data for this iteration, appending for that 'trail' look
                uavX[i].append(uavXFull[i][iterator])
                uavY[i].append(uavYFull[i][iterator])
                uavE[i].append(uavEFull[i][iterator])
                if iterator > 1:
                    uavXB[i].append(uavXFull[i][iterator - 1])
                    uavYB[i].append(uavYFull[i][iterator - 1])
                i += 1

            time.append(timeFull[iterator])

            # Get the plots
            ax1.cla()
            ax2.cla()
            i = 0
            uLegend = []
            eLegend = []
            while i < len(self.uavs):
                # plot the data
                ax1.plot(uavXB[i], uavYB[i], linestyle="-", color=colors[i])
                ax2.plot(time, uavE[i], linestyle="-", color=colors[i])
                uLegend.append(f"UAV {i + 1}")
                eLegend.append(f"Entropy {i + 1}")
                i += 1

            # Add details to the plots
            ax1.set(xlim=([-250, 160]), ylim=([20, 200]))
            ax1.legend(uLegend)
            ax1.set_xlabel("X values")
            ax1.set_ylabel("Y Values")
            ax1.set_title('Positions')

            ax2.legend(eLegend)
            ax2.set_xlabel("Time")
            ax2.set_ylabel("Entropy")
            ax2.set_title('Entropy for each UAV')

            plt.tight_layout()

        # Create the Gif (Will finish in an ERROR by iterating too high, still works)
        ani = FuncAnimation(fig, animate, frames=10000, interval=20, repeat=False)
        ani.save('plotting.gif', writer="ffmpeg")

    


