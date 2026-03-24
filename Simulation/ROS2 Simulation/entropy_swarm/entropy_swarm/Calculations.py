"""
Created by Jason Carnahan - 5/14/2021 for AirSim 

Edited by Tara Moore - 2025 for AirSim 

Edited by Juan José Molina - March 2026 for ROS2 

Contains any necessary calculations needed for the UAV and performed onboard
"""

import numpy as np


class Calculations:

    def __init__(self, parent):
        self.parent = parent  # The parent UAV

    def calculateDistancesAndAngles(self, otherUAVs):
        """
        This function uses the parent x,y position and the positions of all the other UAVs and calculates
        the distance and angle to face the each other UAV.
        :param otherUAVs is an array of all the other UAVs in the system
        :returns a numpy array of the distances from this drone to all other drones
        """
        # create an array for distances and angles
        distances = np.zeros(len(otherUAVs), dtype=float)
        angles = np.zeros(len(otherUAVs), dtype=float)

        # Loop over all the UAVs and calculate distance with the distance formula and the angle with arctan
        i = 0
        for uav in otherUAVs:
            distances[i] = np.sqrt(((uav.x - self.parent.x) ** 2) + ((uav.y - self.parent.y) ** 2))
            angles[i] = np.arctan2((uav.y - self.parent.y), (uav.x - self.parent.x))
            i = i + 1
        return [distances, angles]

    def calculateEntropy(self, otherUAVs):
        """
        This function takes in the parent UAV and each other UAV to calculate the parent UAV's entropy in the system
        :param otherUAVs is an array of all the other UAVs in the system
        """
        # Get the number of UAVs in the system (+ 1 since it does not include parent UAV)
        numOfUAV = len(otherUAVs) + 1

        # DEBUG: show what maxDistance this UAV is using
        print(f"[entropy] UAV {self.parent.ID} using maxDistance = {self.parent.maxDistance}")

        distances = np.zeros(numOfUAV, dtype=float)  # Contains distances to all the other drones
        e = np.zeros(numOfUAV, dtype=float)  # Contains the first entropy calculation dependent on distance
        q = 0.5  # Weighted value for entropy?
        summation = 0  # Summation part of the entropy equation

        # Loop for all the uavs
        i = 0
        for uav in otherUAVs:
            # Calculate this UAVs distance to every other UAV and get the angle to them (Th in Luke's code)
            distances[i] = np.sqrt(((uav.x - self.parent.x) ** 2) + ((uav.y - self.parent.y) ** 2))
            # DEBUG: first few distances so we see their scale
            if self.parent.ID == "Drone1" and i < 3:
                print(f"[entropy] dist[{i}] = {distances[i]:.2f} (maxDist = {self.parent.maxDistance})")
            # Determine what the first step in the entropy calculation is
            if distances[i] >= self.parent.maxDistance:
                e[i] = self.parent.maxDistance
            elif distances[i] <= self.parent.minDistance:
                e[i] = self.parent.minDistance
            else:
                e[i] = distances[i]

            # Add this value to the summation of the Entropy equation
            summation = summation + ((e[i] / self.parent.maxDistance) ** q)
            i = i + 1

        # Calculate entropy
        return (1 - summation) / (q - 1)

    def calculateNextMove(self, otherUAVs):
        """
        This function takes in the parent UAV and all the other UAVs in the system to calculate what the parent's next move
        should be.

        (This May have to be moved to the Manager class.)

        :param otherUAVs is an array of all the other UAVs in the system
        """
        # Calculate entropy and update the value to the UAV
        St = self.calculateEntropy(otherUAVs)
        self.parent.entropy = St
        print(f"[DEBUG] UAV {self.parent.ID}  Entropy St = {St:.3f}")

            # If this UAV has no waypoint (it already reached its target),
        # just hover in place (no forward velocity, no heading change).
        if self.parent.waypoint is None:
            # Optionally still log something:
            # print(f"[DEBUG] UAV {self.parent.ID} has no waypoint, hovering")
            return [0.0, 0.0]

        # Get the distances and angles to other uavs
        store = self.calculateDistancesAndAngles(otherUAVs)
        d = store[0]  # Contains the distance to every other uav
        Th = store[1]  # Contains the angle to every other uav

        # Get the distance and angle to the next waypoint
        xd = self.parent.waypoint[0] - self.parent.x  # Delta x to waypoint
        yd = self.parent.waypoint[1] - self.parent.y  # Delta y to waypoint
        dd = np.sqrt(xd ** 2 + yd ** 2)  # Delta d to waypoint
        waypoint_theta = np.arctan2(yd, xd)  # Target Theta to waypoint

        # Find the closest uav
        closest = d[0]
        closest_index = 0
        i = 0
        for distance in d:
            if distance < closest:
                closest = distance
                closest_index = i
            i = i + 1

        # This threshold will need to be made into an equation eventually
        # Threshold for other experiments
        #threshold = 8 # 6 uav for multiagent coverage 

        # THRESHOLD FORMULA FOR Threshold Experiment
        # num_uav = total number of drones in this run
        num_uav = len(otherUAVs) + 1

        # x is the experiment parameter: -2, -1, 0, 1, 2
        # (for 3 UAVs you�ll run x = -2..2, then for 6 UAVs, etc.)
        #x = -2.5
        # change this manually between runs

        # Scale base threshold to your entropy range.
        # 0.5 * num_uav keeps the threshold in the same ballpark as St.
        #base = 0.5 * num_uav

        threshold = 0.80  # num_uav + x
        

        # DEBUG: see what threshold value is actually being used
        print(f"[threshold] UAV {self.parent.ID}: num_uav={num_uav}, "
              f"threshold={threshold:.3f}, St={St:.3f}")


        maxGroupingDistance = self.parent.minDistance * 5
        theta_com = 0  # Desired theta
        V_com = 0  # Desired velocity

        # ADDED VALUES FOR IF STATEMENT Project 3
        # Small tolerance so we don't overreact when distance is basically == minDistance
        EPS = 0.05          # 20 cm
        #DOCK_RADIUS = 3.0  # within 3 m of waypoint, focus on docking

        # Determine what the UAV should do
        # if closest < self.parent.minDistance:
        if (closest < self.parent.minDistance - EPS): #and (dd > DOCK_RADIUS):
            # ADDED: DEBUG counter
            self.parent.manager.count_too_close += 1
            # UAV is too close to another, move away
            V_com = self.parent.maxVelocity / 1.5  # Set speed

            if Th[closest_index] > 2 * np.pi:
                Th[closest_index] -= 2 * np.pi
            elif Th[closest_index] < 0:
                Th[closest_index] += 2 * np.pi

            # Determine if the other UAV is in front or behind
            if Th[closest_index] > self.parent.theta + np.pi / 2 and Th[closest_index] < 2 * np.pi:
                # other is behind, move straight away
                theta_com = (Th[closest_index] - self.parent.theta) + np.pi
                #Added
                dist = d[closest_index] 
                print(f"[DEBUG] {self.parent.ID} moving away from {otherUAVs[closest_index].ID} "
                      f"| distance={dist:.2f} | minDist={self.parent.minDistance:.2f}")
                #print(f"{self.parent.ID} moving away from {otherUAVs[closest_index].ID}")
            else:
                # Determine if the other uav is front left or front right
                if Th[closest_index] > self.parent.theta:
                    # Front left, move RIGHT
                    theta_com = Th[closest_index] - self.parent.theta - np.pi / 2
                    dist = d[closest_index]
                    print(f"[DEBUG] {self.parent.ID} moving away from {otherUAVs[closest_index].ID} to the RIGHT "
                          f"| distance={dist:.2f} | minDist={self.parent.minDistance:.2f}")
                    #print(f"{self.parent.ID} moving away from {otherUAVs[closest_index].ID} to the RIGHT")
                else:
                    # Front right, move LEFT
                    theta_com = Th[closest_index] - self.parent.theta + np.pi / 2
                    dist = d[closest_index]
                    print(f"[DEBUG] {self.parent.ID} moving away from {otherUAVs[closest_index].ID} to the LEFT "
                          f"| distance={dist:.2f} | minDist={self.parent.minDistance:.2f}")
                    #print(f"{self.parent.ID} moving away from {otherUAVs[closest_index].ID} to the LEFT")

        # Check if the entropy level is less than the threshold
        elif St < threshold:
            # ADDED: DEBUG counter
            self.parent.manager.count_entropy_lt += 1
            V_com = self.parent.maxVelocity
             # Dynamic buffer so large swarms do not deadlock
            numOfUAV = len(otherUAVs) + 1
            buffer = 0.75 if numOfUAV > 3 else 0.85
            # Check if this UAV is too close to another
            #if closest > self.parent.minDistance:
            if closest > buffer * self.parent.minDistance: 
                # not too close, move to waypoint
                print(f"{self.parent.ID} moving to waypoint: [{self.parent.waypoint[0]}, {self.parent.waypoint[1]}]")
                theta_com = waypoint_theta - self.parent.theta
                self.parent.target = self.parent.waypoint

        else:
            # ADDED: DEBUG counter
            self.parent.manager.count_entropy_ge += 1
            # Not in threshold, move to the closest UAV outside the min grouping distance
            V_com = 2 * self.parent.maxVelocity

            # Find what UAV is the closest outside the min grouping distance
            if closest < maxGroupingDistance:
                closest = 999999
                closest_index = 0
                i = 0
                for distance in d:
                    if distance > maxGroupingDistance and distance < closest:
                        closest = distance
                        closest_index = i
                    i = i + 1

            # Move toward it
            print(f"{self.parent.ID} moving to closest neighbor {otherUAVs[closest_index].ID}")
            theta_com = Th[closest_index] - self.parent.theta

        if theta_com > np.pi:
            theta_com = -2 * np.pi + theta_com
        elif theta_com < -np.pi:
            theta_com = 2 * np.pi + theta_com


        # Return the determined Velocity and angle
        return [V_com, theta_com]
