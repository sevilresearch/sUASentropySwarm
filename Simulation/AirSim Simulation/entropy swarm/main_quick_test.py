'''
Created by Jason Carnahan  - 5/14/2021

Edited by Juan José Molina - March 2026
'''


from CentralControl import CentralControl
from Strategies.AirSimStrategy import AirSimStrategy

# Create the strategy and central controller
strategy = AirSimStrategy()
cc = CentralControl(strategy)

# Wire the strategy to the controller
strategy.establish_connection(cc)

# Initialize the system with parameters (example: 6 UAVs, sep=5, rows=3)
cc.init_system(num_of_uavs=6, separating_distance=5, row_length=3)

print("[TEST] launching entropyFormation()...")
cc.entropyFormation()