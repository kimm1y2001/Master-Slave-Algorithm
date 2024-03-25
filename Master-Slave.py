# from LineFollowing import LineFollowerV2 as master
import MasterDrone as master
import SlaveDrone as slave
from time import sleep
import threading
import multiprocessing

def runMaster():
    master.masterInitialize()
    master.getKeyboardInput()          # For Basic Master Drone
    # master.masterRun()                  # For Line Following

def runSlave():
    slave.slaveInitialize()
    slave.createTrackbar()
    slave.runSlave()

# Making two drones run simultaneously
tMaster = threading.Thread(target=runMaster)
tSlave = threading.Thread(target=runSlave)
# tMaster = multiprocessing.Process(target=runMaster)
# tSlave = multiprocessing.Process(target=runSlave)

tSlave.start()
print("Slave start")
sleep(3.80)
tMaster.start()
print("Master start")

tMaster.join()
tSlave.join()


