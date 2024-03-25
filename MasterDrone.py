import KeyMovement as km
from djitellopy import Tello
from time import sleep
master = Tello(host="192.168.31.83")

def masterInitialize():

    km.init()
    master.connect()
    print(master.get_battery())
    sleep(3)
    master.takeoff()

    while True:
        vals = getKeyboardInput()
        master.send_rc_control(vals[0], vals[1], vals[2], vals[3])
        sleep(0.05)

def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50

    if km.getKey("LEFT"):
        if km.getKey("RCTRL"): master.flip_left(); sleep(3)
        else: lr = -speed
    elif km.getKey("RIGHT"):
        if km.getKey("RCTRL"): master.flip_right(); sleep(3)
        else: lr = speed

    if km.getKey("UP"):
        if km.getKey("RCTRL"): master.flip_forward(); sleep(3)
        else: fb = -speed
    elif km.getKey("DOWN"):
        if km.getKey("RCTRL"): master.flip_back(); sleep(3)
        else: fb = speed

    if km.getKey("w"): ud = speed
    elif km.getKey("s"): ud = -speed

    if km.getKey("a"): yv = speed*1.5
    elif km.getKey("d"): yv = -speed*1.5

    if km.getKey("q"): master.land()
    #elif km.getKey("e"): master.takeoff()

    return [lr, fb, ud, yv]




