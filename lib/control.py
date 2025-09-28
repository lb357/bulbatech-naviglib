import sim
import time

if __name__ == "__main__":
    control = sim.Control()
    #while True:
    #    x = float(input("x: "))
    #    y = float(input("y: "))
    #    z = float(input("z: "))
    #    control.move_to(x, y, z)
    while True:
        for p in [[0, 4, 0], [3, 4, 0], [3, 4, 3], [0, 4, 3]]:
            control.move_to(*p)
            time.sleep(4)