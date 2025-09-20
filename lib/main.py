import sim
import cv2

cap = sim.VideoCapture(0)
while True:
    ret, img = cap.read(True)
    cv2.imshow("DEBUG", img)
    cv2.waitKey(1000//30)