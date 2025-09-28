import cv2
import btm
import sim
import numpy as np


with open('param.txt') as f:
    leftCameraMatrix = eval(f.readline())
    leftDistCoeffs = eval(f.readline())
    rightCameraMatrix = eval(f.readline())
    rightDistCoeffs = eval(f.readline())


if __name__ == "__main__":
    lcap = sim.VideoCapture(0)
    rcap = sim.VideoCapture(1)

    lret, limg = lcap.read()
    rret, rimg = rcap.read()

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    digital_map = btm.DigitalMap()
    drone_path = btm.DronePath()

    while cv2.waitKey(1000//30) != ord("q"):
        lret, limg = lcap.read(True)
        rret, rimg = rcap.read()

        # limg = cv2.fisheye.undistortImage(limg, leftCameraMatrix, leftDistCoeffs, Knew=leftCameraMatrix)
        # rimg = cv2.fisheye.undistortImage(rimg, rightCameraMatrix, rightDistCoeffs, Knew=rightCameraMatrix)
        # Если изображение имеет дисторсию, то изображение необходимо выровнить (раскомментировать строки выше)

        # lgray = cv2.cvtColor(limg, cv2.COLOR_BGR2GRAY)
        # rgray = cv2.cvtColor(limg, cv2.COLOR_BGR2GRAY)
        # Если изображение многоканальное, то его необходимо сделать одноканальным (раскомментировать строки выше)

        lgray = limg.copy()
        rgray = rimg.copy()
        limg = cv2.cvtColor(lgray, cv2.COLOR_GRAY2BGR)
        rimg = cv2.cvtColor(rgray, cv2.COLOR_GRAY2BGR)

        lcorners, lids, lrejected = detector.detectMarkers(lgray)
        rcorners, rids, rrejected = detector.detectMarkers(rgray)

        data = btm.markersCoordinates(lcorners, rcorners, lids, rids, leftCameraMatrix, rightCameraMatrix)
        limg, rimg = btm.drawAxes(data, limg, rimg, leftCameraMatrix, rightCameraMatrix)
        if len(data) > 0:
            pos = digital_map.getPos(data)
            drone_path.addPoint(pos[0])

        cv2.imshow("Left", cv2.resize(limg, (1920//2, 1080//2)))
        cv2.imshow("Right", cv2.resize(rimg, (1920//2, 1080//2)))
    drone_path.close()
    cv2.destroyAllWindows()
    lcap.release()
    rcap.release()