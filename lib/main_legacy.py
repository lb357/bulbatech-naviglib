import sim
import cv2
import numpy as np
import re3d

ARUCO_SIZE = 0.2
LEFT_TO_ORIGIN_MAT = np.array([[1, 0, 0, -0.05],
                               [0, 1, 0, 0.00],
                               [0, 0, 1, 0.00],
                               [0, 0, 0, 1]])

RIGHT_TO_LEFT_OFFSET = [0.10, 0.0, 0.0]

with open('calibration/param.txt') as f:
    leftCameraMatrix = eval(f.readline())
    leftDistCoeffs = eval(f.readline())
    rightCameraMatrix = eval(f.readline())
    rightDistCoeffs = eval(f.readline())
    R = eval(f.readline())
    T = eval(f.readline())


def triangulate(lpos, rpos, lcameraMatrix, rcameraMatrix, offset):
    ul, vl = lpos
    ur, vr = rpos
    fxl, fyl = lcameraMatrix[0][0], lcameraMatrix[1][1]
    cxl, cyl = lcameraMatrix[0][2], lcameraMatrix[1][2]
    fxr, fyr = rcameraMatrix[0][0], rcameraMatrix[1][1]
    cxr, cyr = rcameraMatrix[0][2], rcameraMatrix[1][2]
    x, y = offset[:2]
    X = (fxr * x) / ( (fxl / (ul-cxl)) * (ur - cxr) - fxr)
    z = (fxl * X) / (ul - cxl)
    Yl = (vl - cyl) * z / fyl
    Yr = (vr - cyr) * z / fyr - y
    Y = (Yl+Yr)/2
    return X, Y, z

marker_points = np.array(
    [[-ARUCO_SIZE / 2, ARUCO_SIZE / 2, 0], [ARUCO_SIZE / 2, ARUCO_SIZE / 2, 0],
     [ARUCO_SIZE / 2, -ARUCO_SIZE / 2, 0], [-ARUCO_SIZE / 2, -ARUCO_SIZE / 2, 0]],    dtype=np.float32)

lcap = sim.VideoCapture(0)
rcap = sim.VideoCapture(1)

lret, limg = lcap.read()
rret, rimg = rcap.read()

subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)


unDistCoeffs = np.zeros(4, dtype=np.float32)
Rl, Rr, Pl, Pr, Q, validPixROIl, validPixROIr = cv2.stereoRectify(leftCameraMatrix, leftDistCoeffs,# unDistCoeffs,
                                                                  rightCameraMatrix, rightDistCoeffs,#unDistCoeffs,
                                                                  limg.shape[-2::-1], R, T)


print(R, T)
Pla = np.dot(leftCameraMatrix, np.concatenate((R, T), axis=(1)))
print(Pla, Pl)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

while True:
    lret, limg = lcap.read()
    rret, rimg = rcap.read()

    #limg = cv2.fisheye.undistortImage(limg, leftCameraMatrix, leftDistCoeffs, Knew=leftCameraMatrix)
    #rimg = cv2.fisheye.undistortImage(rimg, rightCameraMatrix, rightDistCoeffs, Knew=rightCameraMatrix)

    #lgray = cv2.cvtColor(limg, cv2.COLOR_BGR2GRAY)
    #rgray = cv2.cvtColor(limg, cv2.COLOR_BGR2GRAY)
    lgray = limg.copy()
    rgray = rimg.copy()

    limg = cv2.cvtColor(lgray, cv2.COLOR_GRAY2BGR)
    rimg = cv2.cvtColor(rgray, cv2.COLOR_GRAY2BGR)
    lcorners, lids, lrejected = detector.detectMarkers(lgray)
    rcorners, rids, rrejected = detector.detectMarkers(rgray)
    if lids is not None and rids is not None:
        if len(lids) != 0 and len(rids) != 0:
            for lmarker in range(len(lids)):
                idx = int(lids[lmarker][0])
                rmarker = int(np.where(rids == idx)[0][0])

                lp = cv2.triangulatePoints(Pl, Pr, lcorners[lmarker], rcorners[rmarker])

                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
                lcornersm = cv2.cornerSubPix(lgray, np.float32(lcorners[lmarker]), (3, 3), (-1, -1), subpix_criteria)
                rcornersm = cv2.cornerSubPix(lgray, np.float32(rcorners[rmarker]), (3, 3), (-1, -1), subpix_criteria)

                lretp, lrvec, ltvec, lin = cv2.solvePnPRansac(marker_points, lcornersm, leftCameraMatrix, unDistCoeffs, flags=cv2.SOLVEPNP_SQPNP)
                rretp, rrvec, rtvec, rin = cv2.solvePnPRansac(marker_points, rcornersm, rightCameraMatrix, unDistCoeffs, flags=cv2.SOLVEPNP_SQPNP)

                limg = cv2.drawFrameAxes(limg, leftCameraMatrix, unDistCoeffs, lrvec, ltvec, 0.1, 2)
                rimg = cv2.drawFrameAxes(rimg, leftCameraMatrix, unDistCoeffs, rrvec, rtvec, 0.1, 2)

                #RIGHT_TO_LEFT_MAT = np.dot(re3d.getCTW(lrvec, ltvec), np.linalg.inv(re3d.getCTW(rrvec, rtvec)))
                #lppnp = np.round(np.dot(LEFT_TO_ORIGIN_MAT, np.array([*ltvec,  [1]])).ravel(), 3).tolist()
                #rppnp = np.round(np.dot(LEFT_TO_ORIGIN_MAT, np.dot(RIGHT_TO_LEFT_MAT, np.array(rtvec.tolist()+[[1]], dtype=np.float32))).ravel(), 3).tolist()

                lppnp = np.round(np.dot((re3d.getCTW(lrvec, ltvec)), np.array([0,0,0,1], dtype=np.float32)), 3).tolist()
                rppnp = np.round(np.dot((re3d.getCTW(rrvec, rtvec)), np.array([0,0,0,1], dtype=np.float32)), 3).tolist()

                lppnp = np.round(np.dot(LEFT_TO_ORIGIN_MAT, lppnp).ravel(), 3).tolist()
                rppnp = np.round(np.dot(LEFT_TO_ORIGIN_MAT, np.array([a+b for a, b in zip(list(rppnp), RIGHT_TO_LEFT_OFFSET)] + [1], dtype=np.float32)).ravel(), 3).tolist()

                lmc = [(lcornersm[0][0][0] + lcornersm[0][1][0] + lcornersm[0][2][0] + lcornersm[0][3][0]) / 4,
                       (lcornersm[0][0][1] + lcornersm[0][1][1] + lcornersm[0][2][1] + lcornersm[0][3][1]) / 4]
                rmc = [(rcornersm[0][0][0] + rcornersm[0][1][0] + rcornersm[0][2][0] + rcornersm[0][3][0]) / 4,
                       (rcornersm[0][0][1] + rcornersm[0][1][1] + rcornersm[0][2][1] + rcornersm[0][3][1]) / 4]

                tp = triangulate(lmc, rmc, leftCameraMatrix, rightCameraMatrix, RIGHT_TO_LEFT_OFFSET)
                tppnp = np.round(-1 * np.dot(LEFT_TO_ORIGIN_MAT, np.array([a+b for a, b in zip(list(tp), RIGHT_TO_LEFT_OFFSET)] + [1], dtype=np.float32)).ravel(), 3).tolist()

                print(lppnp, rppnp, tppnp)
                ppnp = [(2*lppnp[0] + 2*rppnp[0] + 1*tppnp[0]) / 5,
                        (2*lppnp[1] + 2*rppnp[1] + 1*tppnp[1]) / 5,
                        (2*lppnp[2] + 2*rppnp[2] + 1*tppnp[2]) / 5]

                #ppnp = [(2 * lppnp[0] + 2 * rppnp[0]  ) / 4,
                #                (2*lppnp[1] + 2*rppnp[1] ) / 4,
                #                (2*lppnp[2] + 2*rppnp[2] ) / 4]

                limg = cv2.putText(
                    limg,
                    f"x:{-1 * ppnp[0]:.3f}/y:{ (ppnp[2]+1.5):.3f}/z:{-1 * ppnp[1]:.3f}",
                    (int(lmc[0]), int(lmc[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2
                )

    cv2.imshow("Left", cv2.resize(limg, (1920//2, 1080//2)))
    cv2.imshow("Right", cv2.resize(rimg, (1920//2, 1080//2)))
    cv2.waitKey(1000//30)