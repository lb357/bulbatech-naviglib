import cv2
import math
import numpy as np
import numpy.typing as npt
import re3d
import json
import geo
import csv
import locale


ARUCO_SIZE = 0.2
LEFT_TO_ORIGIN_MAT = np.array([[1, 0, 0, -0.05],
                                   [0, 1, 0, 0.00],
                                   [0, 0, 1, 0.00],
                                   [0, 0, 0, 1]])
RIGHT_TO_LEFT_OFFSET = [0.10, 0.0, 0.0]
W_COEFF = [2, 2, 1]
unDistCoeffs = np.zeros(4, dtype=np.float32)


def triangulate(lpos, rpos, lcameraMatrix, rcameraMatrix, offset):
    ul, vl = lpos
    ur, vr = rpos
    fxl, fyl = lcameraMatrix[0][0], lcameraMatrix[1][1]
    cxl, cyl = lcameraMatrix[0][2], lcameraMatrix[1][2]
    fxr, fyr = rcameraMatrix[0][0], rcameraMatrix[1][1]
    cxr, cyr = rcameraMatrix[0][2], rcameraMatrix[1][2]
    x, y = offset[:2]
    X = (fxr * x) / ((fxl / (ul - cxl)) * (ur - cxr) - fxr)
    z = (fxl * X) / (ul - cxl)
    Yl = (vl - cyl) * z / fyl
    Yr = (vr - cyr) * z / fyr - y
    Y = (Yl + Yr) / 2
    return X, Y, z


def weighted(data: list, weight: list):
    return sum([float(x) * float(y) for x, y in zip(data, weight)]) / sum(weight)

def markersCoordinates(lcorners, rcorners, lids, rids,
                leftCameraMatrix, rightCameraMatrix,
                leftDistCoeffs = unDistCoeffs,
                rightDistCoeffs = unDistCoeffs,
                aruco_size = ARUCO_SIZE,
                left_to_origin_mat = LEFT_TO_ORIGIN_MAT,
                right_to_left_mat=None,
                w_coeff=None
                ):
    if w_coeff is None:
        w_coeff = W_COEFF
    if right_to_left_mat is None:
        right_to_left_mat = RIGHT_TO_LEFT_OFFSET
    marker_points = np.array(
        [[-aruco_size / 2, aruco_size / 2, 0], [aruco_size / 2, aruco_size / 2, 0],
         [aruco_size / 2, -aruco_size / 2, 0], [-aruco_size / 2, -aruco_size / 2, 0]], dtype=np.float32)
    out = {}
    if lids is not None and rids is not None:
        if len(lids) != 0 and len(rids) != 0:
            for lmarker in range(len(lids)):
                idx = int(lids[lmarker][0])
                npw = np.where(rids == idx)

                if len(npw[0]) != 0:
                    rmarker = int(npw[0][0])


                    lcornersm = lcorners[lmarker]
                    rcornersm = rcorners[rmarker]


                    lretp, lrvecs, ltvecs, lerr = cv2.solvePnPGeneric(marker_points, lcornersm, leftCameraMatrix,
                                                                      leftDistCoeffs,
                                                                      flags=cv2.SOLVEPNP_SQPNP)
                    rretp, rrvecs, rtvecs, rerr = cv2.solvePnPGeneric(marker_points, rcornersm, rightCameraMatrix,
                                                                      rightDistCoeffs, flags=cv2.SOLVEPNP_SQPNP)

                    lrtd = {}
                    for rvect, tvect in zip(lrvecs, ltvecs):
                        tmat = re3d.getCTW(rvect, tvect)
                        ptt = np.dot(tmat, np.array([0.0, 0.0, 1.0, 1.0]))
                        ptt_dist = math.dist((0, 0, 0), ptt.tolist()[:3])
                        lrtd[ptt_dist] = (rvect, tvect)
                    lrvec, ltvec = lrtd[min(list(lrtd.keys()))]

                    rrtd = {}
                    for rvect, tvect in zip(rrvecs, rtvecs):
                        tmat = re3d.getCTW(rvect, tvect)
                        ptt = np.dot(tmat, np.array([0.0, 0.0, 1.0, 1.0]))
                        ptt_dist = math.dist((0, 0, 0), ptt.tolist()[:3])
                        rrtd[ptt_dist] = (rvect, tvect)
                    rrvec, rtvec = rrtd[min(list(rrtd.keys()))]


                    lcTw = re3d.getCTW(lrvec, ltvec)
                    rcTw = re3d.getCTW(rrvec, rtvec)

                    lp0pnp_ = np.round(np.dot((lcTw), np.array([-aruco_size / 2, aruco_size / 2, 0, 1], dtype=np.float32)),
                                       3).tolist()
                    rp0pnp_ = np.round(np.dot((rcTw), np.array([-aruco_size / 2, aruco_size / 2, 0, 1], dtype=np.float32)),
                                       3).tolist()
                    lp0pnp = np.round(np.dot(left_to_origin_mat, lp0pnp_).ravel(), 3).tolist()
                    rp0pnp = np.round(np.dot(left_to_origin_mat,
                                             np.array([a + b for a, b in zip(list(rp0pnp_), right_to_left_mat)] + [1],
                                                      dtype=np.float32)).ravel(), 3).tolist()

                    lp1pnp_ = np.round(np.dot((lcTw), np.array([aruco_size / 2, aruco_size / 2, 0, 1], dtype=np.float32)),
                                       3).tolist()
                    rp1pnp_ = np.round(np.dot((rcTw), np.array([aruco_size / 2, aruco_size / 2, 0, 1], dtype=np.float32)),
                                       3).tolist()
                    lp1pnp = np.round(np.dot(left_to_origin_mat, lp1pnp_).ravel(), 3).tolist()
                    rp1pnp = np.round(np.dot(left_to_origin_mat,
                                             np.array([a + b for a, b in zip(list(rp1pnp_), right_to_left_mat)] + [1],
                                                      dtype=np.float32)).ravel(), 3).tolist()

                    lp2pnp_ = np.round(np.dot((lcTw), np.array([aruco_size / 2, -aruco_size / 2, 0, 1], dtype=np.float32)),
                                       3).tolist()
                    rp2pnp_ = np.round(np.dot((rcTw), np.array([aruco_size / 2, -aruco_size / 2, 0, 1], dtype=np.float32)),
                                       3).tolist()
                    lp2pnp = np.round(np.dot(left_to_origin_mat, lp2pnp_).ravel(), 3).tolist()
                    rp2pnp = np.round(np.dot(left_to_origin_mat,
                                             np.array([a + b for a, b in zip(list(rp2pnp_), right_to_left_mat)] + [1],
                                                      dtype=np.float32)).ravel(), 3).tolist()

                    lp3pnp_ = np.round(np.dot((lcTw), np.array([-aruco_size / 2, -aruco_size / 2, 0, 1], dtype=np.float32)),
                                       3).tolist()
                    rp3pnp_ = np.round(np.dot((rcTw), np.array([-aruco_size / 2, -aruco_size / 2, 0, 1], dtype=np.float32)),
                                       3).tolist()
                    lp3pnp = np.round(np.dot(left_to_origin_mat, lp3pnp_).ravel(), 3).tolist()
                    rp3pnp = np.round(np.dot(left_to_origin_mat,
                                             np.array([a + b for a, b in zip(list(rp3pnp_), right_to_left_mat)] + [1],
                                                      dtype=np.float32)).ravel(), 3).tolist()
                    if np.sum(leftDistCoeffs.ravel()) == 0 and np.sum(rightDistCoeffs.ravel()) == 0:
                        lmc = lcornersm.reshape((4, 2))
                        rmc = rcornersm.reshape((4, 2))

                        tp0 = triangulate(lmc[0], rmc[0], leftCameraMatrix, rightCameraMatrix, right_to_left_mat)
                        tp0pnp = np.round(-1 * np.dot(left_to_origin_mat,
                                                      np.array([a + b for a, b in zip(list(tp0), right_to_left_mat)] + [1],
                                                               dtype=np.float32)).ravel(), 3).tolist()
                        tp1 = triangulate(lmc[1], rmc[1], leftCameraMatrix, rightCameraMatrix, right_to_left_mat)
                        tp1pnp = np.round(-1 * np.dot(left_to_origin_mat,
                                                      np.array([a + b for a, b in zip(list(tp1), right_to_left_mat)] + [1],
                                                               dtype=np.float32)).ravel(), 3).tolist()
                        tp2 = triangulate(lmc[2], rmc[2], leftCameraMatrix, rightCameraMatrix, right_to_left_mat)
                        tp2pnp = np.round(-1 * np.dot(left_to_origin_mat,
                                                      np.array([a + b for a, b in zip(list(tp2), right_to_left_mat)] + [1],
                                                               dtype=np.float32)).ravel(), 3).tolist()
                        tp3 = triangulate(lmc[0], rmc[0], leftCameraMatrix, rightCameraMatrix, right_to_left_mat)
                        tp3pnp = np.round(-1 * np.dot(left_to_origin_mat,
                                                      np.array([a + b for a, b in zip(list(tp3), right_to_left_mat)] + [1],
                                                               dtype=np.float32)).ravel(), 3).tolist()



                        p0pnp = np.array([weighted([lp0pnp[0], rp0pnp[0], tp0pnp[0]], w_coeff),
                                          weighted([lp0pnp[1], rp0pnp[1], tp0pnp[1]], w_coeff),
                                          weighted([lp0pnp[2], rp0pnp[2], tp0pnp[2]], w_coeff)])

                        p1pnp = np.array([weighted([lp1pnp[0], rp1pnp[0], tp1pnp[0]], w_coeff),
                                          weighted([lp1pnp[1], rp1pnp[1], tp1pnp[1]], w_coeff),
                                          weighted([lp1pnp[2], rp1pnp[2], tp1pnp[2]], w_coeff)])

                        p2pnp = np.array([weighted([lp2pnp[0], rp2pnp[0], tp2pnp[0]], w_coeff),
                                          weighted([lp2pnp[1], rp2pnp[1], tp2pnp[1]], w_coeff),
                                          weighted([lp2pnp[2], rp2pnp[2], tp2pnp[2]], w_coeff)])

                        p3pnp = np.array([weighted([lp3pnp[0], rp3pnp[0], tp3pnp[0]], w_coeff),
                                          weighted([lp3pnp[1], rp3pnp[1], tp3pnp[1]], w_coeff),
                                          weighted([lp3pnp[2], rp3pnp[2], tp3pnp[2]], w_coeff)])
                    else:
                        p0pnp = np.array([weighted([lp0pnp[0], rp0pnp[0]], w_coeff[:2]),
                                          weighted([lp0pnp[1], rp0pnp[1]], w_coeff[:2]),
                                          weighted([lp0pnp[2], rp0pnp[2]], w_coeff[:2])])

                        p1pnp = np.array([weighted([lp1pnp[0], rp1pnp[0]], w_coeff[:2]),
                                          weighted([lp1pnp[1], rp1pnp[1]], w_coeff[:2]),
                                          weighted([lp1pnp[2], rp1pnp[2]], w_coeff[:2])])

                        p2pnp = np.array([weighted([lp2pnp[0], rp2pnp[0]], w_coeff[:2]),
                                          weighted([lp2pnp[1], rp2pnp[1]], w_coeff[:2]),
                                          weighted([lp2pnp[2], rp2pnp[2]], w_coeff[:2])])

                        p3pnp = np.array([weighted([lp3pnp[0], rp3pnp[0]], w_coeff[:2]),
                                          weighted([lp3pnp[1], rp3pnp[1]], w_coeff[:2]),
                                          weighted([lp3pnp[2], rp3pnp[2]], w_coeff[:2])])


                    ppnp = np.array([p0pnp, p1pnp, p2pnp, p3pnp], dtype=np.float32)
                    ftr, ftr_scale = cv2.estimateAffine3D(ppnp, marker_points, force_rotation=False)
                    ftr_c = np.dot(ftr / ftr_scale, np.array([0, 0, 0, 1]))
                    if ftr_scale > 0:
                        out[idx] = (ftr_c, math.dist((0,0,0), ftr_c), lrvec, ltvec, rrvec, rtvec)
    return out


class DigitalMap:
    def __init__(self, map_file="map.json"):
        with open(map_file, "r") as file:
            map_file_data = json.loads(file.read())
        self.map_data = {}
        for idx in map_file_data:
            self.map_data[int(idx)] = map_file_data[idx]

    def getPos(self, data):
        geo_pos_pts = [[], []]
        field_pos_pts = [[], []]
        pos_dist = []
        for idx in data:
            coord = data[idx][0]
            dist = data[idx][1]
            xy = geo.xy_offset(self.map_data[idx]["field"], coord)
            bl = geo.bl_offset(self.map_data[idx]["geo"], coord)
            geo_pos_pts[0].append(bl[0])
            geo_pos_pts[1].append(bl[1])
            field_pos_pts[0].append(xy[0])
            field_pos_pts[1].append(xy[1])
            pos_dist.append(dist)
        return ([weighted(field_pos_pts[0], pos_dist), weighted(field_pos_pts[1], pos_dist)],
                [weighted(geo_pos_pts[0], pos_dist), weighted(geo_pos_pts[1], pos_dist)])


class DronePath:
    def __init__(self, path_file="path.csv", locale_c="ru_RU"):
        self.path_file = path_file
        self.file = open(self.path_file, "w", newline='')
        self.writer = csv.writer(self.file, delimiter=' ', quoting=csv.QUOTE_MINIMAL)

    def addPoint(self, point):
        self.writer.writerow(list(point))

    def close(self):
        self.file.close()


def drawAxes(data, limg, rimg, leftCameraMatrix, rightCameraMatrix,
              leftDistCoeffs=unDistCoeffs, rightDistCoeffs=unDistCoeffs):
    for idx in data:
        lrvec, ltvec, rrvec, rtvec = data[idx][2:6]
        limg = cv2.drawFrameAxes(limg, leftCameraMatrix, leftDistCoeffs, lrvec, ltvec, 0.1, 2)
        rimg = cv2.drawFrameAxes(rimg, rightCameraMatrix, rightDistCoeffs, rrvec, rtvec, 0.1, 2)
    return limg, rimg