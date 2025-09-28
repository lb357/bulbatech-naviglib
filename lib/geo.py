from pygeoguz.transform import *


def bl_offset(map_point, offset):
    p_bl = PointBL(map_point[0], map_point[1])
    p_xy = bl2xy(p_bl)
    p_xy.y += offset[0]
    p_xy.x += offset[1]
    op = xy2bl(p_xy)
    return op.b, op.l


def xy_offset(map_point, offset):
    return map_point[0] + offset[0], map_point[1] - offset[1]