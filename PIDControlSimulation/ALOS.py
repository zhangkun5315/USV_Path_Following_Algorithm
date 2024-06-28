import math
import numpy as np
import time
from PythonDll import MyPythonDll
from PIDAm import MyPID
import copy as cp
# from PIDController import PIDController
'''
******************************************************************************************************************

count = 0  # 计数器，方便后续输出查询
pk_list = [0, 0, 0]  # 用于保存路径点序号
meeting_flag = [False, False, False]  # 是否抵达集合点
success_flag = [False, False, False]  # 是否绕桩完成
only_one = [True, True, True]  # 一次标记
only_one_sig = [1]

# A* NNN = AStarDemo -> Line134  NNN = 1  如果再增加到3，大方向就错了，这个不能调了
DP_tolerance = 2  # 路径压缩参数，越小路径点越多，压缩比越小.
wind_offset = 8  # 绕桩的距离
switch_PathPoint = 10 # 这个最好不动，不能调整
end_tunning = 8  # 终点微调
meeting_y_offset = 0  # meeting_y微调
# meeting_y_sigle_offset = 8  # 2,3 避免一条线，容易撞车

# PID参数设置  # Ep,Ei,Ed = 15, 0.000001, 1 # 推进器
Rp,Ri,Rd = 6, 9.25e-06, 2 # 舵

# 实例化三艘船的PID  # USV3Epid = MyPID(Ep,Ei,Ed)  # 推进器PID
USV1Rpid = MyPID(Rp,Ri,Rd)  # 舵PID 0.03, 0.1, 0.00001
USV2Rpid = MyPID(Rp,Ri,Rd)  # 舵PID
USV3Rpid = MyPID(Rp,Ri,Rd)  # 舵PID

def get_R_LOS(X_p, path_points, pk_index):  # find R_LOS coordinate.
    dN = switch_PathPoint  # 10m之内开始切换路径点

    # 构建直线方程 [A, B, C] -> Ax + By + C = 0
    # lineFunc = [0.0, 0.0, 0.0]
    x1 = path_points[pk_index][0]
    x2 = path_points[pk_index + 1][0]
    y1 = path_points[pk_index][1]
    y2 = path_points[pk_index + 1][1]
    R_LOS = [x1, y1]  # 防止有遗漏情况发生。

    if x1 == x2:
        de = np.abs(X_p[0] - x2)
        if de < dN:
            if y1 < y2:
                R_LOS = [round(x1, 5), round(X_p[1] + np.sqrt(dN ** 2 - de ** 2), 5)]
            else:
                R_LOS = [round(x1, 5), round(X_p[1] - np.sqrt(dN ** 2 - de ** 2), 5)]
        else:
            R_LOS = [round(x1, 5), round(X_p[1], 5)]
            if y2 > y1:  # 判断是否在线段内
                if X_p[1] < y1:
                    R_LOS = [round(x1, 5), round(y1, 5)]
                if X_p[1] > y2 :
                    R_LOS = [round(x2, 5), round(y2, 5)]
            if y1 > y2:
                if X_p[1] > y1:
                    R_LOS = [round(x1, 5), round(y1, 5)]
                if X_p[1] < y2 :
                    R_LOS = [round(x2, 5), round(y2, 5)]
    elif y1 == y2:
        de = np.abs(X_p[1] - y2)
        if de < dN:
            if x1 > x2:
                R_LOS = [round(X_p[0] - np.sqrt(dN ** 2 - de ** 2), 5), round(y2, 5)]
            else:
                R_LOS = [round(X_p[0] + np.sqrt(dN ** 2 - de ** 2), 5), round(y2, 5)]
        else:
            R_LOS = [round(X_p[0], 5), round(y1, 5)]
            if x2 > x1:  # 判断是否在线段内
                if X_p[0] < x1:
                    R_LOS = [round(x1, 5), round(y1, 5)]
                if X_p[0] > x2:
                    R_LOS = [round(x2, 5), round(y2, 5)]
            if x1 > x2:
                if X_p[0] > x1:
                    R_LOS = [round(x1, 5), round(y1, 5)]
                if X_p[0] < x2:
                    R_LOS = [round(x2, 5), round(y2, 5)]
    else:
        lineFunc = [y2 - y1, x1 - x2, (x2 - x1) * y1 - (y2 - y1) * x1]
        # 点到直线的距离求de -> d=│（Ax+By+C）/√(A²+B²)│
        de = np.abs(
            (lineFunc[0] * X_p[0] + lineFunc[1] * X_p[1] + lineFunc[2]) / np.sqrt(lineFunc[0] ** 2 + lineFunc[1] ** 2))

        d_pp = np.sqrt((X_p[0]-x1)**2 + (X_p[1]-y1)**2)

        k = -(lineFunc[0] / lineFunc[1])
        b0 = -(lineFunc[2] / lineFunc[1])

        if de < dN:
            a = k ** 2 + 1
            b = 2 * k * (b0 - X_p[1]) - 2 * X_p[0]
            c = (b0 - X_p[1]) ** 2 + X_p[0] ** 2 - dN ** 2
            delta = b ** 2 - 4 * a * c
            if delta >= 0:
                p1x = round((-b - np.sqrt(delta)) / (2 * a), 5)
                p2x = round((-b + np.sqrt(delta)) / (2 * a), 5)
                p1y = round(k * p1x + b0, 5)
                p2y = round(k * p2x + b0, 5)
                inp = [[p1x, p1y], [p2x, p2y]]
                if x2 > x1:  # 判断是否在线段内
                    xxx1 = max(p1x, p2x)
                    yyy1 = k * xxx1 + b0
                    R_LOS = [round(xxx1, 5), round(yyy1, 5)]
                if x1 > x2:
                    xxx1 = min(p1x, p2x)
                    yyy1 = k * xxx1 + b0
                    R_LOS = [round(xxx1, 5), round(yyy1, 5)]

            # 增加一个判断，不要让延长线也考虑在内了。
            if d_pp > dN:
                xxx1 = (k * (X_p[1] - b0) + X_p[0]) / (k ** 2 + 1)
                yyy1 = k * xxx1 + b0
                if x2 > x1:  # 判断是否在线段内
                    if xxx1 < x1:
                        R_LOS = [round(x1, 5), round(y1, 5)]
                if x1 > x2:
                    if xxx1 > x1:
                        R_LOS = [round(x1, 5), round(y1, 5)]

        else:
            xxx1 = (k * (X_p[1] - b0) + X_p[0]) / (k ** 2 + 1)
            yyy1 = k * xxx1 + b0
            R_LOS = [round(xxx1, 5), round(yyy1, 5)]
            if x2 > x1:  # 判断是否在线段内
                if xxx1 < x1:
                    R_LOS = [round(x1, 5), round(y1, 5)]
            if x1 > x2:
                if xxx1 > x1:
                    R_LOS = [round(x1, 5), round(y1, 5)]

    return R_LOS
