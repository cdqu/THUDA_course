# 从rssi计算距离
def get_distance(rssi):
    A = 47  # typical 45-49
    n = 4   # typical 3.25—4.5
    d = 10^((abs(rssi) - A) / (10 * n))
    return d
    
#  从三个距离计算坐标
def getpos(x1, y1, x2, y2, x3, y3, dis1, dis2, dis3):
    pos_x, pos_y = sympy.symbols('x y')
    f1 = 2*pos_x*(x1-x2)+np.square(x2)-np.square(x1)+2*pos_y*(y1-y2)+np.square(y2)-np.square(y1)+np.square(dis1)-np.square(dis2)
    f2 = 2*pos_x*(x3-x2)+np.square(x2)-np.square(x3)+2*pos_y*(y3-y2)+np.square(y2)-np.square(y3)+np.square(dis3)-np.square(dis2)
    r = sympy.solve([f1, f2], [pos_x, pos_y])
    px = r[pos_x]
    py = r[pos_y]
    return px, py