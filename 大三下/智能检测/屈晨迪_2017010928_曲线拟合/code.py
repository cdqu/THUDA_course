# 热敏电阻曲线拟合
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.metrics import mean_squared_error
from sklearn import preprocessing
from sklearn.linear_model import LinearRegression

RT0 = 22
T0 = 25 + 273.15
beta = 3100


def RT(t_):
    return RT0 * np.exp(beta * (1 / (t_ + 273.15) - 1 / T0))


x = range(2, 101, 2)
y = []
for t in x:
    y.append(RT(t))
# 绘制特性曲线
print(y)
plt.plot(x, y)
plt.xlabel("t")
plt.ylabel("RT")
plt.show()

# 划分训练测试集
x_train = range(20, 81, 2)
x_test = list(set(x) - set(x_train))
x_test.sort()

y_train = []
for t in x_train:
    y_train.append(RT(t))
y_train = np.array(y_train)
noise = np.random.normal(0, 0.5, 31)  # 随机噪声
y_train = y_train + noise

y_tr_real = []
for t in x_train:
    y_tr_real.append(RT(t))
y_te_real = []
for t in x_test:
    y_te_real.append(RT(t))

# 最小二乘法拟合
mse_tr = []
mse_te = []
for i in range(1, 7):
    z = np.polyfit(x_train, y_train, i)
    p = np.poly1d(z)
    print(p)

    y_tr_pre = p(x_train)
    mse_tr.append(mean_squared_error(y_train, y_tr_pre))
    y_te_pre = p(x_test)
    mse_te.append(mean_squared_error(y_te_real, y_te_pre))

    plt.subplot(2, 3, i)
    plt.scatter(x_train, y_tr_pre)
    plt.scatter(x_test, y_te_pre)
    plt.plot(x, y)

plt.show()
# 结果展示
print(mse_tr)
print(mse_te)
plt.subplot(1, 2, 1)
plt.plot(range(1, 7), mse_tr, '.-')
plt.title("train")
plt.subplot(1, 2, 2)
plt.plot(range(1, 7), mse_te, '.-')
plt.title("test")
plt.show()


# 梯度下降
def costFunc(X, Y, theta):  # 计算损失
    err = np.dot(X, theta.T) - Y
    return np.dot(err.T, err) / (2 * X.shape[0])


def gradientDescent(X, Y, theta, lr, iter):  # 梯度下降函数
    # cost = np.zeros(iter)
    cost = []
    while True:
        dJ = np.dot((np.dot(X, theta.T) - Y).T, X)
        if np.linalg.norm(dJ) <= 1e-4:
            break
        # print(dJ)
        theta = theta - dJ * lr
        # cost[n] = costFunc(X, Y, theta)
        cost.append(costFunc(X, Y, theta))
    return theta, cost


mse_tr = []
mse_te = []
# 对y做归一化
ytr = y_train
y_train = y_train.reshape(-1, 1)
ytmean = np.mean(y_train)
ytstd = np.std(y_train)
y_train = preprocessing.scale(y_train)

for i in range(1, 5):  # 为了较快展示结果，仅运行1-4阶，5、6阶速度较慢
    print(i)
    pf = PolynomialFeatures(degree=i)
    # 将x化成多项式矩阵形式 归一化
    x_train = np.array(x_train).reshape(-1, 1)
    x_train_quadratic = pf.fit_transform(x_train)
    xtqmean = np.mean(x_train_quadratic, axis=0)
    xtqstd = np.std(x_train_quadratic, axis=0)
    x_train_quadratic = preprocessing.scale(x_train_quadratic)

    # lin = LinearRegression()
    # lin.fit(x_train_quadratic, y_train)
    # print(lin.coef_)

    init = np.zeros((1, i + 1))
    model, cost = gradientDescent(x_train_quadratic, y_train, init, 0.01, 10000)
    model = np.squeeze(model)
    print(model)
    # 将参数反归一化
    poly = np.zeros(model.shape)
    for j in range(i + 1):
        if j == 0:
            add = 0
            for k in range(1, i + 1):
                add = add + ytstd / xtqstd[k] * xtqmean[k] * model[k]
            poly[j] = ytmean + ytstd * model[0] - add
        else:
            poly[j] = model[j] * ytstd / xtqstd[j]
    poly = np.flip(poly)
    p = np.poly1d(poly)
    print(p)
    # 展示迭代曲线
    cost = np.squeeze(np.array(cost))
    plt.plot(range(len(cost)), cost)
    plt.title("lr=0.01")
    plt.show()
    # # 计算均方差
    # y_tr_pre = p(x_train)
    # mse_tr.append(mean_squared_error(ytr, y_tr_pre))
    # y_te_pre = p(x_test)
    # mse_te.append(mean_squared_error(y_te_real, y_te_pre))
    # # 结果展示
    # plt.subplot(2, 3, i)
    # plt.scatter(x_train, y_tr_pre)
    # plt.scatter(x_test, y_te_pre)
    # plt.plot(x, y)
#
# plt.show()
# print(mse_tr)
# print(mse_te)
# plt.subplot(1, 2, 1)
# plt.plot(range(1, 7), mse_tr, 'b.-')
# plt.title("train")
# plt.subplot(1, 2, 2)
# plt.plot(range(1, 7), mse_te, 'r.-')
# plt.title("test")
# plt.show()
