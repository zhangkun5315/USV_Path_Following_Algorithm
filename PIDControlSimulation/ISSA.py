'''
Combine the improved sparrow search algorithm (ISSA).
    The Optimization of PID as below:
        1. intelligent to choose PID parameters using ISSA.

        2. 生产者更新策略调整.

        3. design the fitness function of PID

        4. 种群初始化引入 Tent 混沌映射。

        9. 多次执行ISSA取最优。再给经验条件，例如 Kp > 5*Ki， Kp > 5*Kd
        9. 例程中给的 Δt 的作用也仅仅是包含在计算PID上，可以删除，我们需要自己安排一个control循环时间，相对固定，让PID参数选择更加稳定。



    1200米*800米  海况风速为0-3级，流速为0-0.2米/秒。
'''

# 载入所需的包
import copy
import random
import numpy as np
import matplotlib.pyplot as plt
import t_TestFunction as tF  # 测试函数
import t_levy_distrubution as levy


''' 种群初始化函数 '''  # Tent chaos mapping.
def tent_map(x, a, num_points):
    if x < a:
        return x/a + np.random.rand()/num_points
    else:
        return (1-x)/(1-a) + np.random.rand()/num_points

def generate_chaos(num_points):
    chaos = []
    x = np.random.rand()
    for i in range(num_points):
        x = tent_map(x, 0.5, num_points)
        if x >= 1:
            x -= 1
        chaos.append(x)

    return chaos

''' 种群初始化函数 '''  # Original
def initial_Tent(pop, dim, lb, ub):
    X = np.zeros([pop, dim])
    for i in range(dim):
        chaos = generate_chaos(pop)
        for j in range(pop):
            X[j, i] = chaos[j] * (ub[i] - lb[i]) + lb[i]
    return X


'''边界检查函数'''
def BorderCheck(X, lb, ub, pop, dim):
    for i in range(pop):
        for j in range(dim):
            if X[i, j] > ub[j]:
                X[i, j] = np.random.rand() * (ub[j] - lb[j]) + lb[j]
            elif X[i, j] < lb[j]:
                X[i, j] = np.random.rand() * (ub[j] - lb[j]) + lb[j]
    return X


'''计算适应度函数'''
def CaculateFitness(X, fun):
    pop = X.shape[0]
    fitness = np.zeros([pop, 1])
    for i in range(pop):
        fitness[i] = fun(X[i, :])
    return fitness


'''适应度排序'''
def SortFitness(Fit):
    fitness = np.sort(Fit, axis=0)
    index = np.argsort(Fit, axis=0)
    return fitness, index


'''根据适应度对位置进行排序'''
def SortPosition(X, index):
    Xnew = np.zeros(X.shape)
    for i in range(X.shape[0]):
        Xnew[i, :] = X[index[i], :]
    return Xnew

def distance_avg_producers(X, PDNumber):
    X_new = copy.copy(X)

    sum_distance = 0

    for p in range(PDNumber):  # 和当前最优位置麻雀的距离和。
        sum_distance += np.sqrt((X_new[p, 0]-X_new[0, 0])**2 + (X_new[p, 1]-X_new[0, 1])**2)

    return sum_distance/PDNumber


'''麻雀生产者勘探更新#  Improve: 改到该大步走大步走，该小步走小步走，随机危险改成生产者集中危险。'''
def PDUpdate(X, PDNumber, e_para, Max_iter, dim, lb, ub):
    X_new = copy.copy(X)

    d_avg = distance_avg_producers(X, PDNumber)  # 生产者和最优值的平均距离。
    d_left = d_avg/np.sqrt((ub[0]-lb[0])**2 + (ub[1]-lb[1])**2)  # 归一化。

    for p in range(PDNumber):
        for j in range(dim):
            if d_left > e_para:  # 此时发现危险，生产者太集中了，容易陷入局部最优。
                X_new[p, j] = X[p, j] + X[p, j] * (np.exp(-p / ((random.random()) * Max_iter)) - 1) * (
                np.exp(-6 * (np.abs(X[p, j]) / max(np.abs(ub[0]), np.abs(lb[0])))))

            else:
                X_new[p, j] = X[p, j] + np.random.normal(loc=0.0, scale=(ub[0]-lb[0])/4)
                # 第一维和第二维可能不同步。为什么会出现x一致，y分散了呢？
                # 这里我们要有一个原则， 例如 31.63% 的生产者应当左右分别跳出最大范围的(ub-lb)/4
    return X_new


'''麻雀scrounger更新'''
def JDUpdate(X, PDNumber, pop, dim):
    X_new = copy.copy(X)
    # 产生-1，1的随机数
    A = np.ones([dim, 1])
    for a in range(dim):
        if (random.random() > 0.5):
            A[a] = -1

    for i in range(PDNumber + 1, pop):
        for j in range(dim):
            if i > (pop - PDNumber) / 2 + PDNumber:  # 适应度最差的一些，SD中的后一半
                difff = X[-1, j] - X[i, j]
                if difff > 2000:  #####################################################
                    difff = 2000
                X_new[i, j] = np.random.randn() * np.exp(difff / i ** 2)
            else:
                AA = np.mean(np.abs(X[i, :] - X[0, :]) * A)
                X_new[i, j] = X[0, j] - AA
    return X_new


'''危险更新'''
def SDUpdate(X, pop, SDNumber, fitness, BestF):
    X_new = copy.copy(X)
    dim = X.shape[1]
    Temp = range(pop)
    RandIndex = random.sample(Temp, pop)
    SDchooseIndex = RandIndex[0:SDNumber]  # 从全局里面随机选出来20%的危险意识者。

    for i in range(SDNumber):
        for j in range(dim):
            if fitness[SDchooseIndex[i]] > BestF:
                X_new[SDchooseIndex[i], j] = X[0, j] + np.random.randn() * np.abs(X[SDchooseIndex[i], j] - X[0, j])
            elif fitness[SDchooseIndex[i]] == BestF:
                K = 2 * random.random() - 1
                X_new[SDchooseIndex[i], j] = X[SDchooseIndex[i], j] + K * (
                    np.abs(X[SDchooseIndex[i], j] - X[-1, j]) / (fitness[SDchooseIndex[i]] - fitness[-1] + 10E-8))
    return X_new



'''变异策略'''
def BLMUpdate(X, fitness, pop, dim, ub, lb, t, Max_iter, func):
    # Before next iteration, 引入布朗-莱维突变策略 update X.★

    favg = 0
    for v in fitness:
        favg += v
    favg = favg / pop
    #
    r1 = 1 - t**2/Max_iter**2; r2 = t**2/Max_iter**2
    #
    X_new = copy.copy(X)
    #
    for i in range(pop):
        if fitness[i] < favg:
            for j in range(dim):
                X_new[i, j] = X[i, j] * (1 + r1*np.random.normal(0, 1))
        if fitness[i] >= favg:
            for j in range(dim):
                X_new[i, j] = X[i, j] * (1 + r2*levy.levy_random_generate(beta=3/2))

    X_new = BorderCheck(X_new, ub, lb, pop, dim)  # 边界检测
    fitness_temp = CaculateFitness(X_new, func)  # 计算适应度值

    for i in range(pop):
        if fitness_temp[i] < fitness[i]:
            fitness[i] = fitness_temp[i]
            for j in range(dim):
                X[i, j] = X_new[i, j]

    return X, fitness



'''麻雀搜索算法'''
def ISSA(pop, dim, lb, ub, Max_iter, func):
    # ST = 0.8  # 预警值
    e_para = 1e-6
    PD = 0.3  # 发现者的比，剩下的是追随者 producer
    SD = 0.1  # 意识到有危险麻雀的比重

    PDNumber = int(pop * PD)  # 发现者数量
    SDNumber = int(pop * SD)  # 意识到有危险麻雀数量
    X = initial_Tent(pop, dim, lb, ub)  # 初始化种群

    fitness = CaculateFitness(X, func)  # 计算适应度值
    fitness, sortIndex = SortFitness(fitness)  # 对适应度值排序
    X = SortPosition(X, sortIndex)  # 种群排序
    GbestScore = copy.copy(fitness[0])
    GbestPositon = np.zeros([1, dim])
    GbestPositon[0, :] = copy.copy(X[0, :])
    Curve = np.zeros([Max_iter, 1])

    for i in range(Max_iter):
        print('ISSA Iteration: {}'.format(i + 1))

        BestF = fitness[0]

        X = PDUpdate(X, PDNumber, e_para, Max_iter, dim, lb, ub)  # 发现者更新

        X = JDUpdate(X, PDNumber, pop, dim)  # 追随者更新

        X = SDUpdate(X, pop, SDNumber, fitness, BestF)  # 危险更新

        X = BorderCheck(X, lb, ub, pop, dim)  # 边界检测

        fitness = CaculateFitness(X, func)  # 计算适应度值


        # 这里开始加入 布朗-莱维变异策略
        X, fitness = BLMUpdate(X, fitness, pop, dim,ub, lb, i, Max_iter, func)



        fitness, sortIndex = SortFitness(fitness)  # 对适应度值排序
        X = SortPosition(X, sortIndex)  # 种群排序
        if (fitness[0] <= GbestScore):  # 更新全局最优
            GbestScore = copy.copy(fitness[0])
            GbestPositon[0, :] = copy.copy(X[0, :])

        Curve[i] = GbestScore

        print(X[0], fitness[0], GbestScore)
        # 动图显示麻雀所在点。。。★★★
        # points, = plt.plot(X[:, 0], X[:, 1], 'ob')
        # plt.xlim(lb[0], ub[0]); plt.ylim(lb[1], ub[1])
        # plt.pause(0.3)
        # points.remove()

    return GbestScore, GbestPositon, Curve





if __name__ == '__main__':
    Max_iter = 500
    dim = 30
    lb = [-920 for i in range(dim)]
    ub = [80 for i in range(dim)]

    GbestScore, GbestPositon, Curve = ISSA(100, dim, lb, ub, Max_iter, tF.func8_0)
    print(GbestScore, GbestPositon)
    print('Best position: ', tF.func8('best'))


    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(range(Max_iter), Curve, 'r')
    ax2.set_xlim(0, Max_iter)
    # ax2.set_ylim(lb[1], ub[1])
    plt.show()



    exit()