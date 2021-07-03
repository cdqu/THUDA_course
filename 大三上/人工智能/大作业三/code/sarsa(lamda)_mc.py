# SARSA(lambda) 离散

import numpy as np
import matplotlib.pyplot as plt
import gym

# 导入环境
env = gym.make('MountainCar-v0')
env.reset()
# 参数设置
lr = 0.4
gamma = 0.9
lamda = 0.8
episode = 8000
qt_len = 20
# 初始化q表
state_size = [qt_len] * 2
state_discrete = (env.observation_space.high - env.observation_space.low) / state_size  # 状态空间离散化
# qtable = np.random.uniform(low=0, high=1,
#                             size=(state_size + [env.action_space.n]))
q_table = np.zeros(state_size + [env.action_space.n])
e_trace = np.zeros((state_size + [env.action_space.n]))
# epsilon设置，学习过中动态变化
epsilon = 1
START = 1
END = episode // 2
epsilon_decay = epsilon / (END - START)


# epsilon贪心策略
def take_eg_action(state, epsilon):
    state_ind = get_state_ind(state)
    if np.random.random() < epsilon:
        action = np.random.randint(0, env.action_space.n)
    else:
        action = np.argmax(q_table[state_ind])
    return action


# 获取状态索引
def get_state_ind(state):
    state_ind = (state - env.observation_space.low) // state_discrete
    return tuple(state_ind.astype(int))


# 记录回报，便于后续分析
ep_rewards = []
aggr_ep_rewards = {'ep': [], 'avg': [], 'min': [], 'max': []}

# 训练
for ep in range(episode):
    # 初始化
    ep_reward = 0
    state = env.reset()
    action = take_eg_action(state, epsilon)
    # E表清零
    e_trace = np.zeros((state_size + [env.action_space.n]))

    done = False

    while not done:
        next_state, reward, done, info = env.step(action)
        ep_reward += reward
        next_action = take_eg_action(next_state, epsilon)

        if not done:  # 未成功，使用SARSA(lamda)算法更新Q表
            q_real = reward + gamma * q_table[get_state_ind(next_state)][next_action] - q_table[get_state_ind(state)][
                action]

            e_trace[get_state_ind(state)][action] += 1

            q_table += lr * q_real * e_trace
            e_trace = gamma * lamda * e_trace

        elif next_state[0] >= 0.5:  # 成功抵达
            q_table[get_state_ind(state)][action] = 0

        state = next_state  # 更新状态
        action = next_action  # 按预测执行

    # 如果在限制的episode范围内成功，减小epsilon
    if START <= ep <= END:
        epsilon -= epsilon_decay

    ep_rewards.append(ep_reward)
    if ep % 200 == 0:
        avg_reward = sum(ep_rewards[-200:]) / len(ep_rewards[-200:])
        aggr_ep_rewards['ep'].append(ep)
        aggr_ep_rewards['avg'].append(avg_reward)
        aggr_ep_rewards['min'].append(min(ep_rewards[-200:]))
        aggr_ep_rewards['max'].append(max(ep_rewards[-200:]))


# 绘图
plt.plot(aggr_ep_rewards['ep'], aggr_ep_rewards['avg'], label='avg')
plt.plot(aggr_ep_rewards['ep'], aggr_ep_rewards['min'], label='min')
plt.plot(aggr_ep_rewards['ep'], aggr_ep_rewards['max'], label='max')
plt.legend(loc='upper left')
plt.xlabel('Episodes')
plt.ylabel('Rewards')
plt.show()

# 测试
done = False
state = env.reset()
while not done:
    action = np.argmax(q_table[get_state_ind(state)])
    next_state, _, done, _ = env.step(action)
    state = next_state
    env.render()

env.close()