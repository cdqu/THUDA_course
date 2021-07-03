# Q-learning 连续

import gym
import numpy as np
import matplotlib.pyplot as plt

# 导入环境
env = gym.make('MountainCarContinuous-v0')
env.reset()
# 参数设置
lr = 0.5
gamma = 0.7
episode = 8000
qt_len = 200
# 将动作空间离散化
action_space = np.array(range(-10, 11, 1)) / 10
action_space = action_space.reshape(len(action_space), 1)
# 初始化q表
state_size = [qt_len] * 2
state_discrete = (env.observation_space.high - env.observation_space.low) / state_size
q_table = np.random.uniform(low=0, high=1, size=(state_size + [len(action_space)]))

# epsilon设置，学习过中动态变化
epsilon = 1
START = 1
END = episode // 2
epsilon_decay = (epsilon - 0.1) / (END - START)


def get_state_ind(state):
    state_ind = (state - env.observation_space.low) // state_discrete
    return tuple(state_ind.astype(int))


# epsilon贪心策略
def take_eg_action(state, epsilon):
    discrete_state = get_state_ind(state)
    if np.random.random() < epsilon:
        action_ind = np.random.randint(0, len(action_space))
    else:
        action_ind = np.argmax(q_table[discrete_state])
    return action_ind, action_space[action_ind]


ep_rewards = []
aggr_ep_rewards = {'ep': [], 'avg': [], 'min': [], 'max': []}

for ep in range(episode):
    # 初始化
    ep_reward = 0
    state = env.reset()
    done = False

    while not done:
        action_ind, action = take_eg_action(state, epsilon)

        next_state, reward, done, info = env.step(action)

        reward = reward + abs(next_state[1] - abs(state[1])) * 50 * gamma  # 回报

        ep_reward += reward

        if not done:
            q_real = reward + gamma * np.max(q_table[get_state_ind(next_state)])
            q_table[get_state_ind(state)][action_ind] += lr * (
                    q_real - q_table[get_state_ind(state)][action_ind])

        elif next_state[0] >= 0.45:
            # print("I made it on episode: {} Reward: {}".format(ep, reward))
            q_table[get_state_ind(state)][action_ind] = 0

        state = next_state

    # 如果在限制的episode范围内成功，减小epsilon
    if START <= ep <= END:
        epsilon = epsilon - epsilon_decay

    ep_rewards.append(ep_reward)

    if ep % 200 == 0:
        avg_reward = sum(ep_rewards[-200:]) / len(ep_rewards[-200:])
        aggr_ep_rewards['ep'].append(ep)
        aggr_ep_rewards['avg'].append(avg_reward)
        aggr_ep_rewards['min'].append(min(ep_rewards[-200:]))
        aggr_ep_rewards['max'].append(max(ep_rewards[-200:]))

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
