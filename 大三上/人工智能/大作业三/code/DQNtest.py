# DQN训练模型测试
# 离散
import time
import gym
import numpy as np
from keras import models
env = gym.make('MountainCar-v0')
model = models.load_model('dqn_1.model')
s = env.reset()
score = 0
while True:
    env.render()
    time.sleep(0.01)
    a = np.argmax(model.predict(np.array([s]))[0])
    s, reward, done, _ = env.step(a)
    score += reward
    if done:
        print('score:', score)
        break
env.close()