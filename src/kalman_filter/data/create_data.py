# import numpy as np
# from scipy.ndimage import gaussian_filter1d
# import matplotlib.pyplot as plt

# # 生成200个在[-2, 2]范围内的随机数据
# data = np.random.uniform(-1, 2, 200)

# # 高斯平滑
# smoothed_data = gaussian_filter1d(data, sigma=3)

# # 将平滑后的数据写入本地NumPy文件
# np.save('accelerations.npy', np.array(smoothed_data))

# load_data = np.load(f'./accelerations.npy')
# i=0
# while i<len(load_data):
#     print(f'{load_data[i]}')
#     i += 1
# 直接用python运行，命令：python generate_random_a.py
import numpy as np
import scipy.ndimage

# 生成随机加速度序列
n = 200  # 序列长度
random_accelerations = np.random.randn(n) * 2  # 生成随机加速度

# 应用高斯滤波进行平滑，sigma 控制平滑程度
smoothed_accelerations = scipy.ndimage.gaussian_filter(random_accelerations, sigma=0.8)

print(smoothed_accelerations)
np.save('accelerations.npy', np.array(smoothed_accelerations))
