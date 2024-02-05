import numpy as np
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt

# 生成200个在[-2, 2]范围内的随机数据
data = np.random.uniform(-2, 2, 200)

# 高斯平滑
smoothed_data = gaussian_filter1d(data, sigma=3)

# 将平滑后的数据写入本地NumPy文件
np.save('accelerations.npy', np.array(smoothed_data))

load_data = np.load(f'./accelerations.npy')
i=0
while i<len(load_data):
    print(f'{load_data[i]}')
    i += 1
