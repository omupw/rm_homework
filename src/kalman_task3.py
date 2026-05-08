import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import pandas as pd

def kalman_filter(data, process_variance, measurement_variance):
    n = len(data)
    # 初始化
    x = np.zeros(6)  # 状态向量 [位置, 速度]
    p = np.eye(6)  # 协方差矩阵
    H = np.array([
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 观测 x
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],  # 观测 y
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]   # 观测 z
    ])  # 观测矩阵
    F = np.array([[1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0, 0.0, 0.0, 1.0],
                  [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])  # 状态转移矩阵
    Q = np.array([[1/20.0, 0.0, 0.0, 1/8.0, 0.0, 0.0],
                  [0.0, 1/20.0, 0.0, 0.0, 1/8.0, 0.0],
                  [0.0, 0.0, 1/20.0, 0.0, 0.0, 1/8.0],
                  [1/8.0, 0.0, 0.0, 1/3.0, 0.0, 0.0],
                  [0.0, 1/8.0, 0.0, 0.0, 1/3.0, 0.0],
                  [0.0, 0.0, 1/8.0, 0.0, 0.0, 1/3.0]]) * process_variance
    R = np.eye(3) * measurement_variance  # 测量噪声协方差
    results = np.zeros((n,3))
    for k in range(n):
        # 预测步骤
        x = F @ x  # 状态预测
        p = F @ p @ F.T + Q  # 协方差预测
        # 更新步骤
        kalman_gain = p @ H.T  @ np.linalg.inv(H @ p @ H.T + R)  # 卡尔曼增益
        x = x + kalman_gain @ (data[k] - H @ x)  # 更新状态估计
        p = (np.eye(6) - kalman_gain @ H) @ p  # 更新协方差
        results[k] = x[:3]  # 记录滤波结果
    return results

# Load measurement data
measurement_data = pd.read_csv("../data/kalman/measurements.csv")
measurements = measurement_data.iloc[:, :3].values  # Extract first three columns (x, y, z)

# Load true states data
true_states_data = pd.read_csv("../data/kalman/true_states.csv")
true_states = true_states_data.iloc[:, :3].values  # Extract first three columns (x, y, z)

# Define process and measurement variance
process_variance = 1e-3
measurement_variance = 1e-2

# Apply Kalman filter to measurements
estimated_states = kalman_filter(measurements, process_variance, measurement_variance)

# Plot the results
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot measurements as points
ax.scatter(measurements[:, 0], measurements[:, 1], measurements[:, 2], c='blue', label='Measurements', marker='o',s=10)

# Plot estimated states as a line
ax.plot(estimated_states[:, 0], estimated_states[:, 1], estimated_states[:, 2], c='red', label='Estimated States')

# Plot true states as a line
ax.plot(true_states[:, 0], true_states[:, 1], true_states[:, 2], c='green', label='True States')

# Add labels and legend
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.legend()

# Create sliders for process and measurement noise
fig = plt.figure(figsize=(10, 8))
ax_slider1 = plt.axes([0.2, 0.02, 0.65, 0.03])
ax_slider2 = plt.axes([0.2, 0.06, 0.65, 0.03])

slider_process = Slider(ax_slider1, 'Process Noise', 1e-5, 1e-1, valinit=process_variance, valstep=1e-5)
slider_measurement = Slider(ax_slider2, 'Measurement Noise', 1e-5, 1e-1, valinit=measurement_variance, valstep=1e-5)

def update(val):
    process_variance = slider_process.val
    measurement_variance = slider_measurement.val
    estimated_states = kalman_filter(measurements, process_variance, measurement_variance)

    ax.cla()
    ax.scatter(measurements[:, 0], measurements[:, 1], measurements[:, 2], c='blue', label='Measurements', marker='o',s=10)
    ax.plot(estimated_states[:, 0], estimated_states[:, 1], estimated_states[:, 2], c='red', label='Estimated States')
    ax.plot(true_states[:, 0], true_states[:, 1], true_states[:, 2], c='green', label='True States')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.legend()
    fig.canvas.draw_idle()

slider_process.on_changed(update)
slider_measurement.on_changed(update)

# Show the plot
plt.show()