import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import pandas as pd

# 卡尔曼滤波实现
def kalman_filter(data, process_variance, measurement_variance):
    n = len(data)
    # 初始化
    x = np.array([[data[0]], [0.0]])  # 状态向量 [位置, 速度]
    p = np.array([[1.0, 0.0], [0.0, 1.0]])  # 协方差矩阵
    H = np.array([[1.0, 0.0]])  # 观测矩阵
    F = np.array([[1.0, 1.0], [0.0, 1.0]])  # 状态转移矩阵
    Q = np.array([[1/3.0*process_variance, 0.5*process_variance],
                  [0.5*process_variance, process_variance]])  # 过程噪声协方差
    R = measurement_variance  # 测量噪声协方差
    results = np.zeros(n)
    for k in range(n):
        # 预测步骤
        x = F @ x  # 状态预测
        p = F @ p @ F.T + Q  # 协方差预测
        # 更新步骤
        kalman_gain = p @ H.T / (H @ p @ H.T + R)  # 卡尔曼增益
        x = x + kalman_gain * (data[k] - H @ x)  # 更新状态估计
        p = (np.eye(2) - kalman_gain @ H) @ p  # 更新协方差
        results[k] = (H @ x)[0, 0]  # 记录滤波结果

    return results

# 绘制曲线
def plot_results(days, prices, filtered_data, update_func):
    fig, ax = plt.subplots(figsize=(10, 6))
    plt.subplots_adjust(left=0.25, bottom=0.25)

    # 初始绘图
    original_line, = ax.plot(days, prices, label="Original Prices", marker=".", markersize=4, ls='')
    kalman_line, = ax.plot(days, filtered_data, label="Kalman Filtered", linewidth=1)

    ax.set_xlabel("Days")
    ax.set_ylabel("Price")
    ax.set_title("Stock Prices with Kalman Filter")
    ax.legend()
    ax.grid()

    # 添加滑块
    ax_process = plt.axes([0.25, 0.1, 0.65, 0.03])
    ax_measurement = plt.axes([0.25, 0.15, 0.65, 0.03])

    process_slider = Slider(ax_process, 'Process Noise', 1e-5, 1e-1, valinit=1e-3, valstep=1e-5)
    measurement_slider = Slider(ax_measurement, 'Measurement Noise', 1e-5, 1e-1, valinit=1e-2, valstep=1e-5)

    # 更新函数
    def update(val):
        process_variance = process_slider.val
        measurement_variance = measurement_slider.val
        updated_filtered_data = update_func(process_variance, measurement_variance)
        kalman_line.set_ydata(updated_filtered_data)
        fig.canvas.draw_idle()

    process_slider.on_changed(update)
    measurement_slider.on_changed(update)

    plt.show()

if __name__ == "__main__":
    # 读取CSV数据
    file_path = "../data/kalman/stock_prices.csv"
    data = pd.read_csv(file_path)
    days = data['Day'].values
    prices = data['Price'].values

    # 卡尔曼滤波参数
    initial_process_variance = 1e-3  # 初始过程噪声方差
    initial_measurement_variance = 1e-2  # 初始测量噪声方差

    # 应用卡尔曼滤波
    def update_filtered_data(process_variance, measurement_variance):
        return kalman_filter(prices, process_variance, measurement_variance)

    filtered_data = update_filtered_data(initial_process_variance, initial_measurement_variance)

    # 绘制结果
    plot_results(days, prices, filtered_data, update_filtered_data)