import numpy as np
import matplotlib.pyplot as plt

# 读取数据
def read_data(file_path):
    data = np.loadtxt(file_path)
    time = data[:, 0]
    measurements = data[:, 1]
    return time, measurements

# 卡尔曼滤波实现
def kalman_filter(data, process_variance, measurement_variance):
    n = len(data)
    x_est = np.zeros(n)  # 状态估计值
    p_est = np.zeros(n)  # 估计协方差

    # 初始化
    x_est[0] = data[0]  # 初始状态
    p_est[0] = 1.0  # 初始协方差

    for k in range(1, n):
        # 预测步骤
        x_pred = x_est[k - 1]  # 状态预测
        p_pred = p_est[k - 1] + process_variance  # 协方差预测

        # 更新步骤
        kalman_gain = p_pred / (p_pred + measurement_variance)  # 卡尔曼增益
        x_est[k] = x_pred + kalman_gain * (data[k] - x_pred)  # 更新状态估计
        p_est[k] = (1 - kalman_gain) * p_pred  # 更新协方差

    return x_est



# 绘制曲线
def plot_results(time, measurements, filtered_data):
    plt.figure(figsize=(10, 6))
    plt.plot(time, measurements, label="original data",  marker=".", markersize=1, ls='')
    plt.plot(time, filtered_data, label="kalman", linewidth=1)
    plt.xlabel("time")
    plt.ylabel("measurement")
    plt.title("Kalman Filter and Curve Fitting")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    # 文件路径
    file_path = "../data/kalman/homework_data_1.txt"

    # 读取数据
    time, measurements = read_data(file_path)

    # 卡尔曼滤波参数
    process_variance = 1e-5  # 过程噪声方差
    measurement_variance = 1e-2  # 测量噪声方差

    # 应用卡尔曼滤波
    filtered_data = kalman_filter(measurements, process_variance, measurement_variance)


    # 绘制结果
    plot_results(time, measurements, filtered_data)