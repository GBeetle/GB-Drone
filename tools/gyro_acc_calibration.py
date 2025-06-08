import os
import pandas as pd
import numpy as np
import chardet

# 自动检测文件编码
def detect_encoding(file_path):
    with open(file_path, 'rb') as f:
        result = chardet.detect(f.read(10000))
        return result['encoding']

# 读取 CSV 并提取相关字段
def load_sensor_data(file_path):
    encoding = detect_encoding(file_path)
    df = pd.read_csv(file_path, skiprows=1, encoding=encoding)
    return df[['FC_ACC_X', 'FC_ACC_Y', 'FC_ACC_Z', 'FC_GYR_X', 'FC_GYR_Y', 'FC_GYR_Z']]

# 根据加速度均值识别哪个轴是1g方向
def identify_axis_direction(acc_mean):
    axis_labels = ['x', 'y', 'z']
    max_idx = np.argmax(np.abs(acc_mean))
    axis = axis_labels[max_idx]
    direction = '+' if acc_mean[max_idx] > 0 else '-'
    return axis, direction

# 陀螺仪偏置校准（静止时角速度为0）
def calibrate_gyroscope(df_list):
    all_gyro_data = np.vstack([df[['FC_GYR_X', 'FC_GYR_Y', 'FC_GYR_Z']].values for df in df_list])
    gyro_bias = np.mean(all_gyro_data, axis=0)
    return gyro_bias

# 加速度计偏置和缩放校准
def calibrate_accelerometer(pos_series, neg_series):
    pos_avg = pos_series.mean()
    neg_avg = neg_series.mean()
    scale = (pos_avg - neg_avg) / 2
    bias = (pos_avg + neg_avg) / 2
    return scale, bias

# 主流程
def main(data_dir):
    acc_grouped = {'x': {}, 'y': {}, 'z': {}}
    all_dfs = []

    for filename in os.listdir(data_dir):
        if not filename.endswith('.csv'):
            continue
        filepath = os.path.join(data_dir, filename)
        df = load_sensor_data(filepath)
        all_dfs.append(df)

        acc_mean = df[['FC_ACC_X', 'FC_ACC_Y', 'FC_ACC_Z']].mean().values
        axis, direction = identify_axis_direction(acc_mean)
        acc_grouped[axis][direction] = df[f'FC_ACC_{axis.upper()}']

    # 校准陀螺仪
    gyro_bias = calibrate_gyroscope(all_dfs)
    print("\nGyroscope Bias (X, Y, Z):", gyro_bias)

    # 校准加速度计
    acc_scales = {}
    acc_biases = {}
    for axis in ['x', 'y', 'z']:
        pos = acc_grouped[axis].get('+')
        neg = acc_grouped[axis].get('-')
        if pos is not None and neg is not None:
            scale, bias = calibrate_accelerometer(pos, neg)
            acc_scales[axis] = scale
            acc_biases[axis] = bias
        else:
            print(f"缺少 {axis}-axis 的 +1g 或 -1g 数据，跳过该轴校准")

    print("\nAccelerometer Scale (X, Y, Z):", [acc_scales.get(a, None) for a in ['x', 'y', 'z']])
    print("Accelerometer Bias  (X, Y, Z):", [acc_biases.get(a, None) for a in ['x', 'y', 'z']])

if __name__ == '__main__':
    main('./gyro_acc_calibration_data')
