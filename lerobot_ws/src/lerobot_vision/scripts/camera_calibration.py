#!/usr/bin/env python3
"""相机标定工具 (棋盘格法)

Usage:
    python3 camera_calibration.py --rows 9 --cols 6 --square 0.025

输出:
    相机内参矩阵 + 畸变系数 → 写入 vision_config.yaml
"""

import argparse
import cv2
import numpy as np
import yaml
import os


def main():
    parser = argparse.ArgumentParser(description='相机标定')
    parser.add_argument('--rows', type=int, default=9, help='棋盘格行内角数')
    parser.add_argument('--cols', type=int, default=6, help='棋盘格列内角数')
    parser.add_argument('--square', type=float, default=0.025, help='格子边长(米)')
    parser.add_argument('--output', type=str, default='calibration_result.yaml')
    parser.add_argument('--device', type=int, default=0, help='相机设备ID')
    args = parser.parse_args()

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.rows, 0:args.cols].T.reshape(-1, 2) * args.square

    objpoints, imgpoints = [], []

    cap = cv2.VideoCapture(args.device)
    print("按 's' 拍照, 按 'q' 退出并计算, 至少10张")

    count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.putText(gray, f'Collected: {count}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, 0)
        cv2.imshow('Calibration', gray)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            ret, corners = cv2.findChessboardCorners(gray, (args.rows, args.cols), None)
            if ret:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                objpoints.append(objp)
                imgpoints.append(corners2)
                count += 1
                print(f'采集第 {count} 张')
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if count < 10:
        print('图片不足，至少需要10张')
        return

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    result = {
        'camera_matrix': {'data': mtx.tolist()},
        'dist_coeffs': {'data': dist.tolist()},
        'reprojection_error': float(ret),
    }

    with open(args.output, 'w') as f:
        yaml.dump(result, f, default_flow_style=True)

    print(f'标定完成, 重投影误差: {ret:.4f}')
    print(f'内参:\n{mtx}')
    print(f'畸变:\n{dist}')
    print(f'结果已保存到: {args.output}')


if __name__ == '__main__':
    main()
