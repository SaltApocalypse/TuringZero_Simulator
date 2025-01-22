import numpy as np
import pylinalg as la


def nonlinear_to_linear_depth(non_linear_depth, z_near, z_far):
    """
    将非线性深度值转换为线性深度值。

    :param non_linear_depth: 非线性深度值 (0 到 1)
    :param z_near: 近平面距离
    :param z_far: 远平面距离
    :return: 线性深度值
    """

    # 使用公式进行转换
    linear_depth = (2.0 * z_near * z_far) / (z_far + z_near - non_linear_depth * (z_far - z_near))
    return linear_depth


def camera_matrix(size, fovy):
    width, height = size
    f = 0.5 * height / np.tan(fovy * np.pi / 360)
    res = np.array(((-f, 0, width / 2), (0, f, height / 2), (0, 0, 1)))
    return res


def depth_to_point_cloud(linear_depth_map, fovy, quat, pos, kppe_rate=0.01):
    """
    将深度图转换为3D点云

    :param linear_depth_map: 线性深度图 (H, W)，值范围为 0 到 1
    :param fovy: 垂直视场角 (单位: 度)
    :param quat: 相机的四元数姿态
    :param pos: 相机的位置 (3,)
    :param kppe_rate: 保留点的比例，范围为 0 到 1，默认为 1.0 (保留所有点)
    :return: 3D点云 (N, 3)，N 为点的数量
    """
    # 获取图像宽度和高度
    height, width = linear_depth_map.shape  # 注意这里的顺序

    # 构造相机内参矩阵
    K = camera_matrix((width, height), fovy)
    R = la.mat_from_quat(quat)[:3, :3]
    t = pos.reshape(3, 1)

    # 计算相机内参矩阵的逆
    K_inv = np.linalg.inv(K)

    # 创建像素坐标网格
    x = np.arange(0, width)
    y = np.arange(0, height)
    xv, yv = np.meshgrid(x, y)  # xv 和 yv 的形状是 (height, width)

    # 将像素坐标与深度值结合，形成齐次图像坐标
    image_coords = np.stack((xv, yv, np.ones_like(xv)), axis=-1)  # (height, width, 3)

    # 将齐次图像坐标映射到相机坐标系
    camera_coords = np.dot(image_coords, K_inv.T)  # (height, width, 3)

    # 乘以线性深度值
    camera_coords *= linear_depth_map[:, :, None]  # 广播机制正常工作 (height, width, 1)
    camera_coords[..., 2] *= -1  # 反转 Z 轴方向
    camera_coords = camera_coords[..., [1, 0, 2]]

    # 将相机坐标展平，形成点云
    point_cloud = camera_coords.reshape(-1, 3)  # (N, 3)

    # 转换为世界坐标
    point_cloud = (R @ point_cloud.T + t).T

    # 随机减少总点数
    num_points = point_cloud.shape[0]  # 总点数
    mask = np.random.choice([False, True], size=num_points, p=[1 - kppe_rate, kppe_rate])  # 生成布尔掩码
    filtered_point_cloud = point_cloud[mask]  # 使用掩码保留点

    distances = np.linalg.norm(filtered_point_cloud - pos, axis=1)
    distance_mask = distances < 9.9
    filtered_point_cloud = filtered_point_cloud[distance_mask]

    return filtered_point_cloud
