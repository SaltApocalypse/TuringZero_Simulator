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


def depth_to_world(linear_depth_buffer, intrinsic_matrix, view_matrix, z_near, z_far, keep_ratio=0.2,user_data = None):
    """
    将线性深度缓冲转换为世界坐标，并随机保留部分点（在一开始降采样以减少计算量）
    :param linear_depth_buffer: 深度缓冲 (H, W)，值范围 [0, 1]
    :param intrinsic_matrix: 相机内参矩阵 (3, 3)
    :param view_matrix: 视图矩阵 (4, 4)
    :param z_near: 近裁剪面
    :param z_far: 远裁剪面
    :param keep_ratio: 保留点的比例 (0, 1)
    :return: 世界坐标点云 (N, 3)，其中 N 为有效像素的个数
    """
    # 获取深度缓冲的宽度和高度
    H, W = linear_depth_buffer.shape

    # 根据保留比例随机采样像素索引
    num_pixels = H * W
    num_sampled_pixels = int(num_pixels * keep_ratio)  # 计算采样后要保留的像素数
    sampled_indices = np.random.choice(num_pixels, size=num_sampled_pixels, replace=False)

    # 将一维索引映射回二维像素坐标
    sampled_y, sampled_x = np.unravel_index(sampled_indices, (H, W))

    # 获取采样的深度值
    sampled_depth_buffer = linear_depth_buffer[sampled_y, sampled_x]

    # 将深度缓冲值从 [0, 1] 转换为真实的深度值
    sampled_depth = z_near + sampled_depth_buffer * (z_far - z_near)

    # 将采样的像素坐标转换为相机坐标系下的齐次归一化设备坐标 (NDC)
    fx, fy = intrinsic_matrix[0, 0], intrinsic_matrix[1, 1]
    cx, cy = intrinsic_matrix[0, 2], intrinsic_matrix[1, 2]
    camera_x = (sampled_x - cx) / fx
    camera_y = (sampled_y - cy) / fy
    camera_z = sampled_depth

    # 将采样的像素坐标堆叠成 3D 点云 (N, 3)
    camera_points = np.stack([camera_x * sampled_depth, camera_y * sampled_depth, camera_z], axis=-1)

    # 将相机坐标系下的点云转换到世界坐标系
    # 首先将点扩展为齐次坐标 (N, 4)
    camera_points_homogeneous = np.concatenate([camera_points, np.ones((num_sampled_pixels, 1))], axis=-1)
    
    # 计算视图矩阵的逆矩阵
    inverse_view_matrix = np.linalg.inv(view_matrix)

    # 将点云从相机坐标系转换到世界坐标系
    world_points_homogeneous = camera_points_homogeneous @ inverse_view_matrix.T
    
    
    # rot_mat = la.mat_from_euler([-np.pi / 2, -np.pi / 2],order="YX")
    if user_data is not None:
        euler,order = user_data
        num = euler.split(",")
        for i in range(len(num)):
            num[i] = float(num[i])
        rot_mat = la.mat_from_euler(num,order=order)
        world_points_homogeneous = world_points_homogeneous @ rot_mat

    # 去掉齐次坐标，得到世界坐标 (N, 3)
    world_points = world_points_homogeneous[..., :3]
    
    return world_points


def compute_intrinsic_matrix(cam_intrinsic):

    fx, fy, cx, cy = cam_intrinsic
    intrinsic_matrix = np.array([
        [fx, 0,  cx],
        [0,  fy, cy],
        [0,  0,  1]
    ])
    return intrinsic_matrix

def compute_view_matrix(cam_pos, cam_quat):

    R = la.mat_from_quat(cam_quat)
    T = la.mat_from_translation(cam_pos)
    M = T @ R
    view = np.linalg.inv(M)

    return view