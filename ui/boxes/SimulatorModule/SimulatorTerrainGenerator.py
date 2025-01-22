"""
This file was modified from "unitreerobotics/unitree_mujoco"
Repo: https://github.com/unitreerobotics/unitree_mujoco/tree/main/terrain_tool
"""

from os.path import dirname, join
import xml.etree.ElementTree as xml_et
import numpy as np
import cv2
import noise
import random


# zyx euler angle to quaternion
def euler_to_quat(roll, pitch, yaw):
    cx = np.cos(roll / 2)
    sx = np.sin(roll / 2)
    cy = np.cos(pitch / 2)
    sy = np.sin(pitch / 2)
    cz = np.cos(yaw / 2)
    sz = np.sin(yaw / 2)

    return np.array(
        [
            cx * cy * cz + sx * sy * sz,
            sx * cy * cz - cx * sy * sz,
            cx * sy * cz + sx * cy * sz,
            cx * cy * sz - sx * sy * cz,
        ],
        dtype=np.float64,
    )


# zyx euler angle to rotation matrix
def euler_to_rot(roll, pitch, yaw):
    rot_x = np.array(
        [
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ],
        dtype=np.float64,
    )

    rot_y = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ],
        dtype=np.float64,
    )
    rot_z = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )
    return rot_z @ rot_y @ rot_x


# 2d rotate
def rot2d(x, y, yaw):
    nx = x * np.cos(yaw) - y * np.sin(yaw)
    ny = x * np.sin(yaw) + y * np.cos(yaw)
    return nx, ny


# 3d rotate
def rot3d(pos, euler):
    R = euler_to_rot(euler[0], euler[1], euler[2])
    return R @ pos


def list_to_str(vec):
    return " ".join(str(s) for s in vec)


class TerrainGenerator:

    def __init__(
        self,
        folder_path: str,
        model_file: str,
        output_file: str = "scene_terrain.xml",
    ) -> None:
        """
        @param
        - folder_path: 文件夹路径
        - model_file: 模型文件名
        - output_file: 输出模型文件名
        """
        # 设置路径
        self.folder_path = folder_path
        self.model_path = self.folder_path + model_file
        self.output_path = self.folder_path + output_file
        # 设置场景
        self.scene = xml_et.parse(self.model_path)
        self.root = self.scene.getroot()
        self.worldbody = self.root.find("worldbody")
        self.asset = self.root.find("asset")
        # 保存标记
        self.is_saved = False

    def __del__(self):
        # 没有使用 Save() 进行保存的时候提示
        if not self.is_saved:
            raise RuntimeWarning("The terrain map was not saved yet, it may be lost.")

    def AddGeometry(
        self,
        position: list[3] = [1.0, 0.0, 0.0],
        euler: list[3] = [0.0, 0.0, 0.0],
        size: list = [0.1, 0.1],
        geo_type: str = "box",
    ):
        """
        添加物体

        @param
        - position: 位置
        - euler: 欧拉角
        - size: 尺寸
        - geo_type : "box", "plane", "sphere", "capsule", "ellipsoid", "cylinder"
        """
        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["pos"] = list_to_str(position)
        geo.attrib["type"] = geo_type
        geo.attrib["size"] = list_to_str(0.5 * np.array(size))  # half size of box for mujoco
        quat = euler_to_quat(euler[0], euler[1], euler[2])
        geo.attrib["quat"] = list_to_str(quat)

    def AddStairs(self, init_pos=[1.0, 0.0, 0.0], yaw=0.0, width=0.2, height=0.15, length=1.5, stair_nums=10):

        local_pos = [0.0, 0.0, -0.5 * height]
        for i in range(stair_nums):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.AddGeometry([x + init_pos[0], y + init_pos[1], local_pos[2]], [0.0, 0.0, yaw], [width, length, height])

    def AddSuspendStairs(self, init_pos=[1.0, 0.0, 0.0], yaw=1.0, width=0.2, height=0.15, length=1.5, gap=0.1, stair_nums=10):

        local_pos = [0.0, 0.0, -0.5 * height]
        for i in range(stair_nums):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.AddGeometry([x + init_pos[0], y + init_pos[1], local_pos[2]], [0.0, 0.0, yaw], [width, length, abs(height - gap)])

    def AddRoughGround(
        self,
        init_pos=[1.0, 0.0, 0.0],
        euler=[0.0, -0.0, 0.0],
        nums=[10, 10],
        box_size=[0.5, 0.5, 0.5],
        box_euler=[0.0, 0.0, 0.0],
        separation=[0.2, 0.2],
        box_size_rand=[0.05, 0.05, 0.05],
        box_euler_rand=[0.2, 0.2, 0.2],
        separation_rand=[0.05, 0.05],
    ):

        local_pos = [0.0, 0.0, -0.5 * box_size[2]]
        new_separation = np.array(separation) + np.array(separation_rand) * np.random.uniform(-1.0, 1.0, 2)
        for i in range(nums[0]):
            local_pos[0] += new_separation[0]
            local_pos[1] = 0.0
            for j in range(nums[1]):
                new_box_size = np.array(box_size) + np.array(box_size_rand) * np.random.uniform(-1.0, 1.0, 3)
                new_box_euler = np.array(box_euler) + np.array(box_euler_rand) * np.random.uniform(-1.0, 1.0, 3)
                new_separation = np.array(separation) + np.array(separation_rand) * np.random.uniform(-1.0, 1.0, 2)

                local_pos[1] += new_separation[1]
                pos = rot3d(local_pos, euler) + np.array(init_pos)
                self.AddBox(pos, new_box_euler, new_box_size)

    def AddPerlinHeighField(
        self,
        position: list = [1.0, 0.0, 0.0],  # position
        euler: list = [0.0, -0.0, 0.0],  # attitude
        size: list = [1.0, 1.0],  # width and length
        height_scale: float = 0.2,  # max height
        negative_height: float = 0.2,  # height in the negative direction of z axis
        image_width: int = 128,  # height field image size
        image_height: int = 128,
        smooth: float = 100.0,  # smooth scale
        perlin_octaves: int = 6,  # perlin noise parameter
        perlin_persistence: float = 0.5,
        perlin_lacunarity: float = 2.0,
        output_hfield_image: str = "height_field.png",
    ):
        """
        生成柏林噪声深度图

        @param
        - position: 高度场的位置，列表格式 [x, y, z]
        - euler: 高度场的姿态，欧拉角表示，列表格式 [roll, pitch, yaw]
        - size: 高度场的宽度和长度，列表格式 [width, length]
        - height_scale: 高度场的最大高度
        - negative_height: 高度场在z轴负方向的高度
        - image_width: 高度场图像的宽度
        - image_height: 高度场图像的高度
        - smooth: 平滑尺度，用于柏林噪声生成
        - perlin_octaves: 柏林噪声的倍频数
        - perlin_persistence: 柏林噪声的持久性
        - perlin_lacunarity: 柏林噪声的空隙性
        - output_hfield_image: 输出的高度场图像文件名
        """

        # Generating height field based on perlin noise
        terrain_image = np.zeros((image_height, image_width), dtype=np.uint8)
        for y in range(image_width):
            for x in range(image_width):
                # Perlin noise
                noise_value = noise.pnoise2(x / smooth, y / smooth, octaves=perlin_octaves, persistence=perlin_persistence, lacunarity=perlin_lacunarity)
                terrain_image[y, x] = int((noise_value + 1) / 2 * 255)

        cv2.imwrite(self.folder_path + output_hfield_image, terrain_image)

        hfield = xml_et.SubElement(self.asset, "hfield")
        hfield.attrib["name"] = "perlin_hfield"
        hfield.attrib["size"] = list_to_str([size[0] / 2.0, size[1] / 2.0, height_scale, negative_height])
        hfield.attrib["file"] = "../" + output_hfield_image

        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["type"] = "hfield"
        geo.attrib["hfield"] = "perlin_hfield"
        geo.attrib["pos"] = list_to_str(position)
        quat = euler_to_quat(euler[0], euler[1], euler[2])
        geo.attrib["quat"] = list_to_str(quat)

    def AddHeighFieldFromImage(
        self,
        position=[1.0, 0.0, 0.0],  # position
        euler=[0.0, -0.0, 0.0],  # attitude
        size=[2.0, 1.6],  # width and length
        height_scale=0.02,  # max height
        negative_height=0.1,  # height in the negative direction of z axis
        input_img=None,
        output_hfield_image="height_field.png",
        image_scale=[1.0, 1.0],  # reduce image resolution
        invert_gray=False,
    ):
        """
        从图像生成高度场并添加到仿真环境中

        @param
        - position: 高度场的位置，列表格式 [x, y, z]
        - euler: 高度场的姿态，欧拉角表示，列表格式 [roll, pitch, yaw]
        - size: 高度场的宽度和长度，列表格式 [width, length]
        - height_scale: 高度场的最大高度
        - negative_height: 高度场在z轴负方向的高度
        - input_img: 输入的高度场图像文件路径
        - output_hfield_image: 输出的高度场图像文件名
        - image_scale: 图像缩放比例，列表格式 [scale_x, scale_y]
        - invert_gray: 是否反转灰度图像
        """

        input_image = cv2.imread(input_img)  # 替换为你的图像文件路径

        width = int(input_image.shape[1] * image_scale[0])
        height = int(input_image.shape[0] * image_scale[1])
        resized_image = cv2.resize(input_image, (width, height), interpolation=cv2.INTER_AREA)
        terrain_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        if invert_gray:
            terrain_image = 255 - position
        cv2.imwrite(self.folder_path + output_hfield_image, terrain_image)

        hfield = xml_et.SubElement(self.asset, "hfield")
        hfield.attrib["name"] = "image_hfield"
        hfield.attrib["size"] = list_to_str([size[0] / 2.0, size[1] / 2.0, height_scale, negative_height])
        hfield.attrib["file"] = "../" + output_hfield_image

        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["type"] = "hfield"
        geo.attrib["hfield"] = "image_hfield"
        geo.attrib["pos"] = list_to_str(position)
        quat = euler_to_quat(euler[0], euler[1], euler[2])
        geo.attrib["quat"] = list_to_str(quat)

    def Save(self, output_file: str = None):
        """
        编辑完成后保存 xml 文件

        @param
        - output_file: 自定义输出文件名，路径为 self.folder_path

        @raise
        - ValueError: 当没有设置输出路径时
        """
        if self.output_path is None and output_file is None:
            raise ValueError("No vaild output path.")

        try:
            output_file = self.output_path if output_file is None else join(self.folder_path, output_file)
            self.scene.write(output_file)
            self.is_saved = True
        except Exception as e:
            raise RuntimeError(f'Fail to save the terrain map. Exception is "{e}".')


# ========== Addition ========== #
def get_random_position(size: float):
    """
    生成一个随机二维平面坐标（z轴为0）

    @param
    - size: 生成坐标的范围限制 (-size, size)

    @return
    - list: 生成的坐标 [x, y, z]
    """

    def ran(size: float, positive: bool):
        """
        生成一个随机数字，如果 positive 为 True 则为正数、

        @param
        - size: 同上
        - positive: False 为可能生成负数，True 为仅生成正数
        """
        return random.random() * size * ((-1 if random.random() <= 0.5 else 1) if not positive else 1)

    return [ran(size, 0), ran(size, 0), 0]


if __name__ == "__main__":

    folder_path = join(dirname(__file__) + "/../../../static/models/turingzero_agv/")
    model_file = "tz_agv.xml"  # 纯车，需要加地形
    model_file = "tz_agv_with_plane.xml"  # 有地形
    output_file = "scene_terrain.xml"

    POS_ORIGIN = [0.0, 0.0, -0.1]
    NUM_MAXLEN = 5
    NUM_MAXOBJS = NUM_MAXLEN ^ 2

    tg = TerrainGenerator(folder_path=folder_path, model_file=model_file)

    # tg.AddPerlinHeighField(position=POS_ORIGIN, size=[NUM_MAXLEN * 2, NUM_MAXLEN * 2], height_scale=0.5, negative_height=0.5, image_width=256, image_height=256, perlin_octaves=8, smooth=200)

    num = random.randint(5, NUM_MAXOBJS)
    for i in range(num):
        tg.AddGeometry(position=get_random_position(NUM_MAXLEN), size=[random.random() * 5, random.random() * 5], geo_type="cylinder")

    # the room
    tg.AddGeometry([0, 5, 4], size=[12, 0.02, 8])
    tg.AddGeometry([0, -5, 4], size=[12, 0.02, 8])
    tg.AddGeometry([5, 0, 4], size=[0.02, 12, 8])
    tg.AddGeometry([-5, 0, 4], size=[0.02, 12, 8])
    # tg.AddGeometry([0, 0, 8], size=[12, 12, 0.02])

    tg.Save()

"""
    # 用法参考

    # 添加物体
    tg.AddGeometry(position=[1.5, 0.0, 0.25], euler=[0, 0, 0.0], size=[1.0, 0.5, 0.5], geo_type="cylinder")

    # 楼梯
    tg.AddStairs(init_pos=[1.0, 4.0, 0.0], yaw=0.0)

    # 悬空楼梯
    # tg.AddSuspendStairs(init_pos=[1.0, 6.0, 0.0], yaw=0.0)

    # 粗糙地面
    tg.AddRoughGround(init_pos=[-2.5, 5.0, 0.0], euler=[0, 0, 0.0], nums=[10, 8])

    # 柏林噪声高程地面
    tg.AddPerlinHeighField(position=[0.0, 0.0, 0.0], size=[5.0, 5.0])

    # 由深度图生成的高程地面
    tg.AddHeighFieldFromImage(position=[-1.5, 2.0, 0.0], euler=[0, 0, -1.57], size=[2.0, 2.0], input_img="./unitree_robot.jpeg", image_scale=[1.0, 1.0], output_hfield_image="unitree_hfield.png")
"""
