import os
from glob import glob
from setuptools import setup

package_name = 'y1_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # 如果你在包里有 Python 模块目录的话
    py_modules=[],
    data_files=[
        # 这是必须的，让 ROS 2 能找到此包
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 安装 package.xml 到 share/<包名>
        (os.path.join('share', package_name), ['package.xml']),
        # 安装 launch 脚本
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # 安装 config 文件
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # 如果你还想安装 urdf 文件
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xiaofan zhang',
    maintainer_email='601081321@qq.com',
    description='The y1_controller package',
    license='Apache-2.0',  # 比如 "MIT" 或 "Apache-2.0"
    entry_points={
        'console_scripts': [
            # 把你的 Python 脚本作为可执行节点注册
            # 名称 = 模块路径:main 函数
            'y1_controller = y1_controller.y1_controller:main',
        ],
    },
)