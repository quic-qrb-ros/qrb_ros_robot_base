# Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

from setuptools import setup, find_packages

package_name = 'qrb_ros_robot_base_keyboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author = [
        {"name": "Peng Wang", "email": "quic_penwang@quicinc.com"},
        {"name": "Busy Lao", "email": "quic_laoyi@quicinc.com"},
    ],
    description='QRB ROS robot base keyboard test tools',
    license='BSD-3-Clause-Clear',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_base = src.robot_base_keyboard:main',
        ],
    },
)
