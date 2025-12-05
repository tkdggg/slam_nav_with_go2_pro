from setuptools import find_packages, setup
import os
from glob import glob # 确保导入 glob

package_name = 'message_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tkn-yangfukang',
    maintainer_email='yangfukang@tokennology.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "exe_node = message_test.exe:main",
            "pub_pos_base_link_node = message_test.pub_pos_base_link:main",
        ],
    },
)
