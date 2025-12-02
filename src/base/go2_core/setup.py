from setuptools import setup

package_name = 'go2_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your name',
    maintainer_email='you@email.com',
    description='Go2 core nodes',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # 格式: '可执行名 = 包名.模块名:函数名'
            'pub_pos_base_link = go2_core.pub_pos_base_link:main',
        ],
    },
)
