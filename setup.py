from setuptools import find_packages, setup

package_name = 'franka_gripper_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anunth Ramaswami',
    maintainer_email='anunthramaswami2030@u.northwestern.edu',
    description='Franka Gripper Control Package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'faulhaber_trajectory_node = franka_gripper_control.faulhaber_joint_trajectory_node:main'
        ],
    },
)
