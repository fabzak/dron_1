from setuptools import setup
from glob import glob

package_name = 'oko_osi_z'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf.xacro')),  # ✅ DODANO
        ('share/' + package_name + '/models/crazyflie', glob('models/crazyflie/*.sdf')),
        ('share/' + package_name + '/models/crazyflie/meshes/collada_files', glob('models/crazyflie/meshes/collada_files/*.dae')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabijan',
    maintainer_email='fabijan@todo.todo',
    description='Gazebo simulacija Crazyflie drona s ROS 2 podrškom',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer = oko_osi_z.explorer_node:main',
        ],
    },
)
