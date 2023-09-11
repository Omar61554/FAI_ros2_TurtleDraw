from setuptools import find_packages, setup
from glob import glob

package_name = 'custom_draw'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # Include all config files
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Include all worlds
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        # Include all rviz files
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        # Include all meshes
        ('share/' + package_name + '/meshes', glob('meshes/*.dae')),
        # Include all urdf files
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        # Include all launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omark',
    maintainer_email='wagih.omar11@gmai.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "shape_node = custom_draw.shapeNode:main",
            "turtle_commander= custom_draw.turtle_commander:main"
        ],
    },
)
