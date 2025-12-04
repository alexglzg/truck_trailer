from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'truck_trailer_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Index the package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm')),


        # Install launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),

        # Install URDF files
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf')),
        
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        (os.path.join('share', package_name, 'cfg'), glob('cfg/*.yaml')),  # <--- add this
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alex_gg97@hotmail.com',
    description='Truck + trailer simulation with URDF model',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_node = truck_trailer_sim.simulation_node:main',
            'mppi_planner_node = truck_trailer_sim.mppi_planner_node:main',
        ],
    },
)



