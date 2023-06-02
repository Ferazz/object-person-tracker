from setuptools import setup
from glob import glob

package_name = 'ros_paintball_person_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sebastian',
    maintainer_email='sg.olsson@hotmail.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = ros_paintball_person_tracker.camera:main',
            'yolo = ros_paintball_person_tracker.yolo_detector:main',
            'visualizer = ros_paintball_person_tracker.visualizer:main',
            'serial = ros_paintball_person_tracker.serial_talker:main',
        ],
    },
)
