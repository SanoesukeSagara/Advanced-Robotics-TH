from setuptools import find_packages, setup

package_name = 'drive_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='mohamedislem2015@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processor = drive_controller.image_processor:main',
            'arduino_bridge = drive_controller.arduino_bridge:main',
            'lane_detector_node = drive_controller.lane_detector_node:main',
            'rectangle_tracker_node = drive_controller.rectangle_tracker_node:main',
        ],
    },
)
