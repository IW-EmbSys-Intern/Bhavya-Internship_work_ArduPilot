from setuptools import find_packages, setup

package_name = 'my_plane_controller'

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
    maintainer='indowings24',
    maintainer_email='bhavyasingh2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'circle = my_plane_controller.circle:main',
            'takeoff_mavros = my_plane_controller.takeoff_using_mavros:main',
            'takeoff_mavros2 = my_plane_controller.takeoff_using_mavros2:main',
            'yolo_sample = my_plane_controller.yolo_sample:main',
            'square = my_plane_controller.square:main'
        ],
    },
)
