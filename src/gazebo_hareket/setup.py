from setuptools import find_packages, setup

package_name = 'gazebo_hareket'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtlebot3_full.launch.py']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wololoo',
    maintainer_email='wololoo@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hareket_kontrol = gazebo_hareket.hareket_kontrol:main'
        ],
    },
)
