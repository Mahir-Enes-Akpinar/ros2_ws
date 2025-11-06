from setuptools import find_packages, setup

package_name = 'mahir_enes_akpinar_odev'

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
    maintainer='AD SOYAD',
    maintainer_email='seninmail@ornek.com',
    description='Turtlesim kare hareket ödevi',
    license='MIT',
    entry_points={
        'console_scripts': [
            'turtlesim_square = mahir_enes_akpinar_odev.turtlesim_square:main',
        ],
    },
)
