from setuptools import setup

package_name = 'mahir_enes_akpinar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hareket_kontrol.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mahir Enes Akpınar',
    maintainer_email='mahir.enes@example.com',
    description='TurtleBot3 LIDAR tabanlı engelden kaçınma projesi',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hareket_kontrol = mahir_enes_akpinar.hareket_kontrol:main',
        ],
    },
)
