from setuptools import find_packages, setup

package_name = 'willi'

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
    maintainer='Dennis Koslowski',
    maintainer_email='dennis.koslowski@gmx.de',
    description='Simple 2-wheeled robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'control_node = willi.my_node:main'
            'control_node = willi.control_node:main',
            'sensor_node = willi.sensor_node:main',
        ],
    },
)
