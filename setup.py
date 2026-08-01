from setuptools import find_packages, setup

package_name = 'tp_package'

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
    maintainer='Rakesh Suthar',
    maintainer_email='rakeshsuthar6322@gmail.com',
    description='Trajectory Planner for model car in model city environment',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tp_planner = tp_package.tp_planner:main'
        ],
    },
)
