from setuptools import find_packages, setup

package_name = 'nav_astar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/astar.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jandui',
    maintainer_email='janduigamer13@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'astar_node = nav_astar.astar_node:main',
        ],
    },
)
