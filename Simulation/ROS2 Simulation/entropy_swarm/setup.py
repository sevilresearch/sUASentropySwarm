from setuptools import setup, find_packages

package_name = 'entropy_swarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raymond',
    maintainer_email='raymond@todo.todo',
    description='Entropy swarm node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # executable_name = package.module:function
            'entropy_swarm_node = entropy_swarm.entropy_swarm_node:main',
        ],
    },
)