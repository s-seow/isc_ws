from setuptools import setup, find_packages

package_name = 'isc_task_optimizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/isc_task_optimizer']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel',
    maintainer_email='ssmh1@outlook.com',
    description='ROS2 node that runs iSC optimizer',
    license='MIT',
    entry_points={
        'console_scripts': [
            # ros2 run isc_task_optimizer optimizer_node
            'optimizer_node = isc_optimizer.optimizer.optimizer_node:main',
        ],
    },
)