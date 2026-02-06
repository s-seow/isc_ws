from setuptools import setup, find_packages

package_name = 'isc_subscriber'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/isc_subscriber']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel',
    maintainer_email='ssmh1@outlook.com',
    description='ROS2 subscriber for optimizer_summary JSON output.',
    license='MIT',
    entry_points={
        'console_scripts': [
            # ros2 run isc_subscriber summary_subscriber
            'summary_subscriber = isc_subscriber.summary_subscriber_node:main',
        ],
    },
)
