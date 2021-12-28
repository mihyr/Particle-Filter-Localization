from setuptools import setup

package_name = 'particle_filter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mihir',
    maintainer_email='mspatel1298@gmail.com',
    description='Particle filter implementation for turtlebot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'landmark_detector = particle_filter.landmark_detector:main',
        ],
    },
)
