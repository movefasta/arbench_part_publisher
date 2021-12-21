import os
from glob import glob
from setuptools import setup

package_name = 'arbench_part_publisher'

setup(
    name=package_name,
    version='0.2.0',
    # Packages to export
    packages=[package_name],
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mathias Hauan Arbo',
    author_email='mathias.arbo@ntnu.no',
    maintainer='Igor Brylyov',
    maintainer_email='movefasta@dezcom.org',
    keywords=['arbench', 'part', 'publisher', 'freecad'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS2 node for publishing feature frames annotated using ARBench',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arbench_part_publisher_node = arbench_part_publisher.arbench_part_publisher_node:main',
            'meshloader = arbench_part_publisher.meshloader:main',
            'mesh_object_node = arbench_part_publisher.mesh_object_node:main' 
        ],
    },
)