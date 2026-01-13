from setuptools import find_packages, setup

package_name = 'fr3_vr_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=['fr3_vr_teleop'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lin',
    maintainer_email='am2028082@163.com',
    description='FR3 VR Teleoperation node',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fr3_vr_teleop_node = fr3_vr_teleop.fr3_vr_teleop_node:main',
            'fr3_vr_test = fr3_vr_teleop.fr3_vr_test:main',
            'track_recorder = fr3_vr_teleop.track_recorder:main',
        ],
    },
)

