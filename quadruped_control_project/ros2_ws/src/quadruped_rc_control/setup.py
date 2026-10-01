from glob import glob
from setuptools import setup
setup(name='quadruped_rc_control', version='0.1.0', packages=['quadruped_rc_control'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/quadruped_rc_control']),
                ('share/quadruped_rc_control', ['package.xml']),
                ('share/quadruped_rc_control/config', glob('config/*.yaml')),
                ('share/quadruped_rc_control/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True, maintainer='hai',
    maintainer_email='hai@example.com', description='Four single-axis legs under CRSF control',
    license='MIT', entry_points={'console_scripts': ['controller = quadruped_rc_control.controller:main']})
