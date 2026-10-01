from glob import glob
from setuptools import setup
setup(name='usv_position_estimator', version='0.1.0', packages=['usv_position_estimator'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/usv_position_estimator']),
                  ('share/usv_position_estimator', ['package.xml']),
                  ('share/usv_position_estimator/config', glob('config/*.yaml')),
                  ('share/usv_position_estimator/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools', 'numpy'], maintainer='hai',
      maintainer_email='hai@example.com', description='Planar delayed inertial/UDP position filter',
      license='MIT', entry_points={'console_scripts': ['position_estimator = usv_position_estimator.node:main']})
