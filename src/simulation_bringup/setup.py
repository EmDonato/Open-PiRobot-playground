from glob import glob
from setuptools import setup

setup(name='simulation_bringup', version='0.1.0',
      packages=['simulation_bringup'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/simulation_bringup']),
                  ('share/simulation_bringup', ['package.xml']),
                  ('share/simulation_bringup/launch', glob('launch/*.py')),
                  ('share/simulation_bringup/config', glob('config/*.yaml') + glob('config/*.rviz'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Emanuele Donato', maintainer_email='mcf.donato@gmail.com',
      description='Configurable house simulation and hardware adapters', license='Apache-2.0',
      entry_points={'console_scripts': ['hardware_adapter = simulation_bringup.hardware:main',
                                         'stamped_to_twist = simulation_bringup.teleop:main']})
