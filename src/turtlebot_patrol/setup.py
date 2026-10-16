from setuptools import find_packages, setup
from glob import glob

setup(name='turtlebot_patrol', version='1.0.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/turtlebot_patrol']),
                  ('share/turtlebot_patrol',['package.xml','README.md']),
                  ('share/turtlebot_patrol/config',glob('config/*.yaml')),
                  ('share/turtlebot_patrol/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools','PyYAML'], zip_safe=True,
      maintainer='Package maintainer', maintainer_email='maintainer@example.com',
      description='TurtleBot patrol and persistent emergency stop',license='MIT',
      entry_points={'console_scripts':[
          'patrol = turtlebot_patrol.node:main',
          'simulator = turtlebot_patrol.simulator:main',
          'dashboard = turtlebot_patrol.web_dashboard:main']})
