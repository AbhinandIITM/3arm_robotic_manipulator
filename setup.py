from setuptools import find_packages, setup
import glob
import os

package_name = 'robotic_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob.glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhinand',
    maintainer_email='me23b208@smail.iitm.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = robotic_arm.controller:main', 
            'gui_controller = robotic_arm.gui_control:main', 
        ],
    },
)
