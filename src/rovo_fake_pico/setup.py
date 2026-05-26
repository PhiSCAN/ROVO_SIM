from setuptools import find_packages, setup

package_name = 'rovo_fake_pico'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fake_pico.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phicode',
    maintainer_email='tombabyrose@gmail.com',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fake_pico_node = rovo_fake_pico.fake_pico_node:main',
        ],
    },
)
