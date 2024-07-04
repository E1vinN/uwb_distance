#!/usr/bin/env python3
from setuptools import setup

package_name = 'uwb_distance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'uwb_distance.uwb_receiver',
        'uwb_distance.uwb_transmitter',
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package for UWB distance measurement',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb_receiver = uwb_distance.uwb_receiver:main',
            'uwb_transmitter = uwb_distance.uwb_transmitter:main',
        ],
    },
)

