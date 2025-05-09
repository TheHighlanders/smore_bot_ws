from setuptools import setup
import os
from glob import glob

package_name = 'smore_bot_web'

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
    maintainer='smore_bot',
    maintainer_email='user@example.com',
    description='Web dashboard for SMORE Bot using rosbridge',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = smore_bot_web.web_server:main',
        ],
    },
    package_data={
        'smore_bot_web': ['web/*'],
    },
)