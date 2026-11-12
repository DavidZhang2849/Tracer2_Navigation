from glob import glob

from setuptools import find_packages, setup

setup(
    name='tracer_navigation',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tracer_navigation']),
        ('share/tracer_navigation', ['package.xml', 'README.md', 'LICENSE']),
        ('share/tracer_navigation/launch', glob('launch/*.launch.py')),
        ('share/tracer_navigation/config', glob('config/*')),
        ('share/tracer_navigation/rviz', glob('rviz/*.rviz')),
        ('share/tracer_navigation/scripts', ['scripts/setup.bash']),
        ('share/tracer_navigation/docs', glob('docs/*.md')),
        ('share/tracer_navigation/docs/validation', glob('docs/validation/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steno',
    maintainer_email='steno@example.com',
    description='Hardware-specific Tracer mapping, localization and navigation bringup',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'doctor = tracer_navigation.doctor:main',
        'obstacle_height_filter = tracer_navigation.obstacle_filter:main',
    ]},
)
