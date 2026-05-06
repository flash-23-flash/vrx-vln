from setuptools import setup

package_name = 'formation_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Triangle formation straight-line path following demo for VRX',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'triangle_formation_controller = formation_planner.triangle_formation_controller:main',
        ],
    },
)