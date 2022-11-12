from setuptools import setup

package_name = 'gpg_urdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/display.launch.py']),
        ('share/' + package_name, ['urdf/robot.urdf']),
        ('share/' + package_name, ['launch/gpg_sim.launch.py']),
        ('share/' + package_name, ['config/controllers.yaml']),
        ('share/' + package_name, ['urdf/world.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lci',
    maintainer_email='lci@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'reach_goal = gpg_urdf.reach_goal:main',
        'go = gpg_urdf.go:main'
        ],
    },
)
