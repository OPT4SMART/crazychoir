from setuptools import setup
from glob import glob


package_name = 'crazychoir_examples'
scripts = {
    'bearingformationcontrol': ['controller','guidance','integrator','rviz'],
    'formation_webots': ['controller_leaders','controller_followers','gui','guidance','trajectory'],
    }
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('resource/*.rviz')),
        ('share/' + package_name, glob('resource/*.urdf')),
        ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
        ('share/' + package_name + '/worlds/meshes', glob('worlds/meshes/*.stl')),
    ],
    install_requires=['setuptools','crazychoir'],
    zip_safe=True,
    maintainer='opt4smart',
    maintainer_email='opt4smart@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazychoir_{0}_{1} = crazychoir_examples.{0}.{1}:main'.format(package, file)
            for package, files in scripts.items() for file in files
        ],
    },
)