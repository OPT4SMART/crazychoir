from setuptools import setup
from glob import glob


package_name = 'crazychoir_examples'
scripts = {
    'bearingformationcontrol': ['integrator', 'controller', 'guidance'],
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
    ],
    install_requires=['setuptools','crazychoir','choirbot_examples'],
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
