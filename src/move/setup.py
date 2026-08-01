from setuptools import find_packages, setup

package_name = 'move'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/visual_servoing.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arya-pangging',
    maintainer_email='arya-pangging@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [

            'WrapperNode = move.Wrapper:main',

            'Visual_Servoing = move.Visual_servoing:main',

            'test_node = move.test_node:main',
        ],
    },
)
