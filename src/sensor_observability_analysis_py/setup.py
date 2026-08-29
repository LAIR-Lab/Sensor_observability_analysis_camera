from setuptools import setup
from glob import glob
import os

package_name = 'sensor_observability_analysis_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
        os.path.join('share', 'sensor_observability_analysis_py', 'config'),
        glob('config/*.yaml'),
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    entry_points={
        'console_scripts': [

            'sensor_geometry_node = sensor_observability_analysis_py.sensor_geometry_node:main',

            'soa_solver_node = sensor_observability_analysis_py.soa_solver:main',

            'soa_visualizer_node = sensor_observability_analysis_py.soa_visualizer:main',

            'soa_cam_visualizer_node = sensor_observability_analysis_py.soa_cam_visualizer:main',

            "soa_cam_tf_node = sensor_observability_analysis_py.soa_cam_tf_node:main",

            'soa_camera_jacobian_node = sensor_observability_analysis_py.soa_jacobian_cam_node:main',

            'soa_camera_obs_jacobian_node = sensor_observability_analysis_py.soa_jacobian_cam_obs_node:main',

            'soa_camera_general_jacobian_node = sensor_observability_analysis_py.soa_jacobian_cam_general:main',

            'log_test_node = sensor_observability_analysis_py.log_test:main',

            'soa_cam_move_node = sensor_observability_analysis_py.soa_cam_move:main',

            'soa_cam_move_node_2 = sensor_observability_analysis_py.soa_cam_move_2:main',

            'soa_cam_obs_move_node = sensor_observability_analysis_py.soa_cam_obs_move:main',

            'soa_cam_moving_node = sensor_observability_analysis_py.soa_cam_moving:main',

            

        ],
    },
)