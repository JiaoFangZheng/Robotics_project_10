from setuptools import find_packages, setup

package_name = 'linear_hri_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kang',
    maintainer_email='kang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'virtual_finger = linear_hri_sim.virtual_finger:main',
            'intersection_calc = linear_hri_sim.intersection_calc:main',
            'gesture_trigger = linear_hri_sim.gesture_trigger:main',
            'auto_commander = linear_hri_sim.auto_commander:main',
            'udp_bridge = linear_hri_sim.udp_bridge:main',
            'hand_monitor = linear_hri_sim.hand_monitor:main',
            'exp_non_filtered = linear_hri_sim.exp_non_filtered:main',
            'exp_ekf_fir = linear_hri_sim.exp_ekf_fir_filtered:main',
            'exp_non_filtered_v2 = linear_hri_sim.exp_non_filtered_v2:main',
            'exp_ekf_fir_v2 = linear_hri_sim.exp_ekf_fir_v2:main',
        ],
    },
) 
