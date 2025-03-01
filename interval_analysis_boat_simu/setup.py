from setuptools import find_packages, setup

package_name = 'interval_analysis_boat_simu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'interval_analysis_boat_simu.simu',
        'interval_analysis_boat_simu.boat',
        'interval_analysis_boat_simu.buoy',
        'interval_analysis_boat_simu.calcul_tools',
        'interval_analysis_boat_simu.draw',
        'interval_analysis_boat_simu.potential_fields',
        'interval_analysis_boat_simu.sea_object',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reximini',
    maintainer_email='onoel2050@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boat_simu = interval_analysis_boat_simu.interval_analysis_boat_simu_node:main'
        ],
    },
)
