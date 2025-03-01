from setuptools import find_packages, setup

package_name = 'interval_analysis_examples'

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
    maintainer='Noceo200',
    maintainer_email='ocean.noel@ensta-breatgen.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = interval_analysis_examples.simple_publisher_node:main',
            'simple_subscriber = interval_analysis_examples.simple_subscriber_node:main',
            'localisation_range_bearing_deriv = interval_analysis_examples.localisation_range_bearing_deriv_node:main',
        ],
    },
)
