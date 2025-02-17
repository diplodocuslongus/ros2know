from setuptools import find_packages, setup

package_name = 'rviz_marker'

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
    maintainer='ludofw',
    maintainer_email='hk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'rvizmarker = rviz_marker.rviz_marker_tuto:main',
                    'rviztest = rviz_marker.rviz_test2:main'
        ],
    },
)
