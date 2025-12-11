from setuptools import find_packages, setup

package_name = 'ndvi_cam'

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
    maintainer='Victor Lucas Guillén',
    maintainer_email='victorlucasguillen@gmail.com',
    description='This package has the nodes in charge of NDVI sensor to Camera processing.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proceso_continuo=ndvi_cam.proceso_continuo:main',
            'proces_discontinuous=ndvi_cam.proces_discontinuous:main',
        ],
    },
)
