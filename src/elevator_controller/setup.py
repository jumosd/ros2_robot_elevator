from setuptools import find_packages, setup

package_name = 'elevator_controller'

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
    maintainer='jinsoo',
    maintainer_email='jinsoo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = elevator_controller.test_node:main',
            'elevator_call_service_node = elevator_controller.elevator_call_service_node:main'
        ],
    },
)
