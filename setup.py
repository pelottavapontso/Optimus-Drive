from setuptools import find_packages, setup

package_name = 'my_saab'

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
    maintainer='cg8203827',
    maintainer_email='cg8203827@todo.todo',
    description='saab control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'steering_node = my_saab.steering_node:main',
            'talker=my_saab.publisher_member_function:main',
            'listener=my_saab.subscriber_member_function:main',
            'steering=my_saab.steering_node:main',
            'pedal=my_saab.pedal_node:main',
        ],
    },
)
