from setuptools import setup

package_name = 'test_dynamixel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='pzgq5uh7@s.okayama-u.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = test_dynamixel.test_node:main',
            'display_node = test_dynamixel.test_twist_display_node:main',
            'control_node = test_dynamixel.test_twist_node:main',
            'control_demo_node = test_dynamixel.test_twist_demo_node:main'
        ],
    },
)
