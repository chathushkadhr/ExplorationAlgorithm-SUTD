from setuptools import setup

package_name = 'tf_publish'

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
    maintainer='chathushka-sutd',
    maintainer_email='chathushka.ranasinghe41@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'exec_tf_pub = tf_publish.publish_tf_for_two_robots:main',
        ],
    },
)


