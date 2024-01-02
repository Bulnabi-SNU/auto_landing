from setuptools import setup

package_name = 'goal_pub'

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
    maintainer='bulnabi',
    maintainer_email='bulnabi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_pub = goal_pub.tag_pub:main',
            'filter = goal_pub.filter:main',
            'depth = goal_pub.depth:main'
        ],
    },
)
