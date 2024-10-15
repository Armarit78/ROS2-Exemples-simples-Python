from setuptools import find_packages, setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # Assurez-vous que ce champ indique le bon package
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_sql = my_package.pub_sql:main',
            'pub = my_package.pub:main',
            'sub_sql = my_package.sub_sql:main',
            'verif = my_package.verif:main',
            'pointcloud_translator = my_package.pointcloud_translator:main',
            'pointcloud_comparator = my_package.pointcloud_comparator:main',
        ],
    },
)

