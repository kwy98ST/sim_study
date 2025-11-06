from setuptools import find_packages, setup

package_name = 'dobby_nav2'

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
    maintainer='addinedu',
    maintainer_email='kwy98ST@github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav2_test = dobby_nav2.nav2_test:main',
            'dobby_nav2_server = dobby_nav2.dobby_nav2_server:main',
            'dobby_nav2_client = dobby_nav2.dobby_nav2_client:main',
            'dobby_way_server = dobby_nav2.dobby_way_server:main',
            'dobby_way_client = dobby_nav2.dobby_way_client:main',


        ],
    },
)
