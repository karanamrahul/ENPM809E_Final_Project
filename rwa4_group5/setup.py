from setuptools import setup

package_name = 'rwa4_group5'

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
    maintainer='Rahul Karanam Pratik Acharya',
    maintainer_email='rkaranam@umd.edu pratik99@umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'rwa4 = rwa4_group5.rwa4_node:main',
        ],
    },
)
