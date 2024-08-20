from setuptools import find_packages, setup

package_name = 'ofa_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['mappings/class_mapping.txt']),
        ('share/' + package_name, ['mappings/species_id_to_name.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jan-Marco Haldemann',
    maintainer_email='j.haldemann@proton.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cluster_and_classify = ofa_detection.cluster_and_classify:main'
        ],
    },
)
