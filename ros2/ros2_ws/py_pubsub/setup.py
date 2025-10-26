from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='aprig',
    maintainer_email='aprig@unileon.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publisher_member_function = py_pubsub.publisher_member_function:main',
            'subscriber_member_function = py_pubsub.subscriber_member_function:main',
            'multi_thread_executor = py_pubsub.multi_thread_executor:main',
        ],
    },
)
