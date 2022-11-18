from setuptools import setup

package_name = 'arducam_stereo_hat'

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
    maintainer='Kris Kunovski',
    maintainer_email='kkunovsk@purdue.edu',
    description='Captures camera snapshots and publishes to the appropriate topics for images and camera info.',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'talker = arducam_stereo_hat.arducam_stereo_camera:main'
        ],
    },
)
