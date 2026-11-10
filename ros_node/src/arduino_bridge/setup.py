from setuptools import find_packages, setup

package_name = 'arduino_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    scripts=[
        'scripts/arduino_bridge.py',
        'scripts/flask_gui.py',
        'scripts/fake_arduino.py',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name + '/templates', ['scripts/templates/index.html']),
    ],
    install_requires=['setuptools', 'pyserial', 'Flask'],
    zip_safe=True,
    maintainer='aswarth',
    maintainer_email='m.aswarthreddy@gmail.com',
    description='Arduino serial transport and basic Flask motor control GUI',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
