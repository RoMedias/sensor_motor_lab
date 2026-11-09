from glob import glob
from setuptools import setup

pkg = "sensor_motor_bridge"
setup(
    name=pkg,
    version="0.1.0",
    packages=[pkg],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{pkg}"]),
        (f"share/{pkg}", ["package.xml"]),
        (f"share/{pkg}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    entry_points={"console_scripts": [
        f"bridge = {pkg}.bridge_node:main",
        f"fake_arduino = {pkg}.fake_arduino:main",
    ]},
)
