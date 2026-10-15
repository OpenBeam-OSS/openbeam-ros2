from glob import glob

from setuptools import find_packages, setup

package_name = "openbeam_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenBeam, LLC",
    maintainer_email="support@openbeam.world",
    description="ROS 2 bridge for the OpenBeam simulated SO-101 arm",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = openbeam_bridge.bridge_node:main",
            "measure = openbeam_bridge.measure:main",
            "latency_probe = openbeam_bridge.latency_probe:main",
            "login = openbeam_bridge.login:main",
        ],
    },
)
