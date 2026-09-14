import glob

from setuptools import find_packages, setup

package_name = "quest_rebot_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kojumaru",
    maintainer_email="kojumaru52@gmail.com",
    description=(
        "Bridges Meta Quest 3 controller input to ROS 2 and reBot Arm "
        "B601-DM Cartesian teleoperation commands."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller_raw_publisher = "
            "quest_rebot_bridge.controller_raw_publisher:main",
            "bridge_node = quest_rebot_bridge.bridge_node:main",
            "fake_ik_driver = quest_rebot_bridge.fake_ik_driver:main",
        ],
    },
)
