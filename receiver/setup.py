from glob import glob
from setuptools import setup

package_name = "receiver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[
        "mqtt_receiver_node",
        "mqtt_receiver_node.proto",
        "video_decoder_node",
    ],
    package_data={"mqtt_receiver_node.proto": ["*.proto"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="ROS 2 MQTT receiver and H264 video decoder for RoboMaster",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mqtt_receiver_node = mqtt_receiver_node.mqtt_receiver_node:main",
            "video_decoder_node = video_decoder_node.video_decoder_node:main",
        ],
    },
)
