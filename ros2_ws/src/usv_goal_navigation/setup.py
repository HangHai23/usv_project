from glob import glob
from setuptools import setup

package_name = "usv_goal_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hai",
    maintainer_email="hai@example.com",
    description="Delay-compensated visual navigation to a selected field goal.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "goal_navigation_node = usv_goal_navigation.navigation_node:main",
            "navigation_imu_reference = usv_goal_navigation.imu_reference:main",
            "navigation_analyze = usv_goal_navigation.analysis:main",
            "navigation_identify = usv_goal_navigation.calibration:main",
            "navigation_simulate = usv_goal_navigation.simulation:main",
            "manual_demonstration = usv_goal_navigation.manual_demo:main",
            "manual_demo_analyze = usv_goal_navigation.demo_analysis:main",
        ]
    },
)
