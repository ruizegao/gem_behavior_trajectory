from setuptools import setup

package_name = "gem_behavior_trajectory"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ruize",
    maintainer_email="ruizeg2@illinois.edu",
    description="Behavior to trajectory planner for GEM e4.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "behavior_to_trajectory_node = gem_behavior_trajectory.behavior_to_trajectory_node:main",
        ],
    },
)