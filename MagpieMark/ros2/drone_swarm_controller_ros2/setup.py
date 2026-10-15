from setuptools import find_packages, setup

package_name = "drone_swarm_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            f"share/{package_name}",
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "drone_test_swarms = drone_swarm_controller.drone_test_swarms:main",
        ],
    },
)
