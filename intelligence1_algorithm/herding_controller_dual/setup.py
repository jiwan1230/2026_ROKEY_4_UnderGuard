from setuptools import find_packages, setup

package_name = "herding_controller_dual"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/herding_params.yaml"]),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="sunwook",
    maintainer_email="ekdldkrksek6974@gmail.com",
    description="Two-robot cooperative target herding controller",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "herding_node = herding_controller_dual.herding_node:main",
        ],
    },
)
