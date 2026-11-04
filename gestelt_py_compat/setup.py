from setuptools import find_packages, setup


package_name = "gestelt_py_compat"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gestelt maintainers",
    maintainer_email="maintainers@gestelt.local",
    description="rclpy compatibility helpers for Gestelt",
    license="BSD",
)
