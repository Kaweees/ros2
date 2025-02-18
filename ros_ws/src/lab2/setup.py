from setuptools import find_packages, setup
from glob import glob

package_name = "lab2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Miguel Villa Floran",
    maintainer_email="miguel.villa.floran@gmail.com",
    description="TODO: Package description",
    license="GPLv3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "draw_square = src.draw_square:main",
            "draw_spiral = src.draw_spiral:main",
            "drive_to_point = src.drive_to_point:main",
        ],
    },
)
