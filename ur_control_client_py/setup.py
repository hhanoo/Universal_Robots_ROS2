from setuptools import find_packages, setup

package_name = "ur_control_client_py"

setup(
    # =========================================================
    # Package metadata
    # =========================================================
    name=package_name,
    version="1.0.0",
    # =========================================================
    # Package discovery
    # Automatically find all Python packages in the directory
    # =========================================================
    packages=find_packages(exclude=["test"]),
    # =========================================================
    # Data files to install
    # These files are required for ROS2 package discovery
    # =========================================================
    data_files=[
        # Register package with ROS2 index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Install package.xml for dependency management
        ("share/" + package_name, ["package.xml"]),
    ],
    # =========================================================
    # Dependencies
    # =========================================================
    install_requires=["setuptools"],
    zip_safe=True,
    # =========================================================
    # Package information
    # =========================================================
    maintainer="hhanoo",
    maintainer_email="woo980711@gmail.com",
    description="Python client library and examples for UR robot control (MoveJ, MoveL, I/O, Speed)",
    license="BSD-3-Clause",
    # =========================================================
    # Extra dependencies (for testing, etc.)
    # =========================================================
    extras_require={},
    # =========================================================
    # Entry points (Executable scripts)
    # Maps command names to Python functions
    # Format: 'command_name = package.module:function'
    # =========================================================
    entry_points={
        "console_scripts": [
            # ros2 run ur_control_client_py example_movej
            "example_movej = ur_control_client_py.example_movej:main",
            # ros2 run ur_control_client_py example_io_speed
            "example_io_speed = ur_control_client_py.example_io_speed:main",
            # ros2 run ur_control_client_py example_complete
            "example_complete = ur_control_client_py.example_complete:main",
        ],
    },
)
