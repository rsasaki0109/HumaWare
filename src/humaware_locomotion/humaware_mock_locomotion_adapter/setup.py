from setuptools import setup

package_name = "humaware_mock_locomotion_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HumaWare Maintainers",
    maintainer_email="maintainers@humaware.dev",
    description="Mock HumaWare locomotion adapter.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mock_locomotion_adapter_node = "
            "humaware_mock_locomotion_adapter.mock_locomotion_adapter_node:main",
        ],
    },
)
