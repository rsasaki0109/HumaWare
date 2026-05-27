from setuptools import setup

package_name = "humaware_skill_server"

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
    description="HumaWare skill server.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "skill_server_node = humaware_skill_server.skill_server_node:main",
        ],
    },
)
