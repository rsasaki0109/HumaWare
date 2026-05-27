from setuptools import setup

package_name = "humaware_policy_provider_scripted"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", ["config/example_plan.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HumaWare Maintainers",
    maintainer_email="maintainers@humaware.dev",
    description=(
        "HumaWare scripted policy provider. Reads a waypoint plan from YAML"
        " and publishes candidate velocity commands onto policy/cmd_vel."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "policy_provider_scripted_node"
            " = humaware_policy_provider_scripted.policy_provider_scripted_node:main",
        ],
    },
)
