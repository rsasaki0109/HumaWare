from setuptools import setup

package_name = "humaware_policy_provider_stub"

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
    description=(
        "Minimal HumaWare policy provider stub. Publishes candidate velocity"
        " commands to policy/cmd_vel for arbitration."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "policy_provider_stub_node"
            " = humaware_policy_provider_stub.policy_provider_stub_node:main",
        ],
    },
)
