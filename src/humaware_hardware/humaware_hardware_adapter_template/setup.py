from setuptools import setup

package_name = "humaware_hardware_adapter_template"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (
            f"share/{package_name}/docs",
            [
                "docs/adapter_checklist.md",
                "docs/verified_matrix_template.md",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HumaWare Maintainers",
    maintainer_email="maintainers@humaware.dev",
    description=(
        "Template for HumaWare hardware adapters. Copy, rename, and implement "
        "the vendor command translation while preserving the runtime safety "
        "contract."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hardware_adapter_template_node"
            " = humaware_hardware_adapter_template.template_adapter_node:main",
        ],
    },
)
