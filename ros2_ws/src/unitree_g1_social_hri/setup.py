from glob import glob
from setuptools import setup

package_name = "unitree_g1_social_hri"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dr Sajjad Hussain",
    maintainer_email="ro603-lecturer@example.com",
    description="Speech- and LLM-based social HRI with a simulated Unitree G1",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "speech_input_node = unitree_g1_social_hri.speech_input_node:main",
            "llm_interaction_node = unitree_g1_social_hri.llm_interaction_node:main",
            "action_validator_node = unitree_g1_social_hri.action_validator_node:main",
            "g1_simulation_node = unitree_g1_social_hri.g1_simulation_node:main",
            "speech_output_node = unitree_g1_social_hri.speech_output_node:main",
            "hri_monitor_node = unitree_g1_social_hri.hri_monitor_node:main",
        ],
    },
)
