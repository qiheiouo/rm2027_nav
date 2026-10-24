from glob import glob
from setuptools import find_packages, setup

setup(
    name="rm_posture_motion", version="0.1.0", packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/rm_posture_motion"]),
        ("share/rm_posture_motion", ["package.xml"]),
        ("share/rm_posture_motion/config", glob("config/*.yaml")),
        ("share/rm_posture_motion/launch", glob("launch/*.launch.py")),
    ], install_requires=["setuptools"], tests_require=["pytest"], zip_safe=True,
    entry_points={"console_scripts": [
        "posture_profile_selector = rm_posture_motion.selector:main",
    ]},
)
