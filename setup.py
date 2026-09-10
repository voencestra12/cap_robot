"""역할: Python/ROS 리소스 설치. 인터페이스: colcon build, console_scripts."""

from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

# [변경] SDK는 safety_node만 소유하며 각 실행 모듈을 독립 프로세스로 설치.
modules = [
    "safety_node",
    "agent_node",
    "workstation_node",
    "cooperative_node",
    "perception_node",
    "calibration_node",
    "mount_tf",
    "metrics",
    "metrics_report",
    "cli",
]
setup(
    name="cap_robot",
    version="1.0.1",  # [변경] 기본 demo launch 인자 충돌 수정 배포.
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/cap_robot"]),
        ("share/cap_robot", ["package.xml", "README.md"]),
    ]
    + [
        (f"share/cap_robot/{d}", [p for p in glob(f"{d}/*") if Path(p).is_file()])
        for d in ["config", "prompts", "launch", "models", "urdf", "docs"]
    ],
    install_requires=["setuptools", "PyYAML", "requests", "numpy", "scipy"],
    # [변경] colcon이 unittest 대신 pytest 시험을 실제 수집하도록 명시한다.
    tests_require=["pytest"],
    zip_safe=False,
    maintainer="Capstone team",
    maintainer_email="maintainer@example.com",
    description="Independent LLM dual xArm agents with mandatory safety gateway",
    license="Apache-2.0",
    entry_points={"console_scripts": [f"{m} = cap_robot.{m}:main" for m in modules]},
)
