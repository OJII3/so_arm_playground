import os
import xml.etree.ElementTree as ET
from pathlib import Path


def test_console_script_wrappers_import_expected_entry_points():
    binary_dir = os.environ["TELEOP_IK_BINARY_DIR"].split(os.pathsep)[-1]

    expected_imports = {
        "ik_node": "from teleop_ik.ik_node import main",
        "gamepad_node": "from teleop_ik.gamepad_node import main",
    }

    for script_name, import_line in expected_imports.items():
        script_path = Path(binary_dir) / script_name
        assert script_path.exists()
        assert import_line in script_path.read_text()


def test_manifest_does_not_require_undeclared_cmake_python_dependency():
    cmake_lists = Path(__file__).parents[1] / "CMakeLists.txt"
    if "find_package(ament_cmake_python REQUIRED)" not in cmake_lists.read_text():
        return

    package_xml = Path(__file__).parents[1] / "package.xml"
    root = ET.parse(package_xml).getroot()

    declared_dependencies = {
        element.text
        for tag in ("buildtool_depend", "build_depend", "depend")
        for element in root.findall(tag)
    }

    assert "ament_cmake_python" in declared_dependencies
