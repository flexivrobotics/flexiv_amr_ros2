from pathlib import Path
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "flexiv_amr_description"

ROBOTS = {
    "fmr_300": {
        "urdf": Path("fmr_300") / "urdf" / "FMR300_and_Rizon10.urdf",
        "mesh_package": "FMR300",
        "arms": {"rizon10"},
    },
}

ARM_ELEMENTS = {
    "rizon10": {
        "joints": {
            "rizon10",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
            "link7_to_flange",
        },
        "links": {
            "base_link",
            "link1",
            "link2",
            "link3",
            "link4",
            "link5",
            "link6",
            "link7",
            "flange",
        },
    },
}


def _parse_scalar(value):
    value = value.strip().strip("\"'")
    lower_value = value.lower()
    if lower_value in ("true", "yes", "on", "1"):
        return True
    if lower_value in ("false", "no", "off", "0"):
        return False
    return value


def _load_config(config_file):
    if not config_file:
        return {}

    config_path = Path(config_file)
    if not config_path.is_file():
        raise RuntimeError(
            f"Robot description config file '{config_file}' does not exist"
        )

    config = {}
    for raw_line in config_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        config[key.strip()] = _parse_scalar(value)
    return config


def _remove_named_elements(root, tag, names):
    for element in list(root.findall(tag)):
        if element.get("name") in names:
            root.remove(element)


def _build_robot_description(config_file):
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    config = _load_config(config_file)

    mobile_base = config.get("mobile_base", "fmr_300")
    arm = config.get("arm", "rizon10")
    show_arm = bool(config.get("show_arm", True))

    if mobile_base not in ROBOTS:
        raise RuntimeError(f"Unsupported mobile_base '{mobile_base}'")
    if show_arm and arm not in ROBOTS[mobile_base]["arms"]:
        raise RuntimeError(f"Unsupported arm '{arm}' for mobile_base '{mobile_base}'")

    robot_info = ROBOTS[mobile_base]
    urdf_path = package_share / robot_info["urdf"]
    xml_text = urdf_path.read_text(encoding="utf-8")
    xml_text = xml_text.replace(
        f"package://{robot_info['mesh_package']}/",
        f"package://{PACKAGE_NAME}/{mobile_base}/",
    )

    if show_arm:
        return xml_text

    root = ElementTree.fromstring(xml_text)
    arm_info = ARM_ELEMENTS.get(arm)
    if arm_info is not None:
        _remove_named_elements(root, "joint", arm_info["joints"])
        _remove_named_elements(root, "link", arm_info["links"])
    return ElementTree.tostring(root, encoding="unicode")


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    rviz_config_file = LaunchConfiguration("rviz_config_file").perform(context)
    config = _load_config(config_file)
    use_gui = bool(config.get("use_gui", False))

    robot_description = _build_robot_description(config_file)

    joint_state_node = Node(
        package="joint_state_publisher_gui" if use_gui else "joint_state_publisher",
        executable="joint_state_publisher_gui" if use_gui else "joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    robot_state_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    return [
        joint_state_node,
        robot_state_node,
        rviz_node,
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    default_rviz_config_file = package_share / "rviz" / "flexiv_amr.default.rviz"

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="",
        description=(
            "Path to the robot description selection config file. "
            "If empty, built-in defaults are used."
        ),
    )
    rviz_config_file_arg = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=str(default_rviz_config_file),
        description="Path to the RViz config file.",
    )

    return LaunchDescription(
        [
            config_file_arg,
            rviz_config_file_arg,
            OpaqueFunction(function=_launch_setup),
        ]
    )
