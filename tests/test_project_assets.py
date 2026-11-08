import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


class TestProjectAssets(unittest.TestCase):
    def test_ros_package_manifests_are_well_formed(self):
        for path in (
            ROOT / "src/safe_obstacle_avoidance/package.xml",
            ROOT / "src/safe_obstacle_avoidance_msgs/package.xml",
        ):
            with self.subTest(path=path):
                ET.parse(path)

    def test_message_package_uses_rosidl(self):
        manifest = ET.parse(ROOT / "src/safe_obstacle_avoidance_msgs/package.xml").getroot()
        dependencies = {node.tag: node.text for node in manifest}
        self.assertEqual(manifest.findtext("buildtool_depend"), "ament_cmake")
        self.assertIn("rosidl_default_generators", dependencies.values())
        self.assertEqual(manifest.findtext("export/build_type"), "ament_cmake")

    def test_gazebo_world_has_expected_systems_and_model(self):
        root = ET.parse(ROOT / "simulation/worlds/ellipsoid_obstacle_world.sdf").getroot()
        world = root.find("world")
        self.assertIsNotNone(world)
        plugin_names = {node.attrib.get("name") for node in world.findall("plugin")}
        self.assertIn("gz::sim::systems::Physics", plugin_names)
        self.assertIn("gz::sim::systems::Sensors", plugin_names)
        model_names = {node.attrib.get("name") for node in world.findall("model")}
        self.assertTrue({"safe_bot", "obstacle_1", "obstacle_2", "obstacle_3"}.issubset(model_names))
        robot = world.find("model[@name='safe_bot']")
        self.assertIsNotNone(robot)
        self.assertIsNotNone(robot.find("link/sensor[@type='depth_camera']"))
        self.assertIsNotNone(robot.find("plugin[@name='gz::sim::systems::DiffDrive']"))
        for joint_name in ("left_wheel_joint", "right_wheel_joint", "rear_caster_joint"):
            joint = robot.find(f"joint[@name='{joint_name}']")
            self.assertEqual(joint.find("pose").attrib.get("relative_to"), "base_link")

    def test_bridge_yaml_maps_required_topics(self):
        path = ROOT / "src/safe_obstacle_avoidance/config/ros_gz_bridge.yaml"
        bridges = yaml.safe_load(path.read_text(encoding="utf-8"))
        mapping = {item["ros_topic_name"]: item for item in bridges}
        for topic in ("/clock", "/camera/depth/image_raw", "/odom", "/cmd_vel"):
            self.assertIn(topic, mapping)
        self.assertEqual(mapping["/camera/depth/image_raw"]["direction"], "GZ_TO_ROS")
        self.assertEqual(mapping["/cmd_vel"]["direction"], "ROS_TO_GZ")

    def test_ros_source_has_no_ros1_runtime_apis(self):
        forbidden = ("import rospy", "from rospy", "catkin_make", "roslaunch", "rosrun")
        source_root = ROOT / "src"
        for path in source_root.rglob("*.py"):
            text = path.read_text(encoding="utf-8")
            for token in forbidden:
                with self.subTest(path=path, token=token):
                    self.assertNotIn(token, text)

    def test_install_script_does_not_install_gazebo_simulator(self):
        script = (ROOT / "scripts/install_ros_deps.sh").read_text(encoding="utf-8")
        apt_install_block = script.split("sudo apt install -y", 1)[1].split("if [[", 1)[0]
        self.assertNotIn("ros-lyrical-ros-gz ", apt_install_block)
        self.assertIn("ros-lyrical-ros-gz-bridge", apt_install_block)
        self.assertNotIn("ros_gz_sim", apt_install_block)



if __name__ == "__main__":
    unittest.main()
