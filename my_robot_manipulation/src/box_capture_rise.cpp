#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "robot_arm";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.5;
  visual_tools.publishText(text_pose, "MoveGroupInterface", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();

  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to add a platform");

  //platform
  moveit_msgs::CollisionObject collision_platform;
  collision_platform.header.frame_id = move_group.getPlanningFrame();
 
  collision_platform.id = "platform";

  shape_msgs::SolidPrimitive primitive_platform;
  primitive_platform.type = primitive_platform.BOX;
  primitive_platform.dimensions.resize(3);
  primitive_platform.dimensions[0] = 0.21;
  primitive_platform.dimensions[1] = 0.21;
  primitive_platform.dimensions[2] = 0.13;

  geometry_msgs::Pose platform_pose;
  platform_pose.orientation.w = 1.0;
  platform_pose.position.x = -0.4;
  platform_pose.position.y = 0;
  platform_pose.position.z = 0.0625;

  collision_platform.primitives.push_back(primitive_platform);
  collision_platform.primitive_poses.push_back(platform_pose);
  collision_platform.operation = collision_platform.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_object1;
  collision_object1.push_back(collision_platform);

  ROS_INFO_NAMED("tutorial", "Add a platform into the world");
  planning_scene_interface.addCollisionObjects(collision_object1);

  //box coral
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to add a box");

  moveit_msgs::CollisionObject collision_box;
  collision_box.header.frame_id = move_group.getPlanningFrame();
 
  collision_box.id = "box";

  shape_msgs::SolidPrimitive primitive_box;
  primitive_box.type = primitive_box.BOX;
  primitive_box.dimensions.resize(3);
  primitive_box.dimensions[0] = 0.065;
  primitive_box.dimensions[1] = 0.008;
  primitive_box.dimensions[2] = 0.03;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -0.4;
  box_pose.position.y = 0;
  box_pose.position.z = 0.16;

  collision_box.primitives.push_back(primitive_box);
  collision_box.primitive_poses.push_back(box_pose);
  collision_box.operation = collision_box.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_object2;
  collision_object2.push_back(collision_box);

  ROS_INFO_NAMED("tutorial", "Add a box into the world");
  planning_scene_interface.addCollisionObjects(collision_object2);

  visual_tools.publishText(text_pose, "Add box", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to attach the box from the robot");

  ROS_INFO_NAMED("tutorial", "Attach the box to the robot");
  move_group.attachObject(collision_box.id);

  visual_tools.publishText(text_pose, "Box attached to robot", rvt::WHITE, rvt::XLARGE);
 visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to detach the box from the robot");
 
  ROS_INFO_NAMED("tutorial", "Detach the box from the robot");
 move_group.detachObject(collision_box.id);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to remove the objects from the world");

  ROS_INFO_NAMED("tutorial", "Remove the box from the world");
 std::vector<std::string> object_ids_box;
 object_ids_box.push_back(collision_box.id);
 planning_scene_interface.removeCollisionObjects(object_ids_box);

  ROS_INFO_NAMED("tutorial", "Remove the platform from the world");
 std::vector<std::string> object_ids_platform;
 object_ids_platform.push_back(collision_platform.id);
 planning_scene_interface.removeCollisionObjects(object_ids_platform);

  visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  ros::shutdown();
  return 0;
}
