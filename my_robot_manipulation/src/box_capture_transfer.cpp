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

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to add a platforms");

  //platform1
  moveit_msgs::CollisionObject collision_platform1;
  collision_platform1.header.frame_id = move_group.getPlanningFrame();
 
  collision_platform1.id = "platform1";

  shape_msgs::SolidPrimitive primitive_platform1;
  primitive_platform1.type = primitive_platform1.BOX;
  primitive_platform1.dimensions.resize(3);
  primitive_platform1.dimensions[0] = 0.21;
  primitive_platform1.dimensions[1] = 0.21;
  primitive_platform1.dimensions[2] = 0.13;

  geometry_msgs::Pose platform1_pose;
  platform1_pose.orientation.w = 1.0;
  platform1_pose.position.x = -0.4;
  platform1_pose.position.y = 0;
  platform1_pose.position.z = 0.0625;

  collision_platform1.primitives.push_back(primitive_platform1);
  collision_platform1.primitive_poses.push_back(platform1_pose);
  collision_platform1.operation = collision_platform1.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_object1;
  collision_object1.push_back(collision_platform1);

  ROS_INFO_NAMED("tutorial", "Add a platform1 into the world");
  planning_scene_interface.addCollisionObjects(collision_object1);

  //platform2
  moveit_msgs::CollisionObject collision_platform2;
  collision_platform2.header.frame_id = move_group.getPlanningFrame();
 
  collision_platform2.id = "platform2";

  shape_msgs::SolidPrimitive primitive_platform2;
  primitive_platform2.type = primitive_platform2.BOX;
  primitive_platform2.dimensions.resize(3);
  primitive_platform2.dimensions[0] = 0.21;
  primitive_platform2.dimensions[1] = 0.21;
  primitive_platform2.dimensions[2] = 0.13;

  geometry_msgs::Pose platform2_pose;
  platform2_pose.orientation.w = 1.0;
  platform2_pose.position.x = -0.105;
  platform2_pose.position.y = 0.295;
  platform2_pose.position.z = 0.0625;

  collision_platform2.primitives.push_back(primitive_platform2);
  collision_platform2.primitive_poses.push_back(platform2_pose);
  collision_platform2.operation = collision_platform2.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_object2;
  collision_object2.push_back(collision_platform2);

  ROS_INFO_NAMED("tutorial", "Add a platform2 into the world");
  planning_scene_interface.addCollisionObjects(collision_object2);

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

  std::vector<moveit_msgs::CollisionObject> collision_object3;
  collision_object3.push_back(collision_box);

  ROS_INFO_NAMED("tutorial", "Add a box into the world");
  planning_scene_interface.addCollisionObjects(collision_object3);

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

  ROS_INFO_NAMED("tutorial", "Remove the platform1 from the world");
 std::vector<std::string> object_ids_platform1;
 object_ids_platform1.push_back(collision_platform1.id);
 planning_scene_interface.removeCollisionObjects(object_ids_platform1);

  ROS_INFO_NAMED("tutorial", "Remove the platform2 from the world");
 std::vector<std::string> object_ids_platform2;
 object_ids_platform2.push_back(collision_platform2.id);
 planning_scene_interface.removeCollisionObjects(object_ids_platform2);

  visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  ros::shutdown();
  return 0;
}
