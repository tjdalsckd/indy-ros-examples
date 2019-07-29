
// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// mesh�� �ҷ����� ���� �ʿ�
#include <geometric_shapes/shape_operations.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();

  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("indy7");
  moveit::planning_interface::MoveGroupInterface hand_group("robotiq");
  moveit_visual_tools::MoveItVisualTools visual_tools("/ground");
  visual_tools.deleteAllMarkers();
  // moveit_visual_tools::MoveItVisualTools visual_tools("/ground", rviz_visual_tools::RVIZ_MARKER_TOPIC);

  group.setPlanningTime(3000.0);

  // addCollisionObjects(planning_scene_interface);
  //moveit_visual_tools Ŭ������ collision object�߰��ϱ�
  //=========================================================

  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  // planning_scene_diff�� publish�� ���� ����
  //�Է°��� ���� topic queue size�̴�.
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }

  std::vector<moveit_msgs::CollisionObject> collision_objects;  //���̺�, ť�� 4�� ������ ������ ����
  collision_objects.resize(5);                                  //ũ�� 5�� ����

  //���̺� ===========================================
  collision_objects[0].id = "table";
  collision_objects[0].header.frame_id = "/ground";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.5;
  collision_objects[0].primitives[0].dimensions[1] = 0.7;
  collision_objects[0].primitives[0].dimensions[2] = 0.1;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.55;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.05;

  collision_objects[0].operation = collision_objects[0].ADD;

  //�Ʒ� ���� ť�� 1 ===============================
  collision_objects[1].id = "cube_under_1";
  collision_objects[1].header.frame_id = "/ground";

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.06;
  collision_objects[1].primitives[0].dimensions[1] = 0.06;
  collision_objects[1].primitives[0].dimensions[2] = 0.06;

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.55;
  collision_objects[1].primitive_poses[0].position.y = -0.10;
  collision_objects[1].primitive_poses[0].position.z = 0.13;

  collision_objects[1].operation = collision_objects[0].ADD;

  //���� ���� ť�� 1 ===============================
  collision_objects[2].id = "cube_over_1";
  collision_objects[2].header.frame_id = "/ground";

  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.06;
  collision_objects[2].primitives[0].dimensions[1] = 0.06;
  collision_objects[2].primitives[0].dimensions[2] = 0.06;

  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.45;
  collision_objects[2].primitive_poses[0].position.y = -0.10;
  collision_objects[2].primitive_poses[0].position.z = 0.13;

  collision_objects[2].operation = collision_objects[0].ADD;

  //�Ʒ� ���� ť�� 2 ===============================
  collision_objects[3].id = "cube_under_2";
  collision_objects[3].header.frame_id = "/ground";

  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.06;
  collision_objects[3].primitives[0].dimensions[1] = 0.06;
  collision_objects[3].primitives[0].dimensions[2] = 0.06;

  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.55;
  collision_objects[3].primitive_poses[0].position.y = 0.10;
  collision_objects[3].primitive_poses[0].position.z = 0.13;

  collision_objects[3].operation = collision_objects[0].ADD;

  //���� ���� ť�� 2 ===============================
  collision_objects[4].id = "cube_over_2";
  collision_objects[4].header.frame_id = "/ground";

  collision_objects[4].primitives.resize(1);
  collision_objects[4].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[4].primitives[0].dimensions.resize(3);
  collision_objects[4].primitives[0].dimensions[0] = 0.06;
  collision_objects[4].primitives[0].dimensions[1] = 0.06;
  collision_objects[4].primitives[0].dimensions[2] = 0.06;

  collision_objects[4].primitive_poses.resize(1);
  collision_objects[4].primitive_poses[0].position.x = 0.45;
  collision_objects[4].primitive_poses[0].position.y = 0.10;
  collision_objects[4].primitive_poses[0].position.z = 0.13;

  collision_objects[2].operation = collision_objects[0].ADD;

  // collision_object�� ������ planning_scene �޽��� ����=======================
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(collision_objects[0]);
  planning_scene.world.collision_objects.push_back(collision_objects[1]);
  planning_scene.world.collision_objects.push_back(collision_objects[2]);
  planning_scene.world.collision_objects.push_back(collision_objects[3]);
  planning_scene.world.collision_objects.push_back(collision_objects[4]);

  //���̺��� ť���� ���� ======================================================
  std_msgs::ColorRGBA red;
  red.r = red.a = 1.0;
  red.g = red.b = 0.0;

  std_msgs::ColorRGBA blue;
  blue.r = blue.g = 0.0;
  blue.b = blue.a = 1.0;

  std_msgs::ColorRGBA cyan;
  cyan.r = 0.0;
  cyan.g = cyan.b = cyan.a = 1.0;

  std_msgs::ColorRGBA orange;
  orange.r = orange.a = 1.0;
  orange.g = 0.5;
  orange.b = 0.0;

  std_msgs::ColorRGBA white;
  white.r = white.g = white.b = white.a = 1.0;

  planning_scene.object_colors.resize(5);
  planning_scene.object_colors[0].id = "table";
  planning_scene.object_colors[1].id = "cube_under_1";
  planning_scene.object_colors[2].id = "cube_over_1";
  planning_scene.object_colors[3].id = "cube_under_2";
  planning_scene.object_colors[4].id = "cube_over_2";

  planning_scene.object_colors[0].color = red;
  planning_scene.object_colors[1].color = cyan;
  planning_scene.object_colors[2].color = blue;
  planning_scene.object_colors[3].color = orange;
  planning_scene.object_colors[4].color = white;

  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

  //���������� Collision Object���� �߰��ϴ� �κ� ===============

  ros::WallDuration(1.0).sleep();

  // Pick and Place 1=========================================================
  //==============================================================
  //ť�긦 ���� �� �ִ� ��ġ�� �̵��մϴ�.
  geometry_msgs::Pose pick_preparation_1;
  pick_preparation_1.orientation.x = 0.707;
  pick_preparation_1.orientation.y = -0.707;
  pick_preparation_1.orientation.z = 0.0;
  pick_preparation_1.orientation.w = 0.0;
  // rpy = 0,-pi, -pi/2
  pick_preparation_1.position.x = 0.45;
  pick_preparation_1.position.y = -0.10;
  pick_preparation_1.position.z = 0.28;
  group.setPoseTarget(pick_preparation_1);
  group.move();

  std::vector<std::string> collision_link_when_grasp;
  collision_link_when_grasp.push_back("left_inner_knuckle");
  collision_link_when_grasp.push_back("right_inner_knuckle");
  collision_link_when_grasp.push_back("left_inner_finger");
  collision_link_when_grasp.push_back("right_inner_finger");
  group.attachObject("cube_over_1", "tcp", collision_link_when_grasp);
  //�׸��۰� ť�긦 ���� ��, ��ũ finger���� �浹�� ������ ���� �մϴ�.

  ros::WallDuration(1.0).sleep();


  geometry_msgs::Pose place_preparation_1;
  place_preparation_1.orientation.x = 0.707;
  place_preparation_1.orientation.y = -0.707;
  place_preparation_1.orientation.z = 0.0;
  place_preparation_1.orientation.w = 0.0;
  // rpy = 0,-pi, -pi/2
  place_preparation_1.position.x = 0.55;
  place_preparation_1.position.y = -0.10;
  place_preparation_1.position.z = 0.34;
  group.setPoseTarget(place_preparation_1);
  group.move();

  group.detachObject("cube_over_1");

  //ť�긦 �ű� ���·� planning scene�� ������ ��, publish�ؼ� ����
  planning_scene.object_colors[2].color = blue;
  planning_scene.world.collision_objects[2].primitive_poses[0].position.x = 0.55;
  planning_scene.world.collision_objects[2].primitive_poses[0].position.z = 0.19;
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  // Pick and Place 1=========================================================
  //==============================================================

    // Pick and Place 2=========================================================
  //==============================================================
  //ť�긦 ���� �� �ִ� ��ġ�� �̵��մϴ�.
  pick_preparation_1.orientation.x = 0.707;
  pick_preparation_1.orientation.y = -0.707;
  pick_preparation_1.orientation.z = 0.0;
  pick_preparation_1.orientation.w = 0.0;
  // rpy = 0,-pi, -pi/2
  pick_preparation_1.position.x = 0.45;
  pick_preparation_1.position.y = 0.10;
  pick_preparation_1.position.z = 0.28;
  group.setPoseTarget(pick_preparation_1);
  group.move();

  group.attachObject("cube_over_2", "tcp", collision_link_when_grasp);
  //�׸��۰� ť�긦 ���� ��, ��ũ finger���� �浹�� ������ ���� �մϴ�.

  ros::WallDuration(1.0).sleep();

  place_preparation_1.orientation.x = 0.707;
  place_preparation_1.orientation.y = -0.707;
  place_preparation_1.orientation.z = 0.0;
  place_preparation_1.orientation.w = 0.0;
  // rpy = 0,-pi, -pi/2
  place_preparation_1.position.x = 0.55;
  place_preparation_1.position.y = 0.10;
  place_preparation_1.position.z = 0.34;
  group.setPoseTarget(place_preparation_1);
  group.move();

  group.detachObject("cube_over_2");

  //ť�긦 �ű� ���·� planning scene�� ������ ��, publish�ؼ� ����
  planning_scene.object_colors[4].color = white;
  planning_scene.world.collision_objects[4].primitive_poses[0].position.x = 0.55;
  planning_scene.world.collision_objects[4].primitive_poses[0].position.z = 0.19;
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  // Pick and Place 1=========================================================
  //==============================================================

  // Wait a bit for ROS things to initialize



  // pick_1(group);

  //ros::WallDuration(1.0).sleep();
  // place_1(group);

  // place�� �����ϸ�, �տ��� ť�긦 �и�
  // group.detachObject("cube_over");

  //ros::WallDuration(1.0).sleep();
  //pick_2(group);

  //ros::WallDuration(1.0).sleep();
  //place_2(group);

  ros::waitForShutdown();
  return 0;
}
