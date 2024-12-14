#include <homodeus_arm_interface/ArmInterfaceNode.h>

ArmInterfaceNode::ArmInterfaceNode(ros::NodeHandle n):
nh{n}, 
gac("/gripper_controller/follow_joint_trajectory", true),
tac("/torso_controller/follow_joint_trajectory", true),
hac("/head_controller/follow_joint_trajectory", true)
{
    ROS_INFO("Node init strated");

    pick_pose_sub = nh.subscribe("/object_detector/prehension_pose", 1, &ArmInterfaceNode::pickPoseCB, this);
    drop_pose_sub = nh.subscribe("/object_detector/prehension_drop_pose", 1, &ArmInterfaceNode::dropPoseSafeCB, this);

    close_gripper_goal.trajectory = closedGripper();
    open_gripper_goal.trajectory = openedGripper();

    close_schunk_gripper_goal.trajectory = closedSchunkGripper();
    open_schunk_gripper_goal.trajectory = openedSchunkGripper();
    
    go_up.trajectory = goUp();
    look_down.trajectory = lookDown();
    
    ROS_INFO("Waiting for gripper joint controller server...");
    gac.waitForServer();
    ROS_INFO("Found  gripper joint controller server");
    
    ROS_INFO("Waiting for torso joint controller server...");
    tac.waitForServer();
    ROS_INFO("Found  torso joint controller server");
    
    ROS_INFO("Waiting for head joint controller server...");
    hac.waitForServer();
    ROS_INFO("Found  head joint controller server");
    
    // HBBA observer topics
    hbba_take_response_pub = nh.advertise<homodeus_msgs::HDResponse>("/Homodeus/Behaviour/Take/Response", 1);
    hbba_take_status_pub = nh.advertise<homodeus_msgs::HDStatus>("/Homodeus/Behaviour/Take/Status", 1);
    hbba_drop_response_pub = nh.advertise<homodeus_msgs::HDResponse>("/Homodeus/Behaviour/Drop/Response", 1);
    hbba_drop_status_pub = nh.advertise<homodeus_msgs::HDStatus>("/Homodeus/Behaviour/Drop/Status", 1);

    ROS_INFO("Cleaning obstacles");
    cleanObstacles();

    ROS_INFO("Node init done");
    
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::closedGripper()
{
    trajectory_msgs::JointTrajectory close_fingers;
    close_fingers.joint_names.resize(2);
    close_fingers.joint_names[0] = "gripper_left_finger_joint";
    close_fingers.joint_names[1] = "gripper_right_finger_joint";
    close_fingers.points.resize(1);
    close_fingers.points[0].positions.resize(2);
    close_fingers.points[0].positions[0] = 0.01;
    close_fingers.points[0].positions[1] = 0.01;
    close_fingers.points[0].time_from_start = ros::Duration(0.5);
    return close_fingers;
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::openedGripper()
{
    trajectory_msgs::JointTrajectory open_fingers;
    open_fingers.joint_names.resize(2);
    open_fingers.joint_names[0] = "gripper_left_finger_joint";
    open_fingers.joint_names[1] = "gripper_right_finger_joint";
    open_fingers.points.resize(1);
    open_fingers.points[0].positions.resize(2);
    open_fingers.points[0].positions[0] = 0.045;
    open_fingers.points[0].positions[1] = 0.045;
    open_fingers.points[0].time_from_start = ros::Duration(0.5);
    return open_fingers;
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::closedSchunkGripper()
{
    trajectory_msgs::JointTrajectory close_fingers;
    close_fingers.joint_names.resize(1);
    close_fingers.joint_names[0] = "gripper_finger_joint";
    close_fingers.points.resize(1);
    close_fingers.points[0].positions.resize(1);
    // close_fingers.points[0].positions[0] = 0.024;
    close_fingers.points[0].positions[0] = 0.015;
    close_fingers.points[0].effort.resize(1);
    close_fingers.points[0].effort[0] = 50.0; 
    close_fingers.points[0].time_from_start = ros::Duration(1.5);
    return close_fingers;
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::openedSchunkGripper()
{
    trajectory_msgs::JointTrajectory open_fingers;
    open_fingers.joint_names.resize(1);
    open_fingers.joint_names[0] = "gripper_finger_joint";
    open_fingers.points.resize(1);
    open_fingers.points[0].positions.resize(1);
    open_fingers.points[0].positions[0] = 0.030;
    open_fingers.points[0].time_from_start = ros::Duration(0.5);
    return open_fingers;
}
trajectory_msgs::JointTrajectory ArmInterfaceNode::goUp()
{
    trajectory_msgs::JointTrajectory go_up;
    go_up.joint_names.resize(1);
    go_up.joint_names[0] = "torso_lift_joint";
    go_up.points.resize(1);
    go_up.points[0].positions.resize(1);
    go_up.points[0].positions[0] = 0.36;
    go_up.points[0].time_from_start = ros::Duration(3);
    return go_up;
}

trajectory_msgs::JointTrajectory ArmInterfaceNode::lookDown()
{
    trajectory_msgs::JointTrajectory look_down;
    look_down.joint_names.resize(2);
    look_down.joint_names[0] = "head_1_joint";
    look_down.joint_names[1] = "head_2_joint";
    look_down.points.resize(1);
    look_down.points[0].positions.resize(2);
    look_down.points[0].positions[0] = 0;
    look_down.points[0].positions[1] = -0.5;
    look_down.points[0].time_from_start = ros::Duration(3);
    return look_down;
}

void ArmInterfaceNode::pickPoseCB(const homodeus_prehension::PrehensionPos& prehensionPos)
{
    const homodeus_msgs::HDPose& hd_pose_msg = prehensionPos.hdpose;

    std::vector<moveit_msgs::CollisionObject> obstacles_list = prehensionPos.obstacles;
    addObstacles(obstacles_list);
    
    geometry_msgs::Pose pose = hd_pose_msg.pose;
    bool success = false;

    ROS_INFO("Pick position received, going to grasp preparation pose");
    success = gotoGraspPrep();
    if (success)
    {
        ROS_INFO("Now at grasp preparation pose, opening gripper");
        gac.sendGoalAndWait(open_schunk_gripper_goal, ros::Duration(2));
    }
    else
    {
        ROS_INFO("Failed to go to grasp preparation pose in time, will still attempt rest of pick sequence");
    }

    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose.orientation, quat);
    double roll, pitch, yaw;

    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    auto x  = pose.position.x;
    auto y  = pose.position.y;
    auto z  = pose.position.z;

    pitch = pitch +1.571;

    // z += 0.05;
    // y -= 0.015;

    ROS_INFO("Preparation done, going in front of pose");
    success = moveToCartesian(x-0.3, y, z-0.01, roll, pitch, yaw);

    if (success) {
        ROS_INFO("Now in front, going at the pose");
        success = moveToCartesian(x-0.10, y, z-0.01, roll, pitch, yaw);
    }

    if (success)
    {   
        ROS_INFO("Pose reached, closing gripper");
        gac.sendGoalAndWait(close_schunk_gripper_goal, ros::Duration(2));
        
        ROS_INFO("Gripper closed, going up");
        moveToCartesian(x-0.10, y, z+0.20, roll, pitch, yaw);
 
    }

    if (success)
    {
        ROS_INFO("SUCCES to go to pick point!");
    } else {
        ROS_INFO("FAILED to go to pick point!");
    }

    ROS_INFO("Pickup done, going home");
    goHome();

    ROS_INFO("Home reached, going to carry pose");
    gotoCarryPose();

    if (success) {
        ROS_INFO("SUCCES - sending response");
        homodeus_msgs::HDResponse hd_response_msg;
        hd_response_msg.id =  hd_pose_msg.id;
        hd_response_msg.result = success; 
        hbba_take_response_pub.publish(hd_response_msg);

    }else {
        ROS_INFO("FAILED - sending status");
        homodeus_msgs::HDStatus hd_status_msg;
        hd_status_msg.id =  hd_pose_msg.id;
        hbba_take_status_pub.publish(hd_status_msg);

    }

    cleanObstacles();
}

// For safety, we use this callback that only rotates the arm and open the gripper to drop the object
// This is to avoid any unwanted collisions with humans
// The arm as to be in CarryPose for it to work as intended
void ArmInterfaceNode::dropPoseSafeCB(const homodeus_prehension::PrehensionPos& prehensionPos){

    bool success = true;
    const homodeus_msgs::HDPose& hd_pose_msg = prehensionPos.hdpose;

    ROS_INFO("Drop position received, rotation arm");
    success = moveToJoint(0.35, 0.49, -1.31, -0.78, 2.17, 0.5, 1.39, 1.20);
    
    ROS_INFO("Arm rotated, oppening gripper");
    gac.sendGoalAndWait(open_schunk_gripper_goal, ros::Duration(1));

    ROS_INFO("Gripper open, going back to carry pose");
    success = gotoCarryPose();

    ROS_INFO("SUCCES - sending response");
    homodeus_msgs::HDResponse hd_response_msg;
    hd_response_msg.id =  hd_pose_msg.id;
    hd_response_msg.result = success; 
    hbba_drop_response_pub.publish(hd_response_msg);
}

void ArmInterfaceNode::dropPoseCB(const homodeus_prehension::PrehensionPos& prehensionPos)
{   
    const homodeus_msgs::HDPose& hd_pose_msg = prehensionPos.hdpose;

    std::vector<moveit_msgs::CollisionObject> obstacles_list = prehensionPos.obstacles;
    addObstacles(obstacles_list);
    
    geometry_msgs::Pose pose = hd_pose_msg.pose;
    bool success = false;

    ROS_INFO("Drop position received, going to grasp preparation pose");
    success = gotoGraspPrep();
    if (success)
    {
        ROS_INFO("Now at grasp preparation pose");
    }
    else
    {
        ROS_INFO("Failed to go to grasp preparation pose in time, will still attempt rest of drop sequence");
    }

    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose.orientation, quat);
    double roll, pitch, yaw;

    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    auto x  = pose.position.x;
    auto y  = pose.position.y;
    auto z  = pose.position.z;

    pitch = pitch +1.571;

    // z += 0.05;
    // y -= 0.015;

    ROS_INFO("Preparation done, going in front of pose");
    success = moveToCartesian(x-0.3, y, z-0.01, roll, pitch, yaw);

    if (success) {
        ROS_INFO("Now in front, going at the pose");
        success = moveToCartesian(x-0.10, y, z-0.01, roll, pitch, yaw);
    }

    if (success)
    {   
        ROS_INFO("Pose reached, opening gripper");
        gac.sendGoalAndWait(open_schunk_gripper_goal, ros::Duration(2));
        
        ROS_INFO("Gripper open, going up");
        moveToCartesian(x-0.10, y, z+0.20, roll, pitch, yaw);
 
    }

    if (success)
    {
        ROS_INFO("SUCCES to go to drop point!");
    } else {
        ROS_INFO("FAILED to go to drop point!");
    }

    ROS_INFO("Drop done, going home");
    goHome();

    ROS_INFO("Home reached, going to carry pose");
    gotoCarryPose();

    if (success) {
        ROS_INFO("SUCCES - sending response");
        homodeus_msgs::HDResponse hd_response_msg;
        hd_response_msg.id =  hd_pose_msg.id;
        hd_response_msg.result = success; 
        hbba_drop_response_pub.publish(hd_response_msg);

    }else {
        ROS_INFO("FAILED - sending status");
        homodeus_msgs::HDStatus hd_status_msg;
        hd_status_msg.id =  hd_pose_msg.id;
        hbba_drop_status_pub.publish(hd_status_msg);

    }

    cleanObstacles();    
}

bool ArmInterfaceNode::goHome()
{
    bool success = false;
    success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 1.37, 0.0);
    return success;
}

bool ArmInterfaceNode::gotoGraspPrep()
{
    bool success;
    success = moveToJoint(0.34, 0.20, 0.79, 0.01, 2.10, -1.5, 1.37, 0.0);
    success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 1.37, 0.0);
    return success;
}

bool ArmInterfaceNode::gotoCarryPose()
{
    bool success;
    success = moveToJoint(0.35, 0.49, -1.31, -0.78, 2.17, -1.25, 1.39, 1.20);
    return success;
}

void ArmInterfaceNode::gotoInitPose()
{
    ROS_INFO("Going up to look position");
    tac.sendGoalAndWait(go_up, ros::Duration(3));
    ROS_INFO("Looking down");
    hac.sendGoalAndWait(look_down, ros::Duration(3));
    ROS_INFO("Ready");
}

// Code to use the arm interface
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_interface_node");
    ros::NodeHandle n("~"); 
    
    ArmInterfaceNode arm_node(n); 

    ros::AsyncSpinner spinner(1);
    spinner.start();

    double frequency = 5;
    ros::Rate rate(frequency);
    while ( ros::ok() )
    {
        ros::spinOnce();
        rate.sleep();
    }

    ros::waitForShutdown();
    return 0;
}
