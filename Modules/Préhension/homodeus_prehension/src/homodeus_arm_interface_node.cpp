#include <homodeus_arm_interface/ArmInterfaceNode.h>

ArmInterfaceNode::ArmInterfaceNode(ros::NodeHandle n):
nh{n}, 
gac("/gripper_controller/follow_joint_trajectory", true),
tac("/torso_controller/follow_joint_trajectory", true),
hac("/head_controller/follow_joint_trajectory", true)
{
    ROS_INFO("Node init strated");

    pick_pose_sub = nh.subscribe("/object_detector/hd_pose", 1, &ArmInterfaceNode::pickPoseCB, this);
    drop_pose_sub = nh.subscribe("/drop_point", 1, &ArmInterfaceNode::dropPoseCB, this);

    close_gripper_goal.trajectory = closedGripper();
    open_gripper_goal.trajectory = openedGripper();

    close_schunk_gripper_goal.trajectory = closedSchunkGripper();
    open_schunk_gripper_goal.trajectory = openedSchunkGripper();
    
    go_up.trajectory = goUp();
    look_down.trajectory = lookDown();
    
    ROS_INFO("Waiting for gripper joint controller server...");
    std::cout << "Waiting for gripper joint controller server..." << std::endl;
    //print("Waiting for gripper joint controller server...")
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
    hbba_drop_response_pub = nh.advertise<homodeus_msgs::HDResponse>("/Homodeus/Behaviour/Drop/Response", 1);

    // TEST ONLY: Temp drop pose
    drop_pose_pub = nh.advertise<homodeus_msgs::HDPose>("/drop_point", 1);

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
    close_fingers.points[0].positions[0] = 0.01;
    close_fingers.points[0].time_from_start = ros::Duration(0.5);
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
    go_up.points[0].positions[0] = 0.3;
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

void ArmInterfaceNode::gotoInitPose()
{

    ROS_INFO("Going up to look position");
    tac.sendGoalAndWait(go_up, ros::Duration(3));
    ROS_INFO("Looking down");
    hac.sendGoalAndWait(look_down, ros::Duration(3));
    ROS_INFO("Ready");

}

void ArmInterfaceNode::pickPoseCB(const homodeus_msgs::HDPose& hd_pose_msg)
{
    geometry_msgs::Pose pose = hd_pose_msg.pose;
    bool success = false;
    ROS_INFO("Going to grasp preparation pose");
    success = gotoGraspPrep();
    if (success)
    {
        ROS_INFO("Now at grasp preparation pose, opening gripper");
        //gac.sendGoalAndWait(open_gripper_goal, ros::Duration(2));
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

    roll = roll +1.571;
    ROS_INFO("arm_interface_node: will attempt to move the arm in cartesian space.");
    //success = moveToCartesian(x-0.4, y, z-0.05, roll, pitch, yaw);
    success = moveToCartesian(x-0.4, y, z, roll, pitch, yaw);
    ros::Duration(1).sleep();
    if (success)
    {
        ROS_INFO("arm_interface_node: reached first wayp1oint");
        //success = moveToCartesian(x-0.15, y, z-0.05, roll, pitch, yaw);
        success = moveToCartesian(x-0.15, y, z, roll, pitch, yaw);
    }

    if (success)
    {
        ROS_INFO("arm_interface_node: successfully moved to pick point, closing gripper...");
        //gac.sendGoalAndWait(close_gripper_goal, ros::Duration(2));
        gac.sendGoalAndWait(close_schunk_gripper_goal, ros::Duration(2));
        ROS_INFO("Closed!");
        //
        ROS_INFO("Going up!");
        moveToCartesian(x-0.15, y, z+0.5, roll, pitch, yaw);

        // TODO :
        ROS_INFO("Retreating");
 
    }
    else
        ROS_INFO("arm_interface_node: failed to go to pick point!");
    success = false;
    
    if (success)
    {
        ROS_INFO("arm_interface_node: successfully retreated from pick point.");
        ROS_INFO("Going to carrying pose");
        success  = goHome();

        ROS_INFO("arm_interface_node: SKIP GO HOME");
    }
    else
        ROS_INFO("arm_interface_node: failed to retreat from pick point!");

    if (success) {
        ROS_INFO("Good job, the object has been picked up.");
    }
    homodeus_msgs::HDResponse hd_response_msg;
    hd_response_msg.id =  hd_pose_msg.id;
    hd_response_msg.result = success; 
    hbba_take_response_pub.publish(hd_response_msg);
    

    // ros::Duration(1).sleep();
    // ROS_INFO("Now drop object.");
    // drop_pose_pub.publish(hd_pose_msg);

}

void ArmInterfaceNode::dropPoseCB(const homodeus_msgs::HDPose& hd_pose_msg)
{
    geometry_msgs::Pose pose = hd_pose_msg.pose;
    bool success = true;

    ROS_INFO("Going to drop preparation pose");
    success = gotoGraspPrep();
    // success = moveToJoint(0.35, 0.15, 0.00, -1.08, 2.29, 0.33, 0.27, -2.07);
    if (success)
    {
        ROS_INFO("Now at drop preparation pose");
    }
    else
    {
        ROS_ERROR("Failed to go to drop preparation pose in time, will still attempt rest of pick sequence");
    }

    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    auto x  = pose.position.x;
    auto y  = pose.position.y;
    auto z  = pose.position.z;
    roll = roll +1.571;
    ROS_INFO("arm_interface_node: will attempt to move the arm in cartesian space.");
    success = moveToCartesian(x-0.4, y, z+0.05, roll, pitch, yaw);
    if (success)
    {
        ROS_INFO("arm_interface_node: reached first waypoint");
        success = moveToCartesian(x, y, z+0.05, roll, pitch, yaw);
    }

    if (success)
    {
        ROS_INFO("arm_interface_node: successfully moved to drop point, opening gripper...");
        gac.sendGoalAndWait(open_gripper_goal, ros::Duration(2));
        ROS_INFO("Opened!");
    }
    else
        ROS_INFO("arm_interface_node: failed to go to drop point!");

    if(success)
    {
        success = gotoRetreat(pose);
    }
    
    if (success)
    {
        ROS_INFO("arm_interface_node: successfully retreated from drop point.");
        ROS_INFO("Going home");
        success = goHome();
    }
    else
        ROS_INFO("arm_interface_node: failed to retreat from drop point!");

    homodeus_msgs::HDResponse hd_response_msg;
    hd_response_msg.id = hd_pose_msg.id;
    hd_response_msg.result = success; 
    hbba_drop_response_pub.publish(hd_response_msg);
    
}

//TODO
bool ArmInterfaceNode::goHome()
{
    bool success = false;
    success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 0.14, 0.0);
    success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 1.37, 0.0);
    // success = moveToJoint(0.34, 0.20, 0.79, 0.01, 2.10, -1.5, 1.37, 0.0);
    // success = moveToJoint(0.25, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0);
    return success;
}

bool ArmInterfaceNode::gotoCarryPose()
{
    bool success;
    // success = moveToJoint(0.30, 0.10, 0.00, -1.72, 2.21, 0.00, 0.05, 0.00);
    success = moveToJoint(0.35, 0.15, 0.00, -1.08, 2.29, 0.33, 0.27, -2.07);

    ros::Duration(1).sleep();

    success = moveToJoint(0.20, 0.20, 0.0, 0.0,  2.18, -1.17, 1.01, -1.78);
    return success;
}

bool ArmInterfaceNode::gotoGraspPrep()
{
    bool success;
    success = moveToJoint(0.34, 0.20, 0.79, 0.01, 2.10, -1.5, 1.37, 0.0);
    success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 1.37, 0.0);
    success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 0.14, 0.0);
    return success;
}

bool ArmInterfaceNode::gotoRetreat(const geometry_msgs::Pose pose)
{
    ROS_INFO("Attempting retreat");

    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    auto x  = pose.position.x;
    auto y  = pose.position.y;
    auto z  = pose.position.z;
    return moveToCartesian(x-0.1, y, z+0.3, roll, pitch, yaw);

}

void ArmInterfaceNode::changeVelFactor(){
    std::string changeVel; 
    std::cout << "Current max_vel_factor : " << max_vel_factor << std::endl; 
    std::cout << "Change factor ? (y/n)" << std::endl;  
    std::cin >> changeVel; // Get user input from the keyboard
    float newVelFactor;
    
    if (changeVel == "n") {
        std::cout << "Skipping " << std::endl; 
        return;
    } else if (changeVel == "y") {
        std::cout << "Enter value between 0 and 1 : " << std::endl; 
        std::cin >> newVelFactor;
        if (newVelFactor > 0 && newVelFactor <= 1) {
            max_vel_factor = newVelFactor;
            return;
        } else{
            std::cout << "Wrong value" << std::endl; 
        }
    } else {
        std::cout << "Skipping " << std::endl; 
        return;
    }

    std::cout << "Trying Again " << std::endl; 
    std::cout << "" << std::endl; 
    // Failed Try again
    changeVelFactor();
}

// Code to use the arm interface
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_interface_node");
    ros::NodeHandle n("~"); 
    
    ArmInterfaceNode arm_node(n);
    // arm_node.changeVelFactor();
    arm_node.gotoInitPose();
    

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
