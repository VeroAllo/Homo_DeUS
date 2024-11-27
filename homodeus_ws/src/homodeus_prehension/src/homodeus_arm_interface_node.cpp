#include <homodeus_arm_interface/ArmInterfaceNode.h>

ArmInterfaceNode::ArmInterfaceNode(ros::NodeHandle n):
nh{n}, 
gac("/gripper_controller/follow_joint_trajectory", true),
tac("/torso_controller/follow_joint_trajectory", true),
hac("/head_controller/follow_joint_trajectory", true)
{
    ROS_INFO("Node init strated");

    pick_pose_sub = nh.subscribe("/object_detector/prehension_pose", 1, &ArmInterfaceNode::pickPoseCB, this);
    drop_pose_sub = nh.subscribe("/drop_point", 1, &ArmInterfaceNode::dropPoseCB, this);
    drop_hard_sub = nh.subscribe("/object_detector/prehension_drop_pose", 1, &ArmInterfaceNode::dropPoseHard, this);

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
    hbba_take_status_pub = nh.advertise<homodeus_msgs::HDStatus>("/Homodeus/Behaviour/Take/Status", 1);
    hbba_drop_response_pub = nh.advertise<homodeus_msgs::HDResponse>("/Homodeus/Behaviour/Drop/Response", 1);
    
    // ros::Publisher gripper_pub = nh.advertise<pal_control_msgs::ActuatorCurrentLimit>("/gripper_current_limit_controller/command", 1);
    // pal_control_msgs::ActuatorCurrentLimit actuator;
    // actuator.actuator_names.resize(1);
    // actuator.actuator_names[0] = "gripper_motor";
    // actuator.current_limits.resize(1);
    // actuator.current_limits[0] = 0.1;

    // gripper_pub.publish(actuator);

    // ros::Duration(2).sleep();
    // gripper_pub.shutdown();


    // TEST ONLY: Temp drop pose
    drop_pose_pub = nh.advertise<homodeus_msgs::HDPose>("/drop_point", 1);

    ROS_INFO("Cleaning");
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

void ArmInterfaceNode::gotoInitPose()
{

    ROS_INFO("Going up to look position");
    tac.sendGoalAndWait(go_up, ros::Duration(3));
    ROS_INFO("Looking down");
    hac.sendGoalAndWait(look_down, ros::Duration(3));
    ROS_INFO("Ready");

}

bool ArmInterfaceNode::makeAllPlans(geometry_msgs::Pose pose){
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose.orientation, quat);
    double roll, pitch, yaw;

    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    auto x  = pose.position.x;
    auto y  = pose.position.y;
    auto z  = pose.position.z;

    pitch = pitch +1.571;

    gac.sendGoalAndWait(open_schunk_gripper_goal, ros::Duration(2));

    bool success = false;

    const moveit::planning_interface::MoveGroupInterface::Plan* lastPlan = nullptr;
    moveit::planning_interface::MoveGroupInterface::Plan nextPlan;
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;

    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    plans.clear();

    // setInitState();
    // Infront of Objet
    std::vector<std::string> plannerIds = 
    {"SBLkConfigDefault",
    "RRTConnectkConfigDefault",
    "LBKPIECEkConfigDefault",
    "PRMkConfigDefault",
    "RRTkConfigDefault",
    "ESTkConfigDefault",
    "STOMP",
    "CHOMP"
    };

    for (auto planner : plannerIds){
        ROS_INFO_STREAM("Plan FRONT with : " << planner);

        nextPlan = nextCartesianPlan(plan1, planner, x-0.35, y, z-0.01, roll, pitch, yaw);
        if(_plan_success){break;}
    }

    if (!_plan_success) { ROS_INFO_STREAM("Plan FAILED - FRONT"); return false; }
    plans.push_back(nextPlan);
    plan1 = nextPlan;

    ROS_INFO_STREAM("Plan 1 SUCCES");

    // plan1 = *lastPlan;
    // On the Objet
    for (auto planner : plannerIds){
        ROS_INFO_STREAM("Plan ON with : " << planner);
        nextPlan = nextCartesianPlan(plan1, planner, x-0.15, y, z-0.01, roll, pitch, yaw);
        if(_plan_success){break;}
    }

    if (!_plan_success) { ROS_INFO_STREAM("Plan FAILED - ON"); return false; }
    plans.push_back(nextPlan);
    plan1 = nextPlan;

    ROS_INFO_STREAM("Plan 2 - SUCCES");

    //EXECUTION
    ROS_INFO("Planning was succesfull, now moving");

    success = executePlans(plans);


    if (success) {
        ROS_INFO("Execution SUCCESS");
    } else {
        ROS_INFO("Execution FAILED");
    }


    gac.sendGoalAndWait(close_schunk_gripper_goal, ros::Duration(2));


    if (success)
    {
        ROS_INFO("Going up!");
        moveToCartesian(x-0.11, y, z+0.20, roll, pitch, yaw);
    }


    return success;
}
/**
// PLANNING BEFOR MOVING
void ArmInterfaceNode::pickPoseCB(const homodeus_prehension::PrehensionPos& prehensionPos)
{
    const homodeus_msgs::HDPose& hd_pose_msg = prehensionPos.hdpose;

    std::vector<moveit_msgs::CollisionObject> obstacles_list = prehensionPos.obstacles;
    addObstacles(obstacles_list);
    
    geometry_msgs::Pose pose = hd_pose_msg.pose;
    bool success = false;

    success = gotoGraspPrep();

    if(success) {
        success = makeAllPlans(pose);
    }

    bool end_succes = false;
    ROS_INFO("Go HOME");
    end_succes = goHome();
    if (!end_succes){
        ROS_INFO("FAILED TO GO HOME");

        cleanObstacles();
        return;
    }

    ROS_INFO("TO SAFE POSE");
    end_succes = gotoDropPrep();
    if (!end_succes){
        ROS_INFO("FAILED TO SAFE POSE");

        cleanObstacles();
        return;
    }
    ROS_INFO("SAFE pose reached - SENDING RESPONSE");

    homodeus_msgs::HDResponse hd_response_msg;
    hd_response_msg.id =  hd_pose_msg.id;
    hd_response_msg.result = success; 
    hbba_take_response_pub.publish(hd_response_msg);

    cleanObstacles();
}
**/

void ArmInterfaceNode::pickPoseCB(const homodeus_prehension::PrehensionPos& prehensionPos)
{
    const homodeus_msgs::HDPose& hd_pose_msg = prehensionPos.hdpose;

    std::vector<moveit_msgs::CollisionObject> obstacles_list = prehensionPos.obstacles;
    addObstacles(obstacles_list);
    
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

    ROS_INFO_STREAM("ROLL " << roll );
    ROS_INFO_STREAM("pitch " << pitch );
    ROS_INFO_STREAM("yaw " << yaw );
    pitch = pitch +1.571;

    std::vector<std::string> plannerIds = 
    {"SBLkConfigDefault",
    "RRTConnectkConfigDefault",
    "LBKPIECEkConfigDefault",
    "PRMkConfigDefault",
    "RRTkConfigDefault",
    "ESTkConfigDefault",
    "STOMP",
    "CHOMP"
    };

    z += 0.05;

    ROS_INFO("IN FRONT :");
    // for (auto planner : plannerIds){
    //     ROS_INFO_STREAM("Plan FRONT with : " << planner);
    //     success = moveToCartesian(x-0.3, y, z-0.01, roll, pitch, yaw, planner=planner);
    //     if(success){break;}
    // }
    success = moveToCartesian(x-0.3, y-0.015, z-0.01, roll, pitch, yaw);


    ROS_INFO("ON object :");
    if (success) {
        // for (auto planner : plannerIds){
        //     ROS_INFO_STREAM("Plan ON with : " << planner);
        //     success = moveToCartesian(x-0.11, y, z-0.01, roll, pitch, yaw, true, planner=planner);
        //     if(success){break;}
        // }
        success = moveToCartesian(x-0.10, y-0.015, z-0.01, roll, pitch, yaw, "SBLkConfigDefault", true);
    }

    if (success)
    {
        gac.sendGoalAndWait(close_schunk_gripper_goal, ros::Duration(2));
        ROS_INFO("Closed!");
        moveToCartesian(x-0.10, y-0.015, z+0.20, roll, pitch, yaw);
 
    }
    else
        ROS_INFO("arm_interface_node: failed to go to pick point!");

    if (success)
    {
        ROS_INFO("SUCCES - GOING BACK UP");
    } else {
        ROS_INFO("FAILED - GOING BACK UP");
    }

    bool end_succes = false;
    ROS_INFO("Go HOME");
    end_succes = goHome();
    if (!end_succes){
        ROS_INFO("FAILED TO GO HOME");
    }

    ROS_INFO("TO SAFE POSE");
    end_succes = gotoDropPrep();
    if (!end_succes){
        ROS_INFO("FAILED TO SAFE POSE");
    }

   



    if (success) {
         ROS_INFO("SAFE pose reached - SENDING RESPONSE SUCCESS");
        homodeus_msgs::HDResponse hd_response_msg;
        hd_response_msg.id =  hd_pose_msg.id;
        hd_response_msg.result = success; 
        hbba_take_response_pub.publish(hd_response_msg);

    }else {
         ROS_INFO("SAFE pose reached - SENDING RESPONSE FAILED");
        homodeus_msgs::HDStatus hd_status_msg;
        hd_status_msg.id =  hd_pose_msg.id;
        hbba_take_status_pub.publish(hd_status_msg);

    }

    // if (success) {
    //     ros::Duration(2).sleep();
    //     ROS_INFO("Now drop object.");
    //     drop_pose_pub.publish(hd_pose_msg);
    // }
    cleanObstacles();
}

void ArmInterfaceNode::dropPoseHard(const homodeus_prehension::PrehensionPos& prehensionPos){
    ROS_INFO("DROP - START");
    bool success = true;
    const homodeus_msgs::HDPose& hd_pose_msg = prehensionPos.hdpose;

    ROS_INFO("Going up to look position");
    tac.sendGoalAndWait(go_up, ros::Duration(3));
    // Deplace le joint5 seulement pour descendre 
    success = moveToJoint(0.35, 0.49, -1.31, -0.49, 2.17, 0.15, 1.39, -1.90);
    ROS_INFO("DROP - OPEN");

    actionlib::SimpleClientGoalState state = gac.sendGoalAndWait(open_schunk_gripper_goal, ros::Duration(2));
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_WARN("Gripper goal did not finish successfully, current state: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Gripper action succeeded.");
    }

    ROS_INFO("DROP - DO BACK");
    success = gotoDropPrep();

    ROS_INFO("DROP - END");
    homodeus_msgs::HDResponse hd_response_msg;
    hd_response_msg.id =  hd_pose_msg.id;
    hd_response_msg.result = success; 
    hbba_drop_response_pub.publish(hd_response_msg);
}

void ArmInterfaceNode::dropPoseCB(const homodeus_msgs::HDPose& hd_pose_msg)
{   
    geometry_msgs::Pose pose = hd_pose_msg.pose;
    bool success = true;

    // ROS_INFO("Going to drop preparation pose");
    // success = gotoGraspPrep();
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
    // success = moveToCartesian(x-0.4, y, z, roll, pitch, yaw);
    if (success)
    {
        ROS_INFO("arm_interface_node: reached first waypoint");
        success = moveToCartesian(x-0.15, y, z+0.05, roll, pitch, yaw);
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
        ROS_INFO("Going up!");
        
        moveToCartesian(x-0.2, y, z+0.2, roll, pitch, yaw);
        // success = gotoRetreat(pose);

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
    // success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 0.14, 0.0);
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
    // success = moveToJoint(0.34, 0.20, 0.79, -1.50, 1.60, -1.20, 0.14, 0.0);
    return success;
}

bool ArmInterfaceNode::gotoDropPrep()
{
    bool success;
    success = moveToJoint(0.35, 0.49, -1.31, -0.78, 2.17, -1.25, 1.39, -1.90);

    // Drop POS
    // success = moveToJoint(0.35, 0.07, -1.27, -0.28, 2.27, 0.18, 1.39, 1.16);
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
    return moveToCartesian(x-0.1, y, z+0.4, roll, pitch, yaw);

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
    // arm_node.gotoInitPose();
    

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
