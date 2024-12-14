#ifndef ARMINTERFACENODE_H
#define ARMINTERFACENODE_H

#include <string>
#include <homodeus_arm_interface/ArmInterface.h>

#include <tf_conversions/tf_eigen.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <homodeus_msgs/HDResponse.h>
#include <homodeus_msgs/HDStatus.h>
#include <homodeus_msgs/DesireID.h>
#include <homodeus_prehension/PrehensionPos.h>
#include <moveit_msgs/CollisionObject.h>
#include <pal_control_msgs/ActuatorCurrentLimit.h>


class ArmInterfaceNode: ArmInterface
{
    private:
        ros::NodeHandle nh;

        ros::Subscriber pick_pose_sub;
        ros::Subscriber drop_pose_sub;


        // Gripper client
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gac;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> tac;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> hac;

        control_msgs::FollowJointTrajectoryGoal close_gripper_goal;
        control_msgs::FollowJointTrajectoryGoal open_gripper_goal;
        control_msgs::FollowJointTrajectoryGoal close_schunk_gripper_goal;
        control_msgs::FollowJointTrajectoryGoal open_schunk_gripper_goal;
        control_msgs::FollowJointTrajectoryGoal go_up;
        control_msgs::FollowJointTrajectoryGoal look_down;

        void pickPoseCB(const homodeus_prehension::PrehensionPos& hd_pose_msg);
        void dropPoseCB(const homodeus_prehension::PrehensionPos& hd_pose_msg);
        void dropPoseSafeCB(const homodeus_prehension::PrehensionPos& hd_pose_msg);

        trajectory_msgs::JointTrajectory openedGripper();
        trajectory_msgs::JointTrajectory closedGripper();

        trajectory_msgs::JointTrajectory openedSchunkGripper();
        trajectory_msgs::JointTrajectory closedSchunkGripper();

        trajectory_msgs::JointTrajectory goUp();
        trajectory_msgs::JointTrajectory lookDown();

        ros::Publisher hbba_take_response_pub;
        ros::Publisher hbba_take_status_pub;
        ros::Publisher hbba_drop_response_pub;
        ros::Publisher hbba_drop_status_pub;

    public:
        ArmInterfaceNode(ros::NodeHandle n);

        bool gotoGraspPrep();
        bool goHome();
        bool gotoCarryPose();

        void gotoInitPose();
};

#endif