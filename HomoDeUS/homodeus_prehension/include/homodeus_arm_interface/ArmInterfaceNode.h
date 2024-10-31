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
#include <homodeus_msgs/DesireID.h>
#include <homodeus_msgs/HDPose.h>


class ArmInterfaceNode: ArmInterface
{
    private:
        bool got_pick_pose = false;
        bool got_drop_pose = false;

        ros::NodeHandle nh;

        ros::Subscriber pick_pose_sub;
        ros::Subscriber drop_pose_sub;

        geometry_msgs::PoseStamped pick_point;
        geometry_msgs::PoseStamped drop_point;

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

        void pickPoseCB(const homodeus_msgs::HDPose& hd_pose_msg);
        void dropPoseCB(const homodeus_msgs::HDPose& hd_pose_msg);
        trajectory_msgs::JointTrajectory openedGripper();
        trajectory_msgs::JointTrajectory closedGripper();

        trajectory_msgs::JointTrajectory openedSchunkGripper();
        trajectory_msgs::JointTrajectory closedSchunkGripper();

        trajectory_msgs::JointTrajectory goUp();
        trajectory_msgs::JointTrajectory lookDown();

        
        ros::Publisher hbba_take_response_pub;
        ros::Publisher hbba_drop_response_pub;
        // FOR TEST ONLY : Temp Drop pub to drop after pick
        ros::Publisher drop_pose_pub;

    public:
        ArmInterfaceNode(ros::NodeHandle n);

        bool gotoGraspPrep();
        bool gotoRetreat(const geometry_msgs::Pose pose);
        bool goHome();
        bool gotoCarryPose();

        void gotoInitPose();
        void changeVelFactor();
        void closeHand();
};

#endif