#include <iostream>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

int main(int argc, char **argv)
{
    /*ros initalize*/
    ros::init(argc, argv, "rrbot_move");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /*moveit initializing*/
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    
    arm.setGoalJointTolerance(0.001);
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    /*controlling robot to setted position*/
    arm.setNamedTarget("stand_by");
    arm.move();
    sleep(1);

    /*getting current joint value and show in terminal*/
    vector<double> currentJointState=arm.getCurrentJointValues();
    for(auto & i:currentJointState)
    {
        cout<< i <<", ";
    }
    cout<<endl<<endl;

    /*move 4th joint -90 degree*/
    currentJointState[3]-=3.141592/2;
    arm.setJointValueTarget(currentJointState);
    arm.move();

    /*move 4th joint +90 degree*/
    currentJointState[3]+=3.141592/2;
    arm.setJointValueTarget(currentJointState);
    arm.move();
    sleep(1);

    /*get current pose*/
    geometry_msgs::PoseStamped currentPose;
    currentPose=arm.getCurrentPose();
    cout<<currentPose<<endl;

    /*move end tip to impossible pose*/
    geometry_msgs::Pose movePose;
    movePose.position.x=currentPose.pose.position.x+0.2;
    movePose.position.y=currentPose.pose.position.y;
    movePose.position.z=currentPose.pose.position.z+0.2;
    movePose.orientation=currentPose.pose.orientation;
    arm.setPoseTarget(movePose);
    bool flag = (arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    cout<<flag<<endl;

    /* move to possible pose */
    movePose.position.x=currentPose.pose.position.x+0.1;
    movePose.position.y=currentPose.pose.position.y;
    movePose.position.z=currentPose.pose.position.z;
    movePose.orientation=currentPose.pose.orientation;
    arm.setPoseTarget(movePose);
    flag = (arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    cout<<flag<<endl;
    arm.move();

    /*rotate roll -90degree*/
    currentPose=arm.getCurrentPose();
    movePose.position=currentPose.pose.position;

    tf2::Quaternion q_orig, q_rot, q_new;
    double r=3.141502/2, p=0, y=0;
    
    tf2::convert(currentPose.pose.orientation, q_orig);
    q_rot.setRPY(r,p,y);
    q_new=q_rot*q_orig;
    q_new.normalize();
    tf2::convert(q_new, movePose.orientation);
    
    arm.setPoseTarget(movePose);
    arm.move();
    sleep(1);

    return 0;
}
