#include <vector>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

namespace {
    const auto delay_seconds = 5.0;
    const std::vector<std::array<float, 3>> subgoals = {
            {-3, -5, 2.23},
            {3,   2, 0}
    };
}


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv) {
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // TODO: send the target positions as actual messages, subscribe to them.
    for (const auto& subgoal : subgoals) {
        goal.target_pose.pose.position.x = subgoal[0];
        goal.target_pose.pose.position.y = subgoal[1];
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(subgoal[2]);

        // Send the goal position and orientation for the robot to reach
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        // Wait an infinite time for the results
        ac.waitForResult();

        // Check if the robot reached its goal
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal reached!");
        } else {
            ROS_INFO("Failed to reach goal for some reason");
        }

        ros::Duration(delay_seconds).sleep();
    }

    return 0;
}
