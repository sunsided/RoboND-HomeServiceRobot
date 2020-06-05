#include <cmath>
#include <vector>
#include <array>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>

namespace {
    auto current_x = 0.0,
         current_y = 0.0;

    const auto location_tolerance = 0.15;

    // TODO: Code duplication ... should be received from a topic instead.
    const std::vector<std::array<float, 3>> subgoals = {
            {8.91, 2.93, -M_PI},
            {9.07,   -3.14, -1.55}
    };

    const auto shape = visualization_msgs::Marker::SPHERE;
}

void onOdometryReceived(const nav_msgs::Odometry::ConstPtr &msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(5);
    auto marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    auto odom_sub = n.subscribe("/odom", 100, onOdometryReceived);

    // Ensure we have subscribers for the marker.
    while (marker_pub.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = subgoals[0][0];
    marker.pose.position.y = subgoals[0][1];
    marker.pose.position.z = 0.25;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(subgoals[0][2]);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // TODO: Adjust to more than one item.
    auto have_item = false;
    while (ros::ok()) {
        if (!have_item) {
            marker_pub.publish(marker);
            const auto dx = std::abs(marker.pose.position.x - current_x);
            const auto dy = std::abs(marker.pose.position.y - current_y);
            const auto location_reached = (dx <= location_tolerance) &&
                                          (dy <= location_tolerance);

            if (location_reached) {
                marker.action = visualization_msgs::Marker::DELETE;
                marker_pub.publish(marker);
                have_item = true;
            }
        } else {
            const auto dx = std::abs(subgoals[1][0] - current_x);
            const auto dy = std::abs(subgoals[1][1] - current_y);
            const auto location_reached = (dx <= location_tolerance) &&
                                          (dy <= location_tolerance);

            if (location_reached) {
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = subgoals[1][0];
                marker.pose.position.y = subgoals[1][1];
                marker.pose.orientation = tf::createQuaternionMsgFromYaw(subgoals[1][2]);

                marker_pub.publish(marker);
                // have_item = false;

                // No point in continuing the work here.
                break;
            }
        }

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
