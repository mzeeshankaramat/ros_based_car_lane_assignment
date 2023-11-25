/*
 * @filename    lane_assignment_node.cpp
 * @project     Project
 * @author      Zeeshan
 */

#include <ros/ros.h>
#include <lane_assignment/cLaneAssignment.h>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "lane_assignment_node");
   ros::NodeHandle node;
   ros::NodeHandle nodePrivate("~");

   cLaneAssignment oLaneAssignment(node, nodePrivate);

   ros::spin();
}
