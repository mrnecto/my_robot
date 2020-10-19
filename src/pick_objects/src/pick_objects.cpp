#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  std_msgs::String msg;
  std::stringstream ss;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -1.0;
  goal.target_pose.pose.position.y = 3.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending to pick up the virtual object");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
   {
       ss << "P";
       msg.data = ss.str();
       chatter_pub.publish(msg);
       ROS_INFO("Robot picked up the virtual object");
   }
  else
    ROS_INFO("Robot can't reach the virtual object");

  //Wait 5 sec
   ROS_INFO("Robot waits 5 sec");   
  ros::Duration(5.0).sleep();

   // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.position.y = -2.0; 
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending to drop off the virtual object");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
       ss << "D";
       msg.data = ss.str();
       chatter_pub.publish(msg);
       ROS_INFO("Robot dropped off the virtual object");
    }
  else
    ROS_INFO("Robot can't reach drop off location");

   sleep(5);

  return 0;
}
