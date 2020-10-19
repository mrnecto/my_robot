#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>

bool pickup = false;
bool dropoff = false;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
   //ROS_INFO("%s",msg->data.c_str());   
   if(msg->data == "P") pickup = true;
   if(msg->data == "PD") dropoff = true;
   
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "home_service");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber chatter = n.subscribe("chatter",1000, chatterCallback);
  
  double x_pos = -1.0;
  double y_pos = 3.0;
    
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    
    marker.pose.position.x = x_pos;
    marker.pose.position.y = y_pos;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
   
   while (ros::ok())
  {
    ros::spinOnce();
    
    if(pickup)
    {
         ROS_INFO("The object picked up");         
         marker.action = visualization_msgs::Marker::DELETE;
         sleep(5);
         pickup = false;
    } 

   if(dropoff)
   {
         ROS_INFO("The object dropped off");         
         marker.action = visualization_msgs::Marker::ADD;
         marker.pose.position.x = 2.0;
         marker.pose.position.y = -2.0;
         sleep(5);
         dropoff = false;
   }

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

   marker_pub.publish(marker);
   r.sleep();
  }
   
}
