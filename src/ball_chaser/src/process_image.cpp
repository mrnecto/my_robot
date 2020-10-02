#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
   ball_chaser::DriveToTarget srv;
   srv.request.linear_x = lin_x;
   srv.request.angular_z = ang_z;

   // Call the drive_bot service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_bot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
   int white_pixel = 255*3;               //R255+G255+B255
   int height = img.height*img.step/2;    //use the middle of the height of the picture
   bool ball_found;
   int ball_pose;
      for(int i = 0; i < img.step; i+=3)  //looking for the white ball
      {
         if(img.data[i+height] + img.data[i+height + 1] + img.data[i+height + 2]  == white_pixel)
         {
            ball_found = true;
            ball_pose = i;
            break;
         }
         else ball_found = false;
      }
      
      if(ball_found) 
      {
           if(ball_pose > 7*img.step/16 && ball_pose < 9*img.step/16)drive_robot(1, 0);  //drive forward
           if(ball_pose < 7*img.step/16)drive_robot(0, 3);                           //turn left
           if(ball_pose > 9*img.step/16)drive_robot(0, -3);                         //turn right  
      }
      else drive_robot(0, 0); //stop robot
      
   

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
