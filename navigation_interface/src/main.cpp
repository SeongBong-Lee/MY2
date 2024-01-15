#include <ros/ros.h>
#include "navigation_interface/navigation_interface.h"


int main(int argc, char ** argv)
{ 
  ros::init(argc, argv, "navigation_interface");

  navigation_interface::Publisher* nav_if;
  nav_if = new navigation_interface::Publisher();

  ROS_INFO("[NavigationInterface] Initialized");
  ros::spin();
  ROS_INFO("[NavigationInterface] Good Bye");

  delete nav_if;


  return 0;
}