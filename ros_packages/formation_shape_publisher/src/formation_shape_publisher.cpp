#include "ros/ros.h"
#include "std_msgs/String.h"
#include "formation_shape_publisher/formation_shape.h"

using namespace std;

#include <sstream>

int N = 10;
string default_formation_shape = "grid";
string formation_shape = default_formation_shape;

bool formationShapeCallback(formation_shape_publisher::formation_shape::Request& req, formation_shape_publisher::formation_shape::Response& res){
  if(strcmp(req.formation_shape.c_str(),"grid") == 0){
    ROS_INFO("[FormationController]: Switched formation to grid.");
    formation_shape = "grid";
    res.success = true;
    res.message = "Switched formation to grid";
  }
  else if(strcmp(req.formation_shape.c_str(),"line") == 0){
    ROS_INFO("[FormationController]: Switched formation to line.");
    formation_shape = "line";
    res.success = true;
    res.message = "Switched formation to line";
  }
  else if(strcmp(req.formation_shape.c_str(),"circle") == 0){
    ROS_INFO("[FormationController]: Switched formation to circle.");
    formation_shape = "circle";
    res.success = true;
    res.message = "Switched formation to circle";
  }
  else if(strcmp(req.formation_shape.c_str(),"triangle") == 0){
    ROS_INFO("[FormationController]: Switched formation to triangle.");
    formation_shape = "triangle";
    res.success = true;
    res.message = "Switched formation to triangle";
  }
  else{
    ROS_INFO("[FormationController]: Wrong entry. Switched to default grid formation.");
    formation_shape = default_formation_shape;
    res.success = false;
    res.message = "Wrong entry: Switched to default grid formation";
  }
  return true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "formation_shape_publisher");
  ros::NodeHandle n;
  ros::Publisher shape_pub = n.advertise<std_msgs::String>("formation_shape", 1000);
  // Initialize formation shape service
  ros::ServiceServer formation_shape_service = n.advertiseService("formation_shape_srv", formationShapeCallback);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    std_msgs::String msg;

    msg.data = formation_shape;

    ROS_INFO_ONCE("[%s]: Publishing default shape: %s.", ros::this_node::getName().c_str(), msg.data.c_str());
    shape_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
