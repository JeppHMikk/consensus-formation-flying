#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include <sstream>
//#include <Eigen/Core>
#include <Eigen/Dense>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <cmath>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Point.h"
#include <chrono>

using namespace std::chrono;

using namespace std;
using namespace Eigen;

int fs = 1000;
float ts = 1.0/float(fs);

const int N = 4; // number of robots

Matrix <int, 6, 1> joy_val; // joypad values
Matrix <float, 2, N> C; // base configuration
Matrix <float, 3, N> p; // robot positions

Matrix <float, 5, 1> eta;
Matrix <float, 5, 1> deta;

Matrix <float, 2, 5> J1;
Matrix <float, 2, 5> J2;
Matrix <float, 2, 5> J3;
Matrix <float, 2, 5> J4;

Matrix <float, 2, 1> v;
Matrix <float, 2, 1> v_rot;

Matrix <float, 3, 1> v1;
Matrix <float, 3, 1> v2;
Matrix <float, 3, 1> v3;
Matrix <float, 3, 1> v4;

Matrix <float, 2, 1> p1_ref;
Matrix <float, 2, 1> p2_ref;
Matrix <float, 2, 1> p3_ref;
Matrix <float, 2, 1> p4_ref;

Matrix <float, 2, 2> R;
Matrix <float, 2, 2> S;
Matrix <float, 2, 1> t;

float K = 0.1;

// subscriber callback for getting joypad values
void joyCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  joy_val << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
}

// subscriber callback for getting uav 1 odometry information
void pos1Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  p.col(0) << msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z;
}

// subscriber callback for getting uav 2 odometry information
void pos2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  p.col(1) << msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z;
}

// subscriber callback for getting uav 3 odometry information
void pos3Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  p.col(2) << msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z;
}

// subscriber callback for getting uav 4 odometry information
void pos4Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  p.col(3) << msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z;
}

int main(int argc, char **argv)
{

  /*
  float phi = 3.14/4;
  Matrix <float, 2, 2> R;
  R << cos(phi), -sin(phi),
       sin(phi), cos(phi);

  */

  C.col(0) << 4, 4;
  C.col(1) << -4, 4;
  C.col(2) << -4, -4;
  C.col(3) << 4, -4;

  eta << 0, 1, 1, 0, 0;
  S << 2, 0,
       0, 2;

  // Initialize ROS node
  ros::init(argc, argv, "formation_control");
  ros::NodeHandle n;
  
  // Subscribe to joypad publisher
  ros::Subscriber joy_sub = n.subscribe("joy_value", 1, joyCallback);

  // Subscribe to robot odometry topics
  ros::Subscriber pos_sub_1 = n.subscribe("uav1/estimation_manager/odom_main", 1, pos1Callback);
  ros::Subscriber pos_sub_2 = n.subscribe("uav2/estimation_manager/odom_main", 1, pos2Callback);
  ros::Subscriber pos_sub_3 = n.subscribe("uav3/estimation_manager/odom_main", 1, pos3Callback);
  ros::Subscriber pos_sub_4 = n.subscribe("uav4/estimation_manager/odom_main", 1, pos4Callback);

  // Initialize velocity reference publisher
  ros::Publisher vel_ref_pub_1 = n.advertise<mrs_msgs::VelocityReferenceStamped>("uav1/control_manager/velocity_reference", 1);
  ros::Publisher vel_ref_pub_2 = n.advertise<mrs_msgs::VelocityReferenceStamped>("uav2/control_manager/velocity_reference", 1);
  ros::Publisher vel_ref_pub_3 = n.advertise<mrs_msgs::VelocityReferenceStamped>("uav3/control_manager/velocity_reference", 1);
  ros::Publisher vel_ref_pub_4 = n.advertise<mrs_msgs::VelocityReferenceStamped>("uav4/control_manager/velocity_reference", 1);

  // Set rate of loop
  ros::Rate loop_rate(fs);

  while (ros::ok())
  {

    auto start = high_resolution_clock::now();

    // Transform joypad values into parameter derivatives
    deta << 0.2*(float(joy_val(4,0))/32767.0 - float(joy_val(5,0))/32767.0),
            0.5*float(joy_val(0,0))/32767.0,
            -0.5*float(joy_val(1,0))/32767.0,
            2*float(joy_val(2,0))/32767.0,
            -2*float(joy_val(3,0))/32767.0;

    //cout << deta << endl << endl;

    J1 << -sin(eta(0,0))*eta(1,0)*C(0,0) -cos(eta(0,0))*eta(2,0)*C(1,0), cos(eta(0,0))*C(0,0), -sin(eta(0,0))*C(1,0), 1, 0,
          cos(eta(0,0))*eta(1,0)*C(0,0) -sin(eta(0,0))*eta(2,0)*C(1,0), sin(eta(0,0))*C(0,0), cos(eta(0,0))*C(1,0), 0, 1;
    J2 << -sin(eta(0,0))*eta(1,0)*C(0,1) -cos(eta(0,0))*eta(2,0)*C(1,1), cos(eta(0,0))*C(0,1), -sin(eta(0,0))*C(1,1), 1, 0,
          cos(eta(0,0))*eta(1,0)*C(0,1) -sin(eta(0,0))*eta(2,0)*C(1,1), sin(eta(0,0))*C(0,1), cos(eta(0,0))*C(1,1), 0, 1;
    J3 << -sin(eta(0,0))*eta(1,0)*C(0,2) -cos(eta(0,0))*eta(2,0)*C(1,2), cos(eta(0,0))*C(0,2), -sin(eta(0,0))*C(1,2), 1, 0,
          cos(eta(0,0))*eta(1,0)*C(0,2) -sin(eta(0,0))*eta(2,0)*C(1,2), sin(eta(0,0))*C(0,2), cos(eta(0,0))*C(1,2), 0, 1;
    J4 << -sin(eta(0,0))*eta(1,0)*C(0,3) -cos(eta(0,0))*eta(2,0)*C(1,3), cos(eta(0,0))*C(0,3), -sin(eta(0,0))*C(1,3), 1, 0,
          cos(eta(0,0))*eta(1,0)*C(0,3) -sin(eta(0,0))*eta(2,0)*C(1,3), sin(eta(0,0))*C(0,3), cos(eta(0,0))*C(1,3), 0, 1;

    R << cos(eta(0,0)), -sin(eta(0,0)),
         sin(eta(0,0)), cos(eta(0,0));
    S.diagonal() = eta.block(1,0,2,1);
    t << eta.block(3,0,2,1);

    p1_ref = R*S*C.block(0,0,2,1) + t;
    p2_ref = R*S*C.block(0,1,2,1) + t;
    p3_ref = R*S*C.block(0,2,2,1) + t;
    p4_ref = R*S*C.block(0,3,2,1) + t;

    v1.block(0,0,2,1) << 0.2*J1*deta + K*(p1_ref.block(0,0,2,1) - p.block(0,0,2,1));
    v2.block(0,0,2,1) << 0.2*J2*deta + K*(p2_ref.block(0,0,2,1) - p.block(0,1,2,1));
    v3.block(0,0,2,1) << 0.2*J3*deta + K*(p3_ref.block(0,0,2,1) - p.block(0,2,2,1));
    v4.block(0,0,2,1) << 0.2*J4*deta + K*(p4_ref.block(0,0,2,1) - p.block(0,3,2,1));

    v1(2,0) = K*(1.5 - p(2,0));
    v2(2,0) = K*(1.5 - p(2,1));
    v3(2,0) = K*(1.5 - p(2,2));
    v4(2,0) = K*(1.5 - p(2,3));

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    float bias = 0.000001*float(duration.count());

    eta += (ts + bias)*deta;

    mrs_msgs::VelocityReferenceStamped msg1;
    msg1.reference.velocity.x = v1(0,0); 
    msg1.reference.velocity.y = v1(1,0);
    msg1.reference.velocity.z = v1(2,0);
    vel_ref_pub_1.publish(msg1);

    mrs_msgs::VelocityReferenceStamped msg2;
    msg2.reference.velocity.x = v2(0,0); 
    msg2.reference.velocity.y = v2(1,0);
    msg2.reference.velocity.z = v2(2,0);
    vel_ref_pub_2.publish(msg2);

    mrs_msgs::VelocityReferenceStamped msg3;
    msg3.reference.velocity.x = v3(0,0); 
    msg3.reference.velocity.y = v3(1,0);
    msg3.reference.velocity.z = v3(2,0);
    vel_ref_pub_3.publish(msg3);

    mrs_msgs::VelocityReferenceStamped msg4;
    msg4.reference.velocity.x = v4(0,0); 
    msg4.reference.velocity.y = v4(1,0);
    msg4.reference.velocity.z = v4(2,0);
    vel_ref_pub_4.publish(msg4);
            
    ros::spinOnce();
    loop_rate.sleep();

  }


  return 0;
}
