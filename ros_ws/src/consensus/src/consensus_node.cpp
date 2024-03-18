#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Dense>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/TrajectoryReference.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

#include <sstream>
#include <random>
#include <ctime>
#include <cstdlib>
#include <ctime>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <functional>
#include <boost/bind.hpp>

using namespace std;
using namespace Eigen;

const int N = 4; // number of robots
const float clearance = 2;
const float max_vel = 10.0;
const float rho_0 = 2.0;
const float rho_1 = 5.0;
const float b = max_vel + 1.0;
const float a = -1/2*b*1/(rho_1 - rho_0);
const float g = b*(rho_1 - 1/2*(rho_1 - rho_0));
const float r = 0.4; // robot minimum containing sphere radius
const float Br = 2*r + clearance; // distance where collisions is assumed to happen

Matrix <float, 3, N> p; // robot position
Matrix <float, 3, 3> Sigma[N]; // position estimate covariance

Matrix <float, 5, 1> eta; // formation parameters (phi,sx,sy,tx,ty)
Matrix <float, 5, 1> deta; // formation parameter derivative

Matrix <float, 5, N-1> eta_N; // formation parameters (phi,sx,sy,tx,ty)

Matrix <float, 5, 1> deta_N; // consensus formation parameter derivative
Matrix <float, 5, 1> deta_joy;
Matrix <float, 5, 1> deta_rep;

Matrix <float, 2, N> C; // base configuration

Matrix <int, 6, 1> joy_val; // joypad values

Matrix <float, 2, 2> c_obst;
Matrix <float, 2, 1> r_obst;

Matrix <float, 2, 2> R90;

int arraySize = 5;
const char* uavName = std::getenv("UAV_NAME"); // load UAV_NAME environment variable
std::string uavNameString(uavName); // convert uavName to a string
int uavNum = uavNameString[3] - '0'; // extract uav number

int ros_rate = 10; // ROS rate
float ts = 1e-2; //1.0/float(ros_rate); // sampling time

float alpha = 0.25; // Exponential moving average filter coefficient

float Kv = 0.25;

// Subscriber callback for getting joypad values
void joyCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  deta_joy << 1*(float(msg->data[4])/32767.0 - float(msg->data[5])/32767.0),
              2*float(msg->data[0])/32767.0,
              -2*float(msg->data[1])/32767.0,
              2.5*float(msg->data[2])/32767.0,
              -2.5*float(msg->data[3])/32767.0;
}

void etaCallback(int id, const std_msgs::Float32MultiArray::ConstPtr& msg){
  for(int i = 0; i < 5; i++){
    eta_N(i,id) = msg->data[i];
  }
}

// Subscriber callback for getting position estimates
void posCallback(int id, const nav_msgs::Odometry::ConstPtr& msg) {
    // extract position from message
    p.block(0,id,3,1) << msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z;
    // extract covariance from message
    Eigen::Matrix <float, 6, 6> Sigma_msg;
    for(int i=0; i<6; i++){
        for(int j = 0; j<6; j++){
            Sigma_msg(i,j) = msg->pose.covariance[6*i+j];
        }
    }
    Sigma[id] << Sigma_msg.block(0,0,3,3);
}

// Function for generation random float
float randomFloat(){
    return (float)(rand()) / (float)(rand());
}

// Function for calculating Jacobian 
Eigen::MatrixXf Jacobian(Eigen::MatrixXf eta_in){
  Matrix <float, 2, 5> J;
  J << -sin(eta_in(0,0))*eta_in(1,0)*C(0,uavNum-1) -cos(eta_in(0,0))*eta_in(2,0)*C(1,uavNum-1), cos(eta_in(0,0))*C(0,uavNum-1), -sin(eta_in(0,0))*C(1,uavNum-1), 1, 0,
        cos(eta_in(0,0))*eta_in(1,0)*C(0,uavNum-1) -sin(eta_in(0,0))*eta_in(2,0)*C(1,uavNum-1), sin(eta_in(0,0))*C(0,uavNum-1), cos(eta_in(0,0))*C(1,uavNum-1), 0, 1;
  return J;
}

// Rotation matrix function
Eigen::MatrixXf rotMat(Eigen::MatrixXf eta_in){
  Matrix <float, 2, 2> R;
  R << cos(eta_in(0,0)), -sin(eta_in(0,0)),
       sin(eta_in(0,0)),  cos(eta_in(0,0));
  return R;
}

// Scaling matrix function
Eigen::MatrixXf scaleMat(Eigen::MatrixXf eta_in){
  Matrix <float, 2, 2> S;
  S << eta_in(1,0), 0,
       0     , eta_in(2,0);
  return S;
}

// Translation vector function
Eigen::MatrixXf transVec(Eigen::MatrixXf eta_in){
  Matrix <float, 2, 1> t;
  t << eta_in(3,0),
       eta_in(4,0);
  return t;
}

// Reference position from transformation of base configuration
Eigen::MatrixXf refPos(Eigen::MatrixXf eta_in){
  Eigen::MatrixXf R = rotMat(eta_in);
  Eigen::MatrixXf S = scaleMat(eta_in);
  Eigen::MatrixXf t = transVec(eta_in);
  Eigen::Matrix <float, 3, 1> p;
  p << R*S*C.col(uavNum-1) + t, 1.5;
  return p;
}

// Reference velocity from parameter derivative
Eigen::MatrixXf refVel(Eigen::MatrixXf eta_in, Eigen::MatrixXf deta_in){
  Eigen::Matrix <float, 3, 5> J;
  J << Jacobian(eta_in), 
       0, 0, 0, 0, 0;
  return Kv*J*deta_in;
}

// Repulsive potential for collision avoidance
Eigen::MatrixXf repPot(Eigen::MatrixXf p_in){
  Eigen::Matrix <float, 2, 1> v_rep;
  v_rep << 0,
           0;
  for(int i=0; i < 2; i++){
    float rho = (c_obst.col(i) - p_in.block(0,0,2,1)).norm() - r_obst(i,0) - clearance;
    Eigen::MatrixXf drho = (p_in.block(0,0,2,1) - c_obst.col(i))/((p_in.block(0,0,2,1) - c_obst.col(i)).norm() + 0.00001);
    float df;
    if(rho >= rho_1)
      df = 0.0;
    else if(rho >= rho_0 & rho < rho_1)
      df = 2*a*(rho - rho_0);
    else
      df = b;
    v_rep = v_rep + df*drho;
  }
  return v_rep;
}

// Function for mapping from velocity to parameter derivative
Eigen::MatrixXf v2deta(Eigen::MatrixXf v, Eigen::MatrixXf eta_in){
  Eigen::MatrixXf J = Jacobian(eta_in);
  Eigen::MatrixXf J_inv = J.transpose()*(J*J.transpose()).inverse();
  Eigen::MatrixXf deta = J_inv*v;
  return deta;
}

// CDF for normal distribution
float normalCDF(float x, float mu, float sigma){
  cout << 0.5*(1.0 + erf((x-mu)/(sigma*sqrt(2.0)))) << endl;
  cout << "--------------" << endl;
  return 0.5*(1.0 + erf((x-mu)/(sigma*sqrt(2.0))));
}

// Function for calculating upper bounding probability of collision
float colProb(Eigen::MatrixXf p1, Eigen::MatrixXf p2, Eigen::MatrixXf Sigma1, Eigen::MatrixXf Sigma2){
  Eigen::MatrixXf mu = p2.block(0,0,2,1) - p1.block(0,0,2,1);
  Eigen::MatrixXf Sigma = Sigma1.block(0,0,2,2) + Sigma2.block(0,0,2,2);
  Eigen::Matrix <float, 2, 2> R90;
  R90 << 0, -1, 1, 0;
  Eigen::MatrixXf a = mu/mu.norm();
  Eigen::MatrixXf mu_proj = a.transpose()*mu;
  Eigen::MatrixXf sigma_proj = a.transpose()*Sigma*a;
  float p_col = normalCDF(Br,mu_proj(0,0),sqrt(sigma_proj(0,0)));
  return p_col;
}

// Function for generating linear constraint for collision avoidance
tuple<Eigen::MatrixXf, float> genConst(Eigen::MatrixXf s, Eigen::MatrixXf Sigmai, Eigen:: MatrixXf Sigmaj, Eigen:: MatrixXf Ci, Eigen::MatrixXf Cj, float m){
  // Find robot distance covariance matrix
  Eigen::MatrixXf Sigmaij = Sigmai + Sigmaj;
  // Calculate largest eigenvalue of distance covariance matrix
  float lambda_max = Sigmaij.eigenvalues().real().maxCoeff();
  // Calculate DeltaC matrix
  Eigen::MatrixXf DeltaC = (Cj - Ci).asDiagonal();
  Eigen::MatrixXf Gamma_ij = DeltaC.transpose()*DeltaC;
  float gamma_ij = pow(Br + m*sqrt(lambda_max),2);
  // Calculate linear constraint
  Eigen::MatrixXf a_ij = Gamma_ij*s;
  float a = (s.transpose()*Gamma_ij.transpose()*Gamma_ij*Gamma_ij*s).value();
  float b = (-2*s.transpose()*Gamma_ij.transpose()*Gamma_ij*s).value();
  float c = (s.transpose()*Gamma_ij*s).value() - gamma_ij;
  float alpha = min((-b - sqrt(pow(b,2)-4*a*c))/(2*a),(-b + sqrt(pow(b,2)-4*a*c))/(2*a+0.0001));
  float b_ij = (s.transpose()*Gamma_ij*s - alpha*s.transpose()*Gamma_ij.transpose()*Gamma_ij*s).value();
  b_ij = b_ij/(a_ij.norm());
  a_ij = a_ij/(a_ij.norm());
  return make_tuple(a_ij,b_ij);
}

// Function for calculating smallest element and index for Eigen vector 
tuple<float, int> smEl(Eigen::VectorXf x){
  int M = x.rows();
  int sm_id = 0;
  float sm_el = x(sm_id,0);
  for(int i=1; i<M; i++){
    if(x(i,0) < sm_el){
      sm_el = x(i,0);
      sm_id = i;
    }
  }
  return make_tuple(sm_el,sm_id);
}

// Function for projecting scaling variable onto constraint set
Eigen::MatrixXf projScale(Eigen::MatrixXf s, Eigen::MatrixXf A, Eigen::MatrixXf b){

  MatrixXf s0 = s;

  // Check if current scaling is feasible
  if(((A*s).array() >= b.array()).all()){
    return s;
  }
  else{

    cout << to_string(time(NULL)) + ", projecting" << endl;

    // Find the most violated constraint
    auto tmp = smEl(A*s-b);
    int i = get<1>(tmp);
    //float alpha = (b.row(i)-A.row(i)*s).value()/(A.row(i)*A.row(i).transpose()).value();
    //s = s + alpha*A.row(i).transpose();

    MatrixXf Aw = A.row(i);
    MatrixXf bw = b.row(i);

    int iter = 0;

    while(iter < 100){

      iter++;

      int M = Aw.rows();
      // Build KKT matrix
      Eigen::MatrixXf K(2+M,2+M);
      K << Eigen::MatrixXf::Identity(2,2), Aw.transpose(),
          Aw, Eigen::MatrixXf::Zero(M,M);
      Eigen::MatrixXf t(2+M,1);
      t << (s0 - s), bw;
      // Solve LCQP problem
      MatrixXf sol = K.completeOrthogonalDecomposition().pseudoInverse()*t;
      // Extract optimal solution and Lagrange multipliers
      MatrixXf ds_opt = sol.block(0,0,2,1);
      MatrixXf lambda_opt = sol.block(2,0,M,1);

      if(((A*(s + ds_opt)).array() < b.array() - 0.0001).any()){
        // If solution is infeasible, add most violated constraint
        auto tmp = smEl(A*s-b);
        int i = get<1>(tmp);
        Aw.conservativeResize(Aw.rows()+1,Eigen::NoChange);
        Aw.row(Aw.rows()-1) = A.row(i);
        bw.conservativeResize(bw.rows()+1,Eigen::NoChange);
        bw.row(bw.rows()-1) = b.row(i);
      }
      else
      {
        // If all Lagrange multipliers are negative, the optimiser is done
        if((lambda_opt.array() <= 0).all()){
          return s + ds_opt;
        }
        // If there is a positive Lagrange multiplier, remove the constraint associated with the smallest one
        else{
          // Find the smallest Lagrange multiplier and its index
          auto tmp = smEl(lambda_opt);
          int r = get<1>(tmp);
          // Remove constraint
          Eigen::MatrixXf Aw_new(Aw.rows()-1, A.cols());
          Eigen::MatrixXf bw_new(bw.rows()-1, bw.cols());
          if(r > 1 && r < Aw.rows()){
            Aw_new.block(0,0,r,Aw.cols()) = Aw.block(0,0,r,Aw.cols());
            Aw_new.block(r,0,Aw.rows()-r,Aw.cols()) = Aw.block(r+1,0,Aw.rows()-r,Aw.cols());
          }
          else if(r == 1){
            Aw_new = Aw.block(0,0,Aw.rows()-1,Aw.cols());
            bw_new = bw.block(0,0,bw.rows()-1,bw.cols());
          }
          else{
            Aw_new = Aw.block(1,0,Aw.rows()-1,Aw.cols());
            bw_new = bw.block(1,0,bw.rows()-1,bw.cols());
          }

          Aw.conservativeResize(Aw.rows()-1,Eigen::NoChange);
          bw.conservativeResize(bw.rows()-1,Eigen::NoChange);
          Aw = Aw_new;
          bw = bw_new;  
        }
      }
    }
  }

  return s0;

}

int main(int argc, char **argv)
{

  R90 << 0, -1, 1, 0;

  // Initialise obstacle positions
  c_obst.col(0) << 10, 20;
  r_obst(0,0) = 2;
  c_obst.col(1) << -10, 20;
  r_obst(1,0) = 2;

  // Initialize base configuration
  C.col(0) << 1, 1;
  C.col(1) << -1, 1;
  C.col(2) << -1, -1;
  C.col(3) << 1, -1;

  
  // Fill eta with random numbers
  /*
  srand(time(NULL));
  for(int i = 0; i < 5; i++){
    eta(i,0) = randomFloat();
    if(i == 1 || i == 2)
      eta(i,0) += 4;
    //std::cout << eta(i,0) << std::endl;
  }
  */
  // Initialize eta
  eta << 0, 5, 5, 0, 0;
  for(int i=0; i<N-1; i++){
    eta_N.block(0,i,5,1) = eta;
  }

  // Initialize ROS node
  ros::init(argc, argv, uavNameString + "_consensus_node");
  ros::NodeHandle n;
  
  // Initialize publisher for consensus
  ros::Publisher consensus_pub = n.advertise<std_msgs::Float32MultiArray>("/" + uavNameString + "/eta", 1);

  // Initialize subscriber to joypad publisher
  ros::Subscriber joy_sub = n.subscribe("joy_value", 1, joyCallback);

  // Initialize subscribers for consensus
  ros::Subscriber consensus[N-1];
  int iter = 0;
  for(int i=1; i<=N; i++){
    if(i != uavNum){
      consensus[iter] = n.subscribe<std_msgs::Float32MultiArray>("uav"+to_string(iter+1)+"/eta", 1000, boost::bind(etaCallback, iter, _1));
      iter = iter + 1;
    }
  }

  // Initialize subscribers for position estimates
  ros::Subscriber pos_sub[N];
  for(int i=1; i<=N; i++){
    pos_sub[i-1] = n.subscribe<nav_msgs::Odometry>("uav"+to_string(i)+"/estimation_manager/odom_main", 1, boost::bind(posCallback, i-1, _1));
  }

  // Initialize velocity reference publisher
  ros::Publisher vel_ref_pub = n.advertise<mrs_msgs::VelocityReferenceStamped>(uavNameString+"/control_manager/velocity_reference", 1);

  // Initialize position reference publisher
  ros::Publisher pos_ref_pub = n.advertise<mrs_msgs::ReferenceStamped>(uavNameString+"/control_manager/reference", 1);

  // Initialize cylinder obstacle marker publisher
  ros::Publisher marker_pub1 = n.advertise<visualization_msgs::Marker>("visualization_marker_1", 1);
  ros::Publisher marker_pub2 = n.advertise<visualization_msgs::Marker>("visualization_marker_2", 1);
  uint32_t shape = visualization_msgs::Marker::CYLINDER; // Set shape of marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "simulator_origin";
  marker.header.stamp = ros::Time::now();
  marker.ns = "obstacle";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  //marker.pose.position.x = 10;
  //marker.pose.position.y = 10;
  //marker.pose.position.z = 5;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 4.0;
  marker.scale.y = 4.0;
  marker.scale.z = 10.0;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  
  // Set rate of loop
  ros::Rate loop_rate(ros_rate);

  while (ros::ok())
  {

    float m_soft = 1.5; // soft constraint allowed standard deviations
    float m_hard = 2.3263; // hard constraint allowed standard deviations

    // Generate soft scaling constraint
    Eigen::Matrix <float, 3, 2> A_soft;
    Eigen::Matrix <float, 3, 1> b_soft;
    int iter = 0;
    for(int i=0; i<4; i++){
      if(i != (uavNum-1)){
        auto const_ij = genConst(eta.block(1,0,2,1), Sigma[i].block(0,0,2,2), Sigma[uavNum-1].block(0,0,2,2), C.block(0,i,2,1), C.block(0,uavNum-1,2,1), m_soft);
        A_soft.row(iter) << get<0>(const_ij).transpose();
        b_soft(iter,0) = get<1>(const_ij);
        iter += 1;
      }
    }

    // Generate hard scaling constraint
    Eigen::Matrix <float, 3, 2> A_hard;
    Eigen::Matrix <float, 3, 1> b_hard;
    iter = 0;
    for(int i=0; i<4; i++){
      if(i != (uavNum-1)){
        auto const_ij = genConst(eta.block(1,0,2,1), Sigma[i].block(0,0,2,2), Sigma[uavNum-1].block(0,0,2,2), C.block(0,i,2,1), C.block(0,uavNum-1,2,1), m_hard);
        A_hard.row(iter) << get<0>(const_ij).transpose();
        b_hard(iter,0) = get<1>(const_ij);
        iter += 1;
      }
    }

    // Publish obstacle markers
    marker.pose.position.x = c_obst(0,0);
    marker.pose.position.y = c_obst(1,0);
    marker.pose.position.z = 5;
    marker_pub1.publish(marker);
    marker.pose.position.x = c_obst(0,1);
    marker.pose.position.y = c_obst(1,1);
    marker.pose.position.z = 5;
    marker_pub2.publish(marker);

    // Repulsive potential
    deta_rep = v2deta(repPot(p.block(0,uavNum-1,2,1)),eta);

    // Consensus step
    deta_N.setZero();
    for(int i=0; i<N-1; i++){
      deta_N = deta_N + 1*(eta_N.block(0,i,5,1) - eta);
    }

    // Perform soft projection step
    MatrixXf s_proj_soft = projScale(eta.block(1,0,2,1), A_soft, b_soft);
    MatrixXf deta_soft = Eigen::MatrixXf::Zero(5,1);
    if((!isnan(s_proj_soft.array())).all()){
      deta_soft.block(1,0,2,1) = 0.2*(s_proj_soft - eta.block(1,0,2,1));
    }

    // Calculate parameter derivative
    deta = deta_joy + 5*deta_N + deta_rep + deta_soft;

    // Perform hard projection step
    MatrixXf s_proj_hard = projScale(eta.block(1,0,2,1)+deta.block(1,0,2,1), A_hard, b_hard);
    if((!isnan(s_proj_hard.array())).all()){
      deta.block(1,0,2,1) = s_proj_hard - eta.block(1,0,2,1);
    }

    // Calculate reference position and velocity
    Eigen::MatrixXf vr = refVel(eta,deta);
    Eigen::MatrixXf pr = refPos(eta);

    // Update eta
    eta = eta + ts*deta;

    // Publish eta
    std_msgs::Float32MultiArray eta_msg;
    std::vector<float> data(eta.data(), eta.data() + eta.size());
    eta_msg.data = data;
    consensus_pub.publish(eta_msg);

    // Publish position reference
    mrs_msgs::ReferenceStamped pos_msg;
    pos_msg.reference.position.x = pr(0,0);
    pos_msg.reference.position.y = pr(1,0);
    pos_msg.reference.position.z = pr(2,0);
    pos_ref_pub.publish(pos_msg);

    // Publish velocity reference
    //mrs_msgs::VelocityReferenceStamped vel_msg;
    //vel_msg.reference.velocity.x = vr(0,0); 
    //vel_msg.reference.velocity.y = vr(1,0);
    //vel_msg.reference.velocity.z = vr(2,0);
    //vel_ref_pub.publish(vel_msg);

    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}
