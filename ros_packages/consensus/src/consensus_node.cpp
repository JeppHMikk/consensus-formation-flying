#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Dense>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/TrajectoryReference.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <std_srvs/Trigger.h>
#include <mrs_lib/transformer.h>
#include <geometry_msgs/TransformStamped.h>

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

#include <vector>
#include <array>

// custom helper functions from mrs library
#include <mrs_lib/transformer.h>
 
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
using namespace Eigen;

// PARAMETERS
int N; // Number of robots
float clearance; // Minimum clearance between robots
float clearance_obstacles; // Minimum clearance between robots and obstacles
float r; // Robot minimum containing sphere radius
float m_soft; // Soft constraint allowed standard deviations
float m_hard; // Hard constraint allowed standard deviations
int ros_rate; // ROS rate
float ts; // Eta parameter update step size 
float Kv; // = 0.
float lambda; // Consensus parameter
float v_max; // Maximum velocity
bool all_robots_positions_valid_ = false; // Boolean to check that all robot positions are valid and that the algorithm can initiate
bool control_allowed_ = false; // Boolean to activate algorithm
string uavName = std::string(std::getenv("UAV_NAME"));
int uavNum; // = std::stoi(std::getenv("UAV_NUM")); // UAV ID number
string _control_frame_; // The frame that the control is done in
string default_formation_shape = "grid"; // Default formation shape
string formation_shape = default_formation_shape; // Initiate formation shape to be default
bool change_formation_shape = false; // Bool for when robots have to change formation shape
float rho_act;
float pot_strength;

MatrixXf eta(5,1); // formation parameters (phi,sx,sy,tx,ty)
MatrixXf deta(5,1); // formation parameter derivative
MatrixXf deta_N(5,1); // consensus formation parameter derivative
MatrixXf deta_joy(5,1); // joypad parameter derivative
MatrixXf deta_rep(5,1); // repulsive potential parameter derivative
MatrixXd joy_val(6,1); // joypad values

int N_obst; // Number of obstacles
Eigen::MatrixXf c_obst; // Obstacle center position
Eigen::MatrixXf r_obst; // Obstacle radius

std::shared_ptr<mrs_lib::Transformer> transformer_;

// Service callback for activating robots
bool activationServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
  // service for activation of planning
  ROS_INFO("[FormationController]: Activation service called.");
  res.success = true;
  if (control_allowed_) {
    res.message = "Control was already allowed.";
  } else if (!all_robots_positions_valid_) {
    res.message = "Robots are not ready, control cannot be activated.";
    res.success = false;
  } else {
    control_allowed_ = true;
    res.message = "Control allowed.";
  }
  return res.success;
}

// Subscriber callback for getting joypad values
void joyCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  deta_joy << 0.2*(float(msg->data[4])/32767.0 - float(msg->data[5])/32767.0),
              0.8*float(msg->data[0])/32767.0,
              -0.8*float(msg->data[1])/32767.0,
              1.0*float(msg->data[2])/32767.0,
              -1.0*float(msg->data[3])/32767.0;
}

// Subscriber callback for getting formation shape
void formationShapeCallback(const std_msgs::String::ConstPtr& msg){
  if(formation_shape.compare(msg->data.c_str()) != 0){
    formation_shape = msg->data.c_str();
    change_formation_shape = true;
  }
}

// Function for generating formation shape
vector<Eigen::Matrix<float,2,1>> formationShapeGen(){
  vector<Eigen::Matrix<float,2,1>> C(N);
  if(formation_shape.compare("grid") == 0){
    cout << "grid formation made" << endl;
    float dm = 1;
    while(dm*dm < N){
      dm = dm + 1;
    }
    int iter = 0;
    for(float i=0; i<dm; i++){
      for(float j=0; j<dm; j++){
        C[iter](0,0) = i - (dm-1)/2;
        C[iter](1,0) = j - (dm-1)/2;
        iter = iter + 1;
        if(iter >= N){
          goto end_loops;
        }
      }
    }
  }
  else if(formation_shape.compare("line") == 0){
    cout << "line formation made" << endl;
    for(int i=0; i<N; i++){
      C[i](0,0) = float(i) - (float(N)-1)/2;
      C[i](1,0) = 0.0;
    }
  }
  else if(formation_shape.compare("circle") == 0){
    float r = N/(2*M_PI);
    for(int i=0; i<N; i++){
      float phi_i = float(i)*2*M_PI/N;
      C[i](0,0) = r*cos(phi_i);
      C[i](1,0) = r*sin(phi_i);
    }
  }
  else if(formation_shape.compare("triangle") == 0){
    int iter = 0;
    Eigen::MatrixXf C_cent = Eigen::MatrixXf::Zero(2,1);
    for(float i=0; i<N; i++){
      for(float j=0; j<=i; j++){
        C[iter](0,0) = j - i/2;
        C[iter](1,0) = -0.866*i;
        C_cent = C_cent + C[iter];
        iter = iter + 1;
        if(iter >= N){
          break;
        }
      }
      if(iter >= N){
        break;
      }
    }
    C_cent = C_cent/N;
    for(int i=0; i<N; i++){
      C[iter] = C[iter] - C_cent;
    }
  }
  else{
    throw std::invalid_argument("Invalid formation shape");
    formation_shape = default_formation_shape;
    C = formationShapeGen();
  }
  end_loops:
  return C;
}

// Subscriber callback for getting formation parameters
void etaCallback(Eigen::Matrix<float, 5, 1>* eta_ptr, const std_msgs::Float32MultiArray::ConstPtr& msg){
  for(int i = 0; i < 5; i++){
    (*eta_ptr)(i,0) = msg->data[i];
  }
}

// Subscriber callback for getting position estimates
void posCallback(Eigen::Matrix<float, 3, 1>* p_ptr, Eigen::Matrix<float, 3, 1>* v_ptr, Eigen::Matrix<float, 3, 3>* Sigma_ptr, string uavNm, const nav_msgs::Odometry::ConstPtr& msg) {

    // Generate transform from odometry frame to control frame
    geometry_msgs::TransformStamped tf_est;
    std::optional<geometry_msgs::TransformStamped> res = transformer_->getTransform(msg->header.frame_id, uavNm + "/" + _control_frame_);
    if(res){
      tf_est = res.value();
    }

    // Load position in odometry frame into vector
    Eigen::Vector3d p_msg;
    p_msg << msg->pose.pose.position.x,
             msg->pose.pose.position.y,
             msg->pose.pose.position.z;

    // Transform position to control frame using transform
    auto res2 = transformer_->transformAsVector(p_msg, tf_est);
    Eigen::Vector3d p_msg_tf;
    if(res2){
      (*p_ptr) = res2.value().cast<float>();
    }

    // Extract rotation matrix from transform
    tf2::Transform tf_est_2;
    tf2::fromMsg(tf_est.transform, tf_est_2);
    tf2::Matrix3x3 rotation_matrix(tf_est_2.getRotation());
    Eigen::Matrix<float, 3, 3> R;
    for(int i=0; i<3; i++){
      for(int j=0; j<3; j++){
        R(i,j) = rotation_matrix[i][j];
      }
    }

    /*
    (*v_ptr) << msg->twist.twist.linear.x,
                msg->twist.twist.linear.y,
                msg->twist.twist.linear.z;
    */

    // Extract covariance from message and map into control frame using rotation matrix
    Eigen::Matrix <float, 6, 6> Sigma_msg;
    for(int i=0; i<6; i++){
        for(int j = 0; j<6; j++){
            Sigma_msg(i,j) = msg->pose.covariance[6*i+j];
        }
    }
    (*Sigma_ptr) << R.transpose()*Sigma_msg.block(0,0,3,3)*R;
    all_robots_positions_valid_ = true;
}

// Function for generation random float
float randomFloat(){
    return (float)(rand()) / (float)(rand());
}

// Function for calculating Jacobian 
Eigen::MatrixXf Jacobian(Eigen::MatrixXf eta_in, Eigen::MatrixXf C){
  Matrix <float, 2, 5> J;
  J << -sin(eta_in(0,0))*eta_in(1,0)*C(0,0) -cos(eta_in(0,0))*eta_in(2,0)*C(1,0), cos(eta_in(0,0))*C(0,0), -sin(eta_in(0,0))*C(1,0), 1, 0,
        cos(eta_in(0,0))*eta_in(1,0)*C(0,0) -sin(eta_in(0,0))*eta_in(2,0)*C(1,0), sin(eta_in(0,0))*C(0,0), cos(eta_in(0,0))*C(1,0), 0, 1;
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
Eigen::MatrixXf refPos(Eigen::MatrixXf eta_in, Eigen::MatrixXf C){
  Eigen::MatrixXf R = rotMat(eta_in);
  Eigen::MatrixXf S = scaleMat(eta_in);
  Eigen::MatrixXf t = transVec(eta_in);
  Eigen::Matrix <float, 3, 1> p;
  p << R*S*C + t, 1.3;
  return p;
}

// Reference velocity from parameter derivative
Eigen::MatrixXf refVel(Eigen::MatrixXf eta_in, Eigen::MatrixXf deta_in, Eigen::MatrixXf C){
  Eigen::Matrix <float, 3, 5> J;
  J << Jacobian(eta_in,C), 
       0, 0, 0, 0, 0;
  return Kv*J*deta_in;
}

// Repulsive potential for collision avoidance
Eigen::Matrix<float, 2, 1> repPot(Eigen::MatrixXf p_in, Eigen::MatrixXf Sigma, float m){
  Eigen::Matrix<float, 2, 1> v_rep;
  v_rep.setZero();
  float lambda_max = Sigma.eigenvalues().real().maxCoeff();
  float rho = (c_obst.col(0) - p_in.block(0,0,2,1)).norm() - r_obst(0,0) - clearance_obstacles;
  Eigen::MatrixXf drho = (p_in.block(0,0,2,1) - c_obst.col(0))/((p_in.block(0,0,2,1) - c_obst.col(0)).norm());
  for(int i=1; i<N_obst; i++){
    float rho_i = (c_obst.col(i) - p_in.block(0,0,2,1)).norm() - r_obst(i,0) - (r + m*sqrt(lambda_max) + clearance_obstacles);
    if(rho_i < rho){
      rho = rho_i;
      drho = (p_in.block(0,0,2,1) - c_obst.col(i))/((p_in.block(0,0,2,1) - c_obst.col(i)).norm());
    }
  }
  float df;
  if(rho >= rho_act){
    df = 0.0;
  }
  else{
    df = pot_strength*(1/rho - 1/rho_act)*1/(rho*rho);
  }
  v_rep = v_rep + df*drho;

  return v_rep;
}

// Function for mapping from velocity to parameter derivative
Eigen::MatrixXf v2deta(Eigen::MatrixXf v, Eigen::MatrixXf eta_in, Eigen::MatrixXf C){
  Eigen::MatrixXf J = Jacobian(eta_in,C);
  Eigen::MatrixXf J_inv = J.transpose()*(J*J.transpose()).inverse();
  Eigen::MatrixXf deta = J_inv*v;
  return deta;
}

/*
// CDF for normal distribution
float normalCDF(float x, float mu, float sigma){
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
*/

// Function for generating linear constraint for collision avoidance
tuple<Eigen::MatrixXf, float> genConst(Eigen::MatrixXf s, Eigen::MatrixXf Sigmai, Eigen:: MatrixXf Sigmaj, Eigen:: MatrixXf Ci, Eigen::MatrixXf Cj, float m, float Br){
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

// Function for calculating smallest element and index for vector 
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

// Function for projecting scaling variable onto constraint set using active-set optimizer
Eigen::MatrixXf projScale(Eigen::MatrixXf s, Eigen::MatrixXf A, Eigen::MatrixXf b){

  MatrixXf s0 = s;

  // Check if current scaling is feasible
  if(((A*s).array() >= b.array()).all()){
    return s;
  }
  else{
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

  // Initialize ROS node
  ros::init(argc, argv, "consensus_node_" + uavNum);
  ros::NodeHandle n;

  transformer_ = std::make_shared<mrs_lib::Transformer>(n, "uav" + to_string(uavNum) + "/Transformer");
  transformer_->retryLookupNewest(true);

  std::string _odom_topic;

  // Load ROS parameters
  ros::param::get("~N", N);
  ros::param::get("~clearance", clearance);
  ros::param::get("~clearance_obstacles",clearance_obstacles);
  ros::param::get("~r", r);
  ros::param::get("~m_soft", m_soft);
  ros::param::get("~m_hard", m_hard);
  ros::param::get("~ros_rate", ros_rate);
  ros::param::get("~ts", ts);
  ros::param::get("~Kv", Kv);
  ros::param::get("~lambda", lambda);
  ros::param::get("~control_frame", _control_frame_);
  ros::param::get("~v_max", v_max);
  ros::param::get("~rho_act", rho_act);
  ros::param::get("~pot_strength", pot_strength);
  ros::param::get("~odometry_topic", _odom_topic);

  std::vector<std::string> UAV_names;  
  ros::param::get("~UAV_names", UAV_names);

  for(int i=0; i<N ; i++){
    if(UAV_names[i].compare(uavName) == 0){
      uavNum = i + 1;
      break;
    }
  }

  std::vector<float> c_obst_x_param;
  ros::param::get("~c_obst_x", c_obst_x_param);
  std::vector<float> c_obst_y_param;
  ros::param::get("~c_obst_y", c_obst_y_param);
  std::vector<float> r_obst_param;
  ros::param::get("~r_obst", r_obst_param);

  N_obst = c_obst_x_param.size();

  c_obst.resize(2,N_obst);
  r_obst.resize(N_obst,1);

  for(int i=0; i<N_obst; i++){
    c_obst.col(i) << c_obst_x_param[i], c_obst_y_param[i];
    r_obst(i,0) = r_obst_param[i];
  }

  //uavName = UAV_names[uavNum-1];

  float Br = 2*r + clearance; // distance where collisions is assumed to happen

  // Initialize position estimate, position estimate covariance, and neighbour parameters
  vector<Matrix<float, 3, 1>> p(N); // position estimate
  vector<Matrix<float, 3, 1>> v(N);
  vector<Matrix<float, 3, 3>> Sigma(N); // position estimate covariance
  vector<Matrix<float, 5, 1>> eta_N(N);

  // Initialize base configuration
  vector<Matrix<float, 2, 1>> C = formationShapeGen();

  // Fill eta with random numbers
  /*
  srand(time(NULL));
  for(int i = 0; i < 5; i++){
    eta(i,0) = 5*randomFloat();
    if(i == 1 || i == 2)
      eta(i,0) += 4;
    //std::cout << eta(i,0) << std::endl;
  }
  */

  // Initialize eta
  eta << 0, 10, 10, 0, 0;
  for(int i=0; i<N; i++){
    eta_N[i] = eta;
  }
  
  // Initialize drone activation service
  ros::ServiceServer activation_service = n.advertiseService("/" + uavName + "/activate", activationServiceCallback);

  // Initialize publisher for consensus
  ros::Publisher consensus_pub = n.advertise<std_msgs::Float32MultiArray>("/" + uavName + "/eta", 1);

  // Initialize subscriber to joypad publisher
  ros::Subscriber joy_sub = n.subscribe("/joy_value", 1, joyCallback);

  // Initialize formation shape callback
  ros::Subscriber formation_shape_sub = n.subscribe("/formation_shape", 1, formationShapeCallback);

  // Initialize subscribers for consensus
  ros::Subscriber consensus[N];
  int iter = 0;
  for(int i=0; i<N; i++){
    if(i != uavNum-1){
      auto callback = [&eta_N, i](const std_msgs::Float32MultiArray::ConstPtr& msg) {
        etaCallback(&eta_N[i], msg);
      };
      consensus[i] = n.subscribe<std_msgs::Float32MultiArray>("/"+UAV_names[i]+"/eta", 1, callback);
    }
  }

  // Initialize subscribers for position estimates
  ros::Subscriber pos_sub[N];
  for (int i = 0; i < N; i++) {
    auto callback_pos = [&p, &v, &Sigma, UAV_names, i](const nav_msgs::Odometry::ConstPtr& msg) {
      posCallback(&p[i], &v[i], &Sigma[i], UAV_names[i], msg);
    };
    pos_sub[i] = n.subscribe<nav_msgs::Odometry>("/"+UAV_names[i]+"/"+_odom_topic, 1, callback_pos);
  }

  // Initialize velocity reference publisher
  ros::Publisher vel_ref_pub = n.advertise<mrs_msgs::VelocityReferenceStamped>("/" + uavName + "/control_manager/velocity_reference", 1);

  // Initialize position reference publisher
  ros::Publisher pos_ref_pub = n.advertise<mrs_msgs::ReferenceStamped>("/" + uavName + "/control_manager/reference", 1);

  // Initialize cylinder obstacle marker publisher
  ros::Publisher marker_pub[N_obst];
  for(int i=0; i<N_obst; i++){
    marker_pub[i] = n.advertise<visualization_msgs::Marker>("/visualization_marker_" + to_string(i+1), 1);
  }
  uint32_t shape = visualization_msgs::Marker::CYLINDER; // Set shape of marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "simulator_origin"; //UAV_names[0] + "/" + _control_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "obstacle";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 10.0;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  
  // Set rate of loop
  ros::Rate loop_rate(ros_rate);

  while (ros::ok()){

    //ROS_INFO("UAV_name=%s", UAV_names[uavNum-1]);

    // Publish obstacle markers
    for(int i=0; i<N_obst; i++){
      marker.scale.x = 2*r_obst(i,0);
      marker.scale.y = 2*r_obst(i,0);
      marker.pose.position.x = c_obst(0,i);
      marker.pose.position.y = c_obst(1,i);
      marker.pose.position.z = 5;
      marker_pub[i].publish(marker);
    }

    if(control_allowed_ & all_robots_positions_valid_){

      // If new formation shape is published, change formation shape
      if(change_formation_shape){
        C = formationShapeGen();
        change_formation_shape = false;
      }

      // Generate hard scaling constraint
      Eigen::Matrix <float, 3, 2> A_hard;
      Eigen::Matrix <float, 3, 1> b_hard;
      iter = 0;
      for(int i=0; i<N; i++){
        if(i != (uavNum-1)){
          auto const_ij = genConst(eta.block(1,0,2,1), Sigma[i].block(0,0,2,2), Sigma[uavNum-1].block(0,0,2,2), C[i], C[uavNum-1], m_hard, Br);
          A_hard.row(iter) << get<0>(const_ij).transpose();
          b_hard(iter,0) = get<1>(const_ij);
          iter += 1;
        }
      }

      // Repulsive potential
      Eigen::MatrixXf v_rep = repPot(refPos(eta,C[uavNum-1]),Sigma[uavNum-1].block(0,0,2,2),m_hard); 
      deta_rep = v2deta(v_rep,eta,C[uavNum-1]);

      // Consensus step
      deta_N.setZero();
      for(int i=0; i<N; i++){
        if(i != uavNum-1){
          deta_N = deta_N + lambda*(eta_N[i] - eta);
        }
      }

      // Calculate parameter derivative
      deta = deta_joy + deta_N + deta_rep; // + deta_soft;

      MatrixXf s_proj_hard = projScale(eta.block(1,0,2,1)+deta.block(1,0,2,1), A_hard, b_hard);
      if((!isnan(s_proj_hard.array())).all()){
        deta.block(1,0,2,1) = s_proj_hard - eta.block(1,0,2,1);
      }

      // Scale parameter derivative down to ensure that drones don't fly too fast
      Eigen::MatrixXf vr = refVel(eta,deta,C[uavNum-1]);
      if(vr.norm() > v_max){
        deta = v_max/vr.norm()*deta;
      }

      // Update eta
      eta = eta + ts*deta;

      // Moved projection step to after parameter update
      //MatrixXf s_proj_hard = projScale(eta.block(1,0,2,1), A_hard, b_hard);
      //if((!isnan(s_proj_hard.array())).all()){
      //  eta.block(1,0,2,1) = s_proj_hard;
      //}
      
      Eigen::MatrixXf pr = refPos(eta,C[uavNum-1]);

      mrs_msgs::ReferenceStamped pos_msg;
      pos_msg.header.frame_id = UAV_names[uavNum-1] + "/" + _control_frame_;
      pos_msg.reference.position.x = pr(0,0);
      pos_msg.reference.position.y = pr(1,0);
      pos_msg.reference.position.z = pr(2,0);
      pos_ref_pub.publish(pos_msg);
      
      // Publish eta
      std_msgs::Float32MultiArray eta_msg;
      std::vector<float> data(eta.data(), eta.data() + eta.size());
      eta_msg.data = data;
      consensus_pub.publish(eta_msg);

    }

    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}

