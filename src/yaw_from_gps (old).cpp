#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include "std_msgs/Int32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>

/* When this receives GPS values, it reads from the TF tree
 * the movement since the last GPS reading, and calculates
 * yaw from it. 
 */
 
int NUM_GPS_STORED = 8;

using std::cout;
using std::endl;
using std::string;
using std::vector;
using nav_msgs::Odometry;   
using geometry_msgs::Transform;                     
using geometry_msgs::TransformStamped;
using geometry_msgs::Quaternion;

double seconds_of_gps;
string odom_frame;                   //odom->base should be smooth
string base_frame;                   //centre of robot
vector<Odometry>  gps_vec;            //gps readings
vector<Transform> odom_vec;           //pose in odom frame 
tf2_ros::Buffer tfBuffer;            //for transformation lookup
ros::Subscriber sub_gps;
ros::Publisher  pub_gps_with_yaw;

/**************************************************
 * Helper functions
 **************************************************/

// Looks up a transform in the TF tree.
bool lookupTransform(TransformStamped &output, string frame1, string frame2, ros::Time t){
  try{
    cout << "Trying to get from frame: " << frame1 << " to " << frame2 << endl;                           
    if (!tfBuffer.canTransform(frame1, frame2, t) ){
      ROS_WARN("Unable to obtain a TF in yaw_from_gps.cpp");
      return false;
    }
    output = tfBuffer.lookupTransform(frame1, frame2, t);                                         
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return false;
  }
  return true;
}

/**************************************************
 * Subscriber functions
 **************************************************/

// Receives the GPS value.
void subscribeGPS(const Odometry::ConstPtr& msg) {
  Odometry gps = *msg;
  cout << "Received a GPS!" << endl;
  gps_vec.push_back(gps);
  // Grab the pose of the robot relative to the odom frame.
  TransformStamped ts;
  lookupTransform( ts, odom_frame, base_frame, ros::Time(0) );
  Transform odom = ts.transform;
  odom_vec.push_back(odom);
  
  // If the vector isn't filled, do nothing.
  if( gps_vec.size() < NUM_GPS_STORED ){
    return;
  }
  
  // Grab the previous GPS location and pose relative to odom.
  Odometry gps_prev   = gps_vec.front();
  Transform odom_prev = odom_vec.front();
  gps_vec.erase(gps_vec.begin());
  odom_vec.erase(odom_vec.begin());
  
  cout << "gps_prev: ( " << gps_prev.pose.pose.position.x << " , " <<
                            gps_prev.pose.pose.position.y << " )"  << endl;
  cout << "gps     : ( " << gps.pose.pose.position.x << " , " <<
                            gps.pose.pose.position.y << " )"  << endl;
  cout << "odom_prev: ( " << odom_prev.translation.x << " , " <<
                             odom_prev.translation.y << " )"  << endl;
  cout << "odom     : ( " << odom.translation.x << " , " <<
                             odom.translation.y << " )"  << endl;                          
                                                      
  // Calculate the dGPS and dOdom positions.
  double dgps_x,dgps_y,dodom_x,dodom_y,dgps_bearing,dodom_bearing,dbearing;
  dgps_x  = gps.pose.pose.position.x - gps_prev.pose.pose.position.x;
  dgps_y  = gps.pose.pose.position.y - gps_prev.pose.pose.position.y;
  dodom_x = odom.translation.x - odom_prev.translation.x;
  dodom_y = odom.translation.y - odom_prev.translation.y;
  dgps_bearing  = atan2(dgps_y, dgps_x);
  dodom_bearing = atan2(dodom_y, dodom_x);
   
  
  cout << "dgps bearing: " << dgps_bearing << endl;
  cout << "dodom_bearing: " << dodom_bearing << endl;
  
  // Calculate rotation to move odom bearing to GPS bearing.
  dbearing = dgps_bearing - dodom_bearing;
  
  // Rotate the odom yaw by the bearing change.
  tf2::Quaternion odom_q( odom.rotation.x, odom.rotation.y, 
                          odom.rotation.z, odom.rotation.w );
  tf2::Quaternion rot_q;
  rot_q.setRPY(0, 0, dbearing);
  tf2::Quaternion result_q = odom_q * rot_q;
  
  // Create the output message, the GPS with added yaw.
  Quaternion result_msg_q;
  result_msg_q.x = result_q.getX();
  result_msg_q.y = result_q.getY();
  result_msg_q.z = result_q.getZ();
  result_msg_q.w = result_q.getW();
  
  Odometry output = gps;
  output.pose.pose.orientation = result_msg_q;
  
  // The yaw covariance is inversely proportional to distance travelled.
  double cov_x = gps.pose.covariance[0];
  double cov_y = gps.pose.covariance[7];
  double cov_max = std::max(cov_x, cov_y);
  double dgps_dist = sqrt(dgps_x*dgps_x + dgps_y*dgps_y);
  double dist_stds = ( sqrt(cov_max) / dgps_dist ) / 1;  //aprox
  if( dgps_dist <= 0.0001 ){
    dist_stds = 1e5;
  }
  double dist_covs = dist_stds * dist_stds;
  output.pose.covariance[35] = dist_covs;
  
  cout << "std_max:   " << sqrt(cov_max) << endl;
  cout << "dgps_dist: " << dgps_dist << endl;
  cout << "dist_stds: " << dist_stds << endl;
  
  // Publish.
  pub_gps_with_yaw.publish(output);
  
}

/**************************************************
 * Initialising functions
 **************************************************/
// Read GPS.
void loadSubs(ros::NodeHandle n){
  sub_gps = n.subscribe("odometry/gps", 100, subscribeGPS);                                              
}

// Publish GPS with yaw.
void loadPubs(ros::NodeHandle n){
  pub_gps_with_yaw = n.advertise<Odometry>("odometry/gps_with_yaw", 10);                                              
}

// Loads parameters from the launch file. Uses default values if any values are
// missing.
void loadParams(ros::NodeHandle n_priv){
  // Set default parameters.
  string default_odom_frame = "odom";
  string default_base_frame = "base_link";
  double default_seconds_of_gps = 5;
  // Check parameter server to override defaults.
  n_priv.param( "odom_frame", odom_frame, default_odom_frame);
  n_priv.param( "base_frame", base_frame, default_base_frame);
  n_priv.param( "seconds_of_gps", seconds_of_gps, default_seconds_of_gps);
}


/**************************************************
 * Main
 **************************************************/
int main(int argc, char** argv){
  ros::init(argc, argv, "yaw_from_gps");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");
  tf2_ros::TransformListener tfListener(tfBuffer);   //must stay persistent
  cout << "Starting!" << endl;
  loadParams(n_priv);
  loadPubs(n);
  loadSubs(n);
  ros::spin();
  return 0;
};
