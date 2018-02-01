#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include <geometry_msgs/Twist.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

#include <sstream>

// Respective to robot, not centroid
float x = 0;
float y = 0;
float z = 0;

// Are we moving?
int moving = false;

void centroid_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg) {
  // centroid: x=l/r, y=u/d, z:distance
  // Robot: x=foreward, y=l/r, z=u/d
  // centroid z = robot x, centroid y = robot y, centroid x = robot z

  pcl::PointXYZ centroid;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //(*cloud).push_back(pcl::PointXYZ (1,1,1));
  pcl:computeCentroid(*msg, centroid);

  x = centroid.z;
  y = centroid.y;
  z = centroid.x;
}

void killswitch_callback(const sensor_msgs::Joy::ConstPtr & msg) {
  if(msg->buttons[2] == 1) {
    moving = true;
  }
  else if(msg->buttons[1])
    moving = false;
}

int main(int argc, char **argv) {
  // Constants (magic numbers):
  float stop_distance = 1;
  float x_scale = 0.2;
  float z_scale = 0.2;
  float x_threshold = 0.003;
  float z_threshold = 0.003;

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "move_to_centroid");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber centroid_sub = n.subscribe("target_cluster", 1, centroid_callback);

  //rosrun joy y_node
  ros::Subscriber joystick_sub = n.subscribe("joy", 10, killswitch_callback);

  ros::Rate rate(10);
  while (ros::ok()) {
    if (moving) {
      // Get centroid we are navigating to
      geometry_msgs::Twist vel_msg;
      // Calculate V
      if(x != 0) {
        if(abs(x- stop_distance) * x_scale > x_threshold) {
          vel_msg.linear.x = (x - stop_distance) * x_scale;
        }
      }
      else {
        // stop
        vel_msg.linear.x = 0;
      }

      // Calculate omega (w)
      if(abs(z) * z_scale > z_threshold ) {
        vel_msg.angular.z = -z * z_scale;
      }
      else {
        // stop
        vel_msg.angular.z = 0;
      }

      // Publish Twist
      std::cout << "x: " << x << "\ny: " << y << "\nz: " << z << "\n";
      std::cout << "linear x: " << vel_msg.linear.x << "\nangular z: " << vel_msg.angular.z << "\n\n\n";
      cmd_vel_pub.publish(vel_msg);

    }
    // Else do nothing
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // (*cloud).push_back(pcl::PointXYZ (1,1,1));
    // (*cloud).push_back(pcl::PointXYZ (3,3,3));
    // test_pub.publish(*cloud);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
