#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <algorithm>
#include <geometry_msgs/Twist.h>

class LaserScan
{
private:
  ros::NodeHandle nh;
  ros::Subscriber scan_sub;
  ros::Publisher vel_pub;
  geometry_msgs::Twist vel;
  bool moving_forward;
  double distance_diff;
  double wide;
  bool stop;
  
public:
  LaserScan();
  void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
  
};

LaserScan::LaserScan()
{
  scan_sub = nh.subscribe("/scan", 1000, &LaserScan::scanCb, this);
  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  moving_forward = false;
  stop = true;
  distance_diff = 0.05;
  wide = 0.5;
}

void LaserScan::scanCb(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  int scan_size = scan_msg->ranges.size();
  int middle = scan_size / 2;
/*   std::vector<float> scan_msg_v;
  
  for (int i = 0; i < scan_size; i++)
  {
    scan_msg_v.push_back(scan_msg->ranges[i]);
  }
  std::vector<float>::iterator min = std::min_element(scan_msg_v.begin(), scan_msg_v.end());
  int distance = std::distance(scan_msg_v.begin(), min);
  std::cout<<"min element is "<<*min<<" at position "<<distance<<std::endl;
  int middle = scan_size/2;
  float position = 0.0625 * (distance - middle);
  std::cout<<"hexapod should turn "<<position<<"°"<<std::endl;
  if (std::abs(position) < 1 && moving_forward == false)
  {
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.angular.z = 0;
    vel_pub.publish(vel);
    ros::Duration(5).sleep();
    ROS_INFO("hexapod is directly facing to the wall now.");
    ros::Duration(1).sleep();
    moving_forward = true;
    
  }
  if(std::abs(position) >= 0.3 && moving_forward == false)
  {
    int sign = (position > 0 ? 1 : -1);
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.angular.z = sign * 0.05;
    vel_pub.publish(vel);
  }
  if(moving_forward == true)
  {
    if(*min > 0.5)
    {
      ROS_INFO("hexapod is moving forward to the wall now.");
      vel.linear.x = 0.05;
      vel.linear.y = 0;
      vel.angular.z = 0;
      vel_pub.publish(vel);
    }
    else
    {
      vel.linear.x = 0;
      vel.linear.y = 0;
      vel.angular.z = 0;
      vel_pub.publish(vel);
      ros::Duration(3).sleep();
      ROS_INFO("hexapod is on position, waiting for adjusting.");
    }
  }*/
 
  double wall_left_wide = 0.0;
  double wall_right_wide = 0.0;
  int errCnt = 0;
  for( int i = 1; i < middle; i++ )
  {
   double right_distance = scan_msg ->ranges[middle-i] * sin( M_PI / 2 - scan_msg->angle_increment * i );
   double left_distance = scan_msg ->ranges[middle+i] * sin( M_PI / 2 - scan_msg->angle_increment * i );
   ROS_INFO("left: %f, right: %f",  scan_msg ->ranges[middle-i] ,  scan_msg ->ranges[middle+i]);
   ROS_INFO("left_distance[%d]: %f, right_distance[%d]: %f, middle_distance: %f .", middle-i, left_distance, middle+i, right_distance, scan_msg ->ranges[middle]);
   
   
   if ( std::abs(left_distance - scan_msg ->ranges[middle]) > distance_diff || std::abs(right_distance - scan_msg ->ranges[middle]) > distance_diff || scan_msg ->ranges[middle] > (scan_msg->range_max - 1) || scan_msg ->ranges[middle-i] > (scan_msg->range_max - 1) || scan_msg ->ranges[middle+i] > (scan_msg->range_max - 1))
   {
     errCnt++;
     if(errCnt>3)
     {
       vel.linear.x = 0;
       vel.linear.y = 0;
       vel.angular.z = 0.1;
       vel_pub.publish(vel);
       break;
    }
   }
   else
   {
     wall_left_wide = sqrt( pow(scan_msg->ranges[middle], 2) + pow(scan_msg->ranges[middle-i], 2) - 2 * scan_msg->ranges[middle] * scan_msg->ranges[middle-i] * cos(scan_msg->angle_increment * i) );
     wall_right_wide = sqrt( pow(scan_msg->ranges[middle], 2) + pow(scan_msg->ranges[middle+i], 2) - 2 * scan_msg->ranges[middle] * scan_msg->ranges[middle+i] * cos(scan_msg->angle_increment * i) );
     ROS_INFO("wall_length: %f, %f", wall_left_wide, wall_right_wide);
     if (wall_left_wide > wide && wall_right_wide > wide) 
     {
       vel.linear.x = 0;
       vel.linear.y = 0;
       vel.angular.z = 0;
       vel_pub.publish(vel);
       ros::Duration(5).sleep();
       ROS_INFO("wall finded.");
//        if(stop)
//        {
// 	 stop = false;
// 	 break;
//       }
//        int min_distance_point;
//        double min_distance = scan_msg->ranges[middle];
//        for(int j = 1; j <= i; j++)
//        {
// 	 if(scan_msg->ranges[middle-j] < min_distance)
// 	 {
// 	   min_distance = scan_msg->ranges[middle-j];
// 	   min_distance_point = -j;
// 	}
// 	if(scan_msg->ranges[middle+j] < min_distance)
// 	 {
// 	   min_distance = scan_msg->ranges[middle+j];
// 	   min_distance_point = j;
// 	}
//       }
//       double adjust_position = scan_msg->angle_increment * min_distance_point;
//       ROS_INFO("min_distance: %f", min_distance);
//       ROS_INFO("min_distance_point: %d", min_distance_point);
//       ROS_INFO("adjust_position: %f", adjust_position);
       ros::shutdown();
    }
  }
  }
  
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "laser_scan");
  LaserScan laser_scan;
  ros::Rate loop_rate(50);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  return 0; 
}




