#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <xv_lidar_controller/getSurrealDriver.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

  GetSurrealDriver driver;

  unsigned int num_readings = 360;
  double laser_frequency = 5;
  double ranges[num_readings];
  double intensities[num_readings];

  driver.motorOn();

  while(n.ok()){

    int angle; double range; double intensity;
    bool gotData = false;

    while(driver.readNextValues(&angle, &range, &intensity)) {
      ranges[angle] = range;
      intensities[angle] = intensity;
      gotData = true;
    }

    if(gotData) {
 
      ros::Time scan_time = ros::Time::now();

      //populate the LaserScan message
      sensor_msgs::LaserScan scan;
      scan.header.stamp = scan_time;
      scan.header.frame_id = "laser_frame";
      scan.angle_min = -3.1416;
      scan.angle_max = 3.1416;
      scan.angle_increment = 6.2832 / num_readings;
      scan.time_increment = (1 / laser_frequency) / (num_readings);
      scan.range_min = 0.0;
      scan.range_max = 100.0;

      scan.ranges.resize(num_readings);
      scan.intensities.resize(num_readings);
      for(unsigned int i = 0; i < num_readings; ++i){
        scan.ranges[i] = ranges[i];
        scan.intensities[i] = intensities[i];
      }

      scan_pub.publish(scan);
    }
  }
  driver.motorOff();
}
