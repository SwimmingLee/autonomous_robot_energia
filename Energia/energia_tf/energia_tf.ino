#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

ros::NodeHandle nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char odom[] = "/odom";

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  broadcaster.init(nh);
}

void loop() {
  // put your main code here, to run repeatedly: 
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = 1.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.x = 0.0;
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
  nh.spinOnce();
  delay(10);
}
