#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <complex>

const float pickupZoneX = 3.84;
const float pickupZoneY = -6.88;
const float pickupZoneZ = 1.0;

const float  dropOffZoneX = 7.69;
const float  dropOffZoneY = 3.8;
const float  dropOffZoneZ = 1.0;


bool isPickupZone = false;

visualization_msgs::Marker marker;
ros::Publisher marker_pub;
ros::Subscriber odom_sub;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  float currentPoseX = msg -> pose.pose.position.x;
  float currentPoseY = msg -> pose.pose.position.y;

  float oclidDistance = 0.0;
  if (isPickupZone) {
	oclidDistance = sqrt(pow(currentPoseX - dropOffZoneX, 2) + pow(currentPoseY - dropOffZoneY, 2));	
  }else {
        oclidDistance = sqrt(pow(currentPoseX - pickupZoneX, 2) + pow(currentPoseY - pickupZoneY, 2));
  }

  if (oclidDistance < 0.3f) {
        // Pick up zone reached
	if (!isPickupZone) {
		isPickupZone = true;
		deleteMarker();
		marker_pub.publish(marker);
		ros::Duration(5).sleep();
		updateMaker();
		marker_pub.publish(marker);
        } else {
		deleteMarker();	
		marker_pub.publish(marker);
        }
  }
}

static void deleteMarker(){
	marker.action = visualization_msgs::Marker::DELETE;
}

static void updateMaker(){
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = dropOffZoneX;
	marker.pose.position.y = dropOffZoneY;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
}

static void createMarker(const uint8_t action)
{
	// Set the marker type.  can be a CUBE, SPHERE, ARROW, or CYLINDER
	marker.type = visualization_msgs::Marker::CUBE;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "add_markers";
	marker.id = 0;

	marker.type = shape;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = action;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = pickupZoneX;
	marker.pose.position.y = pickupZoneY;
	marker.pose.position.z = 0.0;

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0f;

	marker.lifetime = ros::Duration();
}


int main( int argc, char** argv )
{
ros::init(argc, argv, "add_markers");
	ros::NodeHandle n;
	ros::Rate r(1);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	odom_sub = n.subscribe("/odom", 10, &odom_callback);

	createMarker(visualization_msgs::Marker::ADD);
	
        // Check for subscribers
	while(marker_pub.getNumSubscribers()<1){
		ROS_WARN("Please subscribe to the marker");
		if(!ros::ok()){
			return 0;
		}
	}

	marker.action=visualization_msgs::Marker::ADD;
	marker_pub.publish(marker);
	ROS_INFO("Published marker at the pick_up point");
	ros::Rate r(1);
	bool picked=false;
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
	ROS_INFO("\n not shut down:%d",ros::ok());
		
}

