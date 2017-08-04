#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

class CameraPositioner {
private:
   ros::Subscriber sub;

   // TF communication channels
   tf::TransformListener listener;
   tf::TransformBroadcaster br;

   // constant transforms
   tf::StampedTransform world_biotac_transform;
   tf::StampedTransform poc_world_tag_transform;
   tf::StampedTransform biotac_tag_transform;

   // latest measured position of the camera
   tf::Transform world_camera_transform;
   tf::Transform poc_world_camera_transform;
   ros::Time latest_detection_time;

   // for successful initialization the apriltag has to be detected _once_
   bool initialized;

public:
   CameraPositioner() : initialized(false)
   {
      ros::NodeHandle node;
      getConstantTransforms();
      sub = node.subscribe("tag_detections", 1, &CameraPositioner::callback, this);
   }

   void getConstantTransforms(){
      while(true){
         try {
            listener.waitForTransform("/poc_world", "/tag_1", ros::Time(0), ros::Duration(5.0) );
            listener.lookupTransform("/poc_world", "/tag_1", ros::Time(0), poc_world_tag_transform);
            break;
         }
         catch(...){}
         ROS_WARN_THROTTLE(10, "Waiting for poc_world->tag_1 transform");
      }
      while(true){
         try {
            listener.waitForTransform("/world", "/rh_ff_biotac_link", ros::Time(0), ros::Duration(5.0) );
            listener.lookupTransform("/world", "/rh_ff_biotac_link", ros::Time(0), world_biotac_transform);
            break;
         }
         catch(...){}
         ROS_WARN_THROTTLE(10, "Waiting for world->rh_ff_biotac_link transform");
      }
      while(true){
         try {
            listener.waitForTransform("/poc_ff_biotac_link", "/tag_1", ros::Time(0), ros::Duration(5.0) );
            listener.lookupTransform("/poc_ff_biotac_link", "/tag_1", ros::Time(0), biotac_tag_transform);
            break;
         }
         catch(...){}
         ROS_WARN_THROTTLE(10, "Waiting for poc_ff_biotac_link->tag_1 transform");
      }
   }

   void callback(const apriltags_ros::AprilTagDetectionArray& msg){
      // if we got a valid tag detection, update world_camera_transform
      for (int i=0; i < msg.detections.size(); i++){
        if( msg.detections[i].id == 1){
           tf::Transform tag_transform;
           tf::poseMsgToTF(msg.detections[i].pose.pose, tag_transform);
           world_camera_transform= world_biotac_transform * biotac_tag_transform * tag_transform.inverse();
           poc_world_camera_transform= poc_world_tag_transform * tag_transform.inverse();
           if(!initialized){
              ROS_INFO("camera positioner is running");
              initialized = true;
           }
           latest_detection_time = msg.detections[i].pose.header.stamp;
        }
      }

      // if we measured the camera's position successfully, publish it
      if(initialized){
         br.sendTransform(tf::StampedTransform(world_camera_transform.inverse(), ros::Time::now(), "/usb_cam", "/world"));
         br.sendTransform(tf::StampedTransform(poc_world_camera_transform.inverse(), ros::Time::now(), "/usb_cam", "/poc_world"));
      }
   }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "camera_position_node");
  CameraPositioner cam_pos;
  ros::spin();
  return 0;
};
