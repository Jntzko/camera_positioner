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
   tf::StampedTransform fingertip_tag_transform;

   // latest measured position of the camera
   tf::Transform fingertip_camera_transform;

   // for successful initialization the apriltag has to be detected _once_
   bool initialized;
   std::string frame;

public:
   CameraPositioner() : initialized(false)
   {
      ros::NodeHandle pnh("~");
      pnh.param<std::string>("finger", frame, "ff");
      frame = "/rh_" + frame + "tip";
      ros::NodeHandle node;
      getConstantTransforms();
      sub = node.subscribe("tag_detections", 1, &CameraPositioner::callback, this);
   }

   void getConstantTransforms(){
      while(ros::ok()){
         try {
            listener.waitForTransform(frame, "/tag_1", ros::Time(0), ros::Duration(5.0) );
            listener.lookupTransform(frame, "/tag_1", ros::Time(0), fingertip_tag_transform);
            break;
         }
         catch(...){}
         ROS_WARN_THROTTLE(10, "Waiting for %s->tag_1 transform",frame.c_str());
      }
   }

   void callback(const apriltags_ros::AprilTagDetectionArray& msg){
      // if we got a valid tag detection, update fingertip_camera_transform
      for (int i=0; i < msg.detections.size(); i++){
        if( msg.detections[i].id == 1){
           tf::Transform tag_transform;
           tf::poseMsgToTF(msg.detections[i].pose.pose, tag_transform);
           fingertip_camera_transform = fingertip_tag_transform * tag_transform.inverse();
           if(!initialized){
              ROS_INFO("camera positioner is running");
              initialized = true;
           }
        }
      }

      // if we measured the camera's position successfully, publish it
      if(initialized){
         br.sendTransform(tf::StampedTransform(fingertip_camera_transform, ros::Time::now(), frame, "/usb_cam"));
      }
   }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "camera_position_node");
  CameraPositioner cam_pos;
  ros::spin();
  return 0;
};
