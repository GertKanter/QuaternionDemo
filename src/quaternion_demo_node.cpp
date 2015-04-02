#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

class QuaternionDemo
{
   private:
      ros::NodeHandle node_handle_;
      visualization_msgs::MarkerArray markers_;
      ros::Publisher markers_publisher_;
      ros::Publisher pose_publisher_;
   public:

   QuaternionDemo(ros::NodeHandle nh) : node_handle_(nh)
   {
      pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("quat_pose", 0, true);
      Eigen::Vector3d object = Eigen::Vector3d(0.5, 0.1, 0.6);
      Eigen::Vector3d manipulator_ee = Eigen::Vector3d(0.5, 0.1+0.2, 0.6+0.15);
      addMarker(object);
      addMarker(manipulator_ee);
      Eigen::Vector3d vector1 = object - manipulator_ee;
      Eigen::Vector3d vector2(0, 0, 1);
      Eigen::Vector3d rotation_vector = vectorCrossProduct(vector1, vector2);
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = "base_link";
      ps.pose.position.x = manipulator_ee[0];
      ps.pose.position.y = manipulator_ee[1];
      ps.pose.position.z = manipulator_ee[2];
      Eigen::Quaterniond q =  rotationVectorToQuaternion(-rotation_vector, angleBetweenTwoVectors(vector1, vector2)) * Eigen::Quaterniond(0.7071, 0, 0, 0.7071);
      ps.pose.orientation.w = q.w();
      ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y();
      ps.pose.orientation.z = q.z();
      pose_publisher_.publish(ps);
   }
      void addMarker(visualization_msgs::Marker marker)
      {
         this->markers_.markers.push_back(marker);
         this->markers_publisher_.publish(this->markers_);
      }

      void addMarker(Eigen::Vector3d point)
      {

         geometry_msgs::Pose pose;
         pose.position.x = point[0];
         pose.position.y = point[1];
         pose.position.z = point[2];
         pose.orientation.w = 1;
         pose.orientation.x = 0;
         pose.orientation.y = 0;
         pose.orientation.z = 0;
         geometry_msgs::Vector3 scale;
         scale.x = 0.03;
         scale.y = 0.03;
         scale.z = 0.03;

         std_msgs::ColorRGBA color;
         color.a = 1.0;
         color.r = 0.2;
         color.g = 0.2;
         color.b = 0.8;
         this->markers_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("quat_markers", 0, true);
         this->addMarker(pose, scale, color);
         ROS_ERROR("published marker");
      }

      double angleBetweenTwoVectors(const Eigen::Vector3d p1, const Eigen::Vector3d p2)
      {
         double angle = (p1[0]*p2[0] + p1[1]*p2[1] + p1[2]*p2[2]) / (sqrt(p1[0]*p1[0] + p1[1]*p1[1] + p1[2]*p1[2]) * sqrt(p2[0]*p2[0] + p2[1]*p2[1] + p2[2]*p2[2]));
         if (angle < -1) {
            angle = -1;
         }
         else if (angle > 1) {
            angle = 1;
         }
         ROS_INFO_STREAM("Angle: " << acos(angle));
         return acos(angle);
      }

      Eigen::Vector3d vectorCrossProduct(const Eigen::Vector3d p1, const Eigen::Vector3d p2)
      {
         Eigen::Vector3d vector;
         vector[0] = p1[1]*p2[2] - p2[1]*p1[2];
         vector[1] = p1[2]*p2[0] - p1[0]*p2[2];
         vector[2] = p1[0]*p2[1] - p2[0]*p1[1];
         //    x2y3 - x3y2, x3y1 - x1y3, x1y2 - x2y1
         return vector;
      }

      Eigen::Quaterniond rotationVectorToQuaternion(Eigen::Vector3d rotVec, double angle) {

         double magnitude = sqrt(rotVec[0]*rotVec[0] + rotVec[1]*rotVec[1] + rotVec[2]*rotVec[2]);

         double beta_x = rotVec[0] / magnitude;
         double beta_y = rotVec[1] / magnitude;
         double beta_z = rotVec[2] / magnitude;

         Eigen::Quaterniond q(cos(angle/2), sin(angle/2) * beta_x, sin(angle/2) * beta_y, sin(angle/2) * beta_z);
         return q;
      }

      void addMarker(geometry_msgs::Pose pose, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color)
      {
         visualization_msgs::Marker marker;
         marker.header.frame_id = "base_link";
         marker.header.stamp = ros::Time();
         marker.ns = "quat_demo";
         marker.id = markers_.markers.size();
         marker.type = visualization_msgs::Marker::SPHERE;
         marker.action = visualization_msgs::Marker::ADD;
         marker.pose = pose;
         marker.scale = scale;
         marker.color = color;
         this->addMarker(marker);
      }
};

int main(int argc, char** argv)
{
   ros::init(argc, argv, "quaternion_demo");
   ros::NodeHandle nh;
   QuaternionDemo demo(nh);
   ros::spin();
   return 0;
}
