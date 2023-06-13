#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <math.h>

#define NODE_NAME TFPractice


// bool PoseStampedMsgToTF(geometry_msgs::PoseStamped::ConstPtr pose, tf::Transform &ret){
//     ret.setOrigin(tf::Vector3(pose->pose.position.x,pose->pose.position.y,pose->pose.position.z));
//     ret.setRotation(tf::Quaternion(pose->pose.orientation.x,pose->pose.orientation.y,pose->pose.orientation.z,pose->pose.orientation.w));
//     return true;
// }
// bool TFToPoseStampedMsg(tf::Transform &transform, geometry_msgs::PoseStamped::Ptr ret){
//     tf::Vector3 tf_pos = transform.getOrigin();
//     tf::Quaternion tf_pose = transform.getRotation();
//     ret->pose.orientation.w = tf_pose.getW();
//     ret->pose.orientation.x = tf_pose.getX();
//     ret->pose.orientation.y = tf_pose.getY();
//     ret->pose.orientation.z = tf_pose.getZ();
//     ret->pose.position.x = tf_pos.getX();
//     ret->pose.position.y = tf_pos.getY();
//     ret->pose.position.z = tf_pos.getZ();
//     return true;
// }

std::string pretty(tf::StampedTransform &trans){
    double sec = trans.stamp_.toSec();
    std::string parent = trans.frame_id_;
    std::string child = trans.child_frame_id_;
    tf::Quaternion pose = trans.getRotation();
    double qw = pose.getW();
    double qx = pose.getX();
    double qy = pose.getY();
    double qz = pose.getZ();
    tf::Vector3 position = trans.getOrigin();
    double x = position.getX();
    double y = position.getY();
    double z = position.getZ();
    tf::Matrix3x3 mat = trans.getBasis();
    tf::Vector3 row0 = mat[0];
    tf::Vector3 row1 = mat[1];
    tf::Vector3 row2 = mat[2];
    double roll,pitch,yaw;
    mat.getRPY(roll,pitch,yaw);
    double rolld = roll / M_PI * 180.0;
    double pitchd = pitch / M_PI * 180.0;
    double yawd = yaw / M_PI * 180.0;

    std::stringstream ss;
    ss << std::endl;
    ss << "transform at : "<<sec<<std::endl;
    ss << "parent : ["<<parent<<"]" <<std::endl;
    ss << "child : ["<<child<<"]" <<std::endl;
    ss << "quat(w,x,y,z) : " << qw << ","<<qx<<","<<qy<<","<<qz<<std::endl;
    ss << "rpy : "<<roll<<","<<pitch<<","<<yaw<<std::endl;
    ss << "rpyd : "<<rolld<<","<<pitchd<<","<<yawd<<std::endl;
    ss << "transform ["<<child<<"]->["<<parent<<"] = Rx + t "<<std::endl;
    ss << "  R : "<<row0[0]<<","<<row0[1]<<","<<row0[2]<< std::endl;
    ss << "    : "<<row1[0]<<","<<row1[1]<<","<<row1[2]<< std::endl;
    ss << "    : "<<row2[0]<<","<<row2[1]<<","<<row2[2]<< std::endl;
    ss << "  t : "<<x<<std::endl;
    ss << "    : "<<y<<std::endl;
    ss << "    : "<<z<<std::endl;
    return ss.str();
}

class NODE_NAME{ 
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Timer evt_tf_broadcast;
        ros::Timer evt_tf_listen;
        ros::Timer evt_tf_internal;
        ros::Timer evt_tf_impl;
        ros::Publisher pub_cloud;

        tf::TransformListener tf_listener;
        tf::TransformBroadcaster tf_broadcaster;

    public:
        NODE_NAME();
        void spin();
        void evt_tf_broadcast_cb(const ros::TimerEvent &event);
        void evt_tf_listen_cb(const ros::TimerEvent &event);
        void evt_tf_internal_cb(const ros::TimerEvent &event);
        void evt_tf_impl_cb(const ros::TimerEvent &event);
};
NODE_NAME::NODE_NAME():
        nh_(""),
        pnh_("~")
{
    this->evt_tf_broadcast = pnh_.createTimer(ros::Duration(0.1),&NODE_NAME::evt_tf_broadcast_cb,this);
    this->evt_tf_listen = pnh_.createTimer(ros::Duration(0.1),&NODE_NAME::evt_tf_listen_cb,this);
    this->evt_tf_internal = pnh_.createTimer(ros::Duration(0.1),&NODE_NAME::evt_tf_internal_cb,this);
    this->evt_tf_impl = pnh_.createTimer(ros::Duration(0.1),&NODE_NAME::evt_tf_impl_cb,this);
    this->pub_cloud = pnh_.advertise<sensor_msgs::PointCloud2>("/cloud",1);
}

void NODE_NAME::evt_tf_broadcast_cb(const ros::TimerEvent &event){

    double roll_B = 0.1745329;
    double pitch_B = 0.0;
    double yaw_B = 2.2689281;
    double east_B = 5.0;
    double north_B = 0.0;
    double alt_B = 4.0;

    // create transform
    tf::Quaternion quat_B = tf::createQuaternionFromRPY(roll_B,pitch_B,yaw_B);
    tf::Vector3 vec_B = tf::Vector3(north_B,east_B,-alt_B);
    tf::Transform transform;
    transform.setRotation(quat_B);
    transform.setOrigin(vec_B);

    // broadcast transform ned -> B
    tf::StampedTransform stamped_transform = tf::StampedTransform(transform, ros::Time::now(), "ned", "B");
    this->tf_broadcaster.sendTransform(stamped_transform);
    // ROS_INFO("broadcast : %s",pretty(stamped_transform).c_str());
}

void NODE_NAME::evt_tf_listen_cb(const ros::TimerEvent &event){
    std::string target_frame = "ned";
    std::string source_frame = "A";
    ros::Time time = ros::Time::now();

    // subscribe transform
    tf::StampedTransform transform;
    try{
        // get transform from A to ned
        this->tf_listener.waitForTransform(target_frame, source_frame, time, ros::Duration(5.0));
        this->tf_listener.lookupTransform(target_frame, source_frame, time, transform);
        // ROS_INFO("listener result : %s",pretty(transform).c_str());
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    double z = 3.0;
    // transform pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i = 0; i < 100; i++){
        pcl::PointXYZ pt_source_i = pcl::PointXYZ(cosf(2*M_PI*i/100),sinf(2*M_PI*i/100),z);
        cloud_source->push_back(pt_source_i);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(*cloud_source, *cloud_target, transform);

    // transform Vector3
    tf::Vector3 pt_source = tf::Vector3(0,0,z);
    tf::Vector3 pt_target = transform * pt_source;
    pcl::PointXYZ pt_target_pcl = pcl::PointXYZ(pt_target.x(),pt_target.y(),pt_target.z());
    cloud_target->push_back(pt_target_pcl);

    // pcl to ROS msg
    sensor_msgs::PointCloud2 output_pc_msg;
    pcl::toROSMsg(*cloud_target, output_pc_msg);
    output_pc_msg.header.frame_id = target_frame;
    output_pc_msg.header.stamp = ros::Time::now();
    this->pub_cloud.publish(output_pc_msg);
}


void NODE_NAME::evt_tf_internal_cb(const ros::TimerEvent &event){
    tf::Transformer tf_transformer = tf::Transformer();

    // create transfrom 1
    double roll_C_1 = 0.0;
    double pitch_C_1 = 0.0;
    double yaw_C_1 = 0.0;
    double east_C_1 = 0.0;
    double north_C_1 = 0.0;
    double alt_C_1 = 0.0;
    tf::Quaternion quat_C_1 = tf::createQuaternionFromRPY(roll_C_1,pitch_C_1,yaw_C_1);
    tf::Vector3 vec_C_1 = tf::Vector3(north_C_1,east_C_1,-alt_C_1);
    tf::StampedTransform tf_C_1 = tf::StampedTransform(tf::Transform(quat_C_1,vec_C_1),ros::Time(1),"ned","C");
    tf_transformer.setTransform(tf_C_1);

    // create transfrom 2
    double roll_C_2 = 0.0;
    double pitch_C_2 = 0.0; 
    double yaw_C_2 =  1.57;
    double east_C_2 = 0.0;
    double north_C_2 = -5.0;
    double alt_C_2 = 0.0;
    tf::Quaternion quat_C_2 = tf::createQuaternionFromRPY(roll_C_2,pitch_C_2,yaw_C_2);
    tf::Vector3 vec_C_2 = tf::Vector3(north_C_2,east_C_2,-alt_C_2);
    tf::StampedTransform tf_C_2 = tf::StampedTransform(tf::Transform(quat_C_2,vec_C_2),ros::Time(2),"ned","C");
    tf_transformer.setTransform(tf_C_2);

    // get interpolated transform C -> ned
    double t = ros::Time::now().toSec();
    double t_interp = (t-int(t))+1;// 1~2
    tf::StampedTransform transform;
    tf_transformer.lookupTransform("ned","C",ros::Time(t_interp),transform);
    // ROS_INFO("internal interp : %s",pretty(transform).c_str());

    // broadcast
    transform.stamp_ = ros::Time::now();
    this->tf_broadcaster.sendTransform(transform);

}

void NODE_NAME::evt_tf_impl_cb(const ros::TimerEvent &event){
    // print transform
    tf::StampedTransform transform1;
    tf::StampedTransform transform2;
    this->tf_listener.lookupTransform("ned", "D",  ros::Time(0), transform1);
    this->tf_listener.lookupTransform("D", "ned",  ros::Time(0), transform2);
    ROS_INFO("n1 %s",pretty(transform1).c_str());
    ROS_INFO("n2 %s",pretty(transform2).c_str());
}

void NODE_NAME::spin(){
    ros::spin();
}

int main(int argc, char** argv)
{
    std::string node_name = "TFPractice";
    ros::init(argc, argv, node_name);
    NODE_NAME node;
    node.spin();
    return 0;
}