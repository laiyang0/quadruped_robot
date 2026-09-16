#include "masscenter_des/masscenter_des.hpp"


MasscenterDes::MasscenterDes(): rclcpp::Node("masscenter_des_node")
{
    RCLCPP_INFO(this->get_logger(), "MasscenterDes node has been started.");
    keyboard_sub_=this->create_subscription<std_msgs::msg::String>("keyboard_input",10,std::bind(&MasscenterDes::sub_callback,this,std::placeholders::_1));
    path_pub_=this->create_publisher<nav_msgs::msg::Path>("masscenter_des_path",10);
    timer_=this->create_timer(std::chrono::milliseconds(long(t*1000)), std::bind(&MasscenterDes::timer_callback,this));
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "map";
    angle_w[2]=angle_z_init;    //当前b与o的初始偏航角
    pos_w[2]=pos_z_init;        //当前b与o的初始期望高度
}
MasscenterDes::~MasscenterDes()
{
    RCLCPP_INFO(this->get_logger(), "MasscenterDes node has been stopped.");
}
void MasscenterDes::timer_callback()
{   
    static int count=0;
    count++;
    if(count%250==0)
    {
        // RCLCPP_INFO(this->get_logger(), "vel in world frame: [%f, %f, %f]", vel_w[0], vel_w[1], vel_w[2]);
        // RCLCPP_INFO(this->get_logger(), "W in world frame: [%f, %f, %f]", w_w[0], w_w[1], w_w[2]);
        RCLCPP_INFO(this->get_logger(), "pos in world frame: [%f, %f, %f]", pos_w[0], pos_w[1], pos_w[2]);
        RCLCPP_INFO(this->get_logger(), "angle in world frame: [%f, %f, %f]", angle_w[0], angle_w[1], angle_w[2]);
    }
    /********计算质心轨迹*********/
    w_w=w_b;    //当前world系下的角速度
    angle_w[2]=angle_w[2]+w_w[2]*t;   //计算当前b与o的偏航角
    Eigen::Matrix3d R;
    R<<cos(angle_w[2]),-sin(angle_w[2]),0,
       sin(angle_w[2]),cos(angle_w[2]),0,
       0,0,1;    //旋转矩阵
    vel_w=R*vel_b;   //当前world系下的质心速度

    pos_w=pos_w+vel_w*t;   //当前world系下的质心位置
    angle_w=angle_w+w_w*t;   //当前world系下的质心角度
    /********计算期望质心轨迹*********/
    /********发布期望质心轨迹*********/
    Eigen::Quaterniond q(Eigen::AngleAxisd(angle_w[2], Eigen::Vector3d::UnitZ()));//将欧拉角转换为四元数
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = path_msg.header;
    pose_msg.pose.position.x = pos_w[0];
    pose_msg.pose.position.y = pos_w[1];
    pose_msg.pose.position.z = pos_w[2];
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    path_msg.poses.push_back(pose_msg);
    path_pub_->publish(path_msg);
    /********发布期望质心轨迹*********/

}
void MasscenterDes::sub_callback(std_msgs::msg::String::SharedPtr msg)
{
    switch(msg->data.c_str()[0])
    {
        case 'w':
            vel_b[0]+=0.1;
            // RCLCPP_INFO(this->get_logger(), "Moving forward");
            break;
        case 's':
            vel_b[0]-=0.1;
            // RCLCPP_INFO(this->get_logger(), "Moving backward");
            break;
        case 'a':
            vel_b[1]+=0.1;
            // RCLCPP_INFO(this->get_logger(), "Turning left");
            break;
        case 'd':
            vel_b[1]-=0.1;
            // RCLCPP_INFO(this->get_logger(), "Turning right");
            break;
        case 'q':
            angle_w[2]-=5;
            // RCLCPP_INFO(this->get_logger(), "Rotating left");
            break;
        case 'e':
            angle_w[2]+=5;
            // RCLCPP_INFO(this->get_logger(), "Rotating right");
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Invalid input");
    }
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto masscenter_des_node=std::make_shared<MasscenterDes>();
    rclcpp::spin(masscenter_des_node);
    rclcpp::shutdown();
    return 0;
}