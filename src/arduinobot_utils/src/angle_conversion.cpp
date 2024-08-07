#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arduinobot_msgs/srv/euler_to_quaternion.hpp>
#include <arduinobot_msgs/srv/quaternion_to_euler.hpp>
#include <tf2/utils.h>

using namespace std::placeholders;
class AnglesConverter : public rclcpp::Node
{
public:
    AnglesConverter() : Node("angles_conversion_service")
    {
        euler_to_quaternion_ = create_service<arduinobot_msgs::srv::EulerToQuaternion>("quaternion_to_euler", std::bind(&AnglesConverter::eulerToQuaternionCallback, this, _1,_2));
        quaternion_to_euler_ = create_service<arduinobot_msgs::srv::QuaternionToEuler>("euler_to_quaternion", std::bind(&AnglesConverter::quaternionToEulerCallback, this, _1,_2));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service add_two_ints is Ready");
    }

private:
    rclcpp::Service<arduinobot_msgs::srv::EulerToQuaternion>::SharedPtr euler_to_quaternion_;
    rclcpp::Service<arduinobot_msgs::srv::QuaternionToEuler>::SharedPtr quaternion_to_euler_;

    void eulerToQuaternionCallback(const std::shared_ptr<arduinobot_msgs::srv::EulerToQuaternion::Request> req,
                        const std::shared_ptr<arduinobot_msgs::srv::EulerToQuaternion::Response> res)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Request to convert euler angles roll: " <<req->roll << " pitch: " << req->pitch << " yaw: "<<req->yaw);
        tf2::Quaternion q;
        q.setRPY(req->roll,req->pitch,req->yaw);
        res->x = q.getX();
        res->y = q.getY();
        res->z = q.getZ();
        res->w = q.getW();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Corresponding quaternion x: " <<res->x << " y: " << res->y << " z: "<<res->z << " w: "<<res->w);
        

    }
    void quaternionToEulerCallback(const std::shared_ptr<arduinobot_msgs::srv::QuaternionToEuler::Request> req,
                        const std::shared_ptr<arduinobot_msgs::srv::QuaternionToEuler::Response> res)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Request to convert quaternion angles x: " <<req->x << " y: " << req->y << " z: "<<req->z << " w: "<<req->w);
        tf2::Quaternion q(req->x,req->y,req->z,req->w);
        tf2::Matrix3x3 m(q);
        m.getRPY(res->roll, res->pitch, res->yaw);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Corresponding euler angles roll: " <<res->roll << " pitch: " << res->pitch << " yaw: "<<res->yaw);
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnglesConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
