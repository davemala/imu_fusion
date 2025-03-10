#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/qos.hpp>  // Per il QoS
#include <

using std::placeholders::_1;

class ImuFusion : public rclcpp::Node
{
public:
  ImuFusion()
  : Node("imu_fusion"), f1(false), f2(false)
  {

    // Creazione di un profilo QoS con Best Effort
    rclcpp::QoS qos_profile = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // Modifica delle sottoscrizioni per usare Best Effort
    sub_imu1_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", qos_profile, std::bind(&ImuFusion::imu1_callback, this, _1));

    sub_imu2_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/ouster/imu", qos_profile, std::bind(&ImuFusion::imu2_callback, this, _1));
    
    // Publisher per il nuovo IMU fuso
    pub_fused_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/fused", 10);
  }

private:
  sensor_msgs::msg::Imu imu1_;
  sensor_msgs::msg::Imu imu2_;
  bool f1, f2;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu1_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu2_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_fused_imu_;

  void imu1_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu1_ = *msg;
    f1 = true;
    if (f2) {
      fuse_and_publish();
    }
  }

  void imu2_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu2_ = *msg;
    f2 = true;
    if (f1) {
      fuse_and_publish();
    }
  }

  void fuse_and_publish()
  {
    // Definizione dei pesi (modifica se necessario)
    double w1 = 0.7;
    double w2 = 0.3;
    double sum_w = w1 + w2;

    // Creazione del nuovo messaggio IMU fuso
    sensor_msgs::msg::Imu fused_imu;
    fused_imu.header.stamp = imu2_.header.stamp;
    fused_imu.header.frame_id = "imu_fused";

    // **1. Media pesata dell'orientazione (quaternioni)**
    fused_imu.orientation.x = (w1 * imu1_.orientation.x + w2 * imu2_.orientation.x) / sum_w;
    fused_imu.orientation.y = (w1 * imu1_.orientation.y + w2 * imu2_.orientation.y) / sum_w;
    fused_imu.orientation.z = (w1 * imu1_.orientation.z + w2 * imu2_.orientation.z) / sum_w;
    fused_imu.orientation.w = (w1 * imu1_.orientation.w + w2 * imu2_.orientation.w) / sum_w;

    // **2. Media pesata della velocità angolare**
    fused_imu.angular_velocity.x = (w1 * imu1_.angular_velocity.x + w2 * imu2_.angular_velocity.x) / sum_w;
    fused_imu.angular_velocity.y = (w1 * imu1_.angular_velocity.y + w2 * imu2_.angular_velocity.y) / sum_w;
    fused_imu.angular_velocity.z = (w1 * imu1_.angular_velocity.z + w2 * imu2_.angular_velocity.z) / sum_w;

    // **3. Media pesata dell'accelerazione lineare**
    fused_imu.linear_acceleration.x = (w1 * imu1_.linear_acceleration.x + w2 * imu2_.linear_acceleration.x) / sum_w;
    fused_imu.linear_acceleration.y = (w1 * imu1_.linear_acceleration.y + w2 * imu2_.linear_acceleration.y) / sum_w;
    fused_imu.linear_acceleration.z = (w1 * imu1_.linear_acceleration.z + w2 * imu2_.linear_acceleration.z) / sum_w;

    // Pubblicazione del messaggio fuso
    pub_fused_imu_->publish(fused_imu);
    RCLCPP_INFO(this->get_logger(), "Published fused IMU data");

    // Reset flag
    f1 = false;
    f2 = false;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuFusion>());
  rclcpp::shutdown();
  return 0;
}
