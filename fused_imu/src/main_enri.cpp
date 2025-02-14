
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using std::placeholders::_1;

class ImuFusion : public rclcpp::Node
{
public:
  ImuFusion()
  : Node("imu_fusion")
  {
    // subscription_ = this->create_subscription<std_msgs::msg::String>(
    //   "topic", 10, std::bind(&ImuFusion::topic_callback, this, _1));
    sub_imu1 = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, std::bind(&ImuFusion::imu1_callback, this, _1));
    sub_imu2 = this->create_subscription<sensor_msgs::msg::Imu>("/ouster/imu", 10, std::bind(&ImuFusion::imu2_callback, this, _1));
    publisher = this->create_publisher<double>("media_imu", 10);
  }

private:

  sensor_msgs::msg::Imu imu1;
  sensor_msgs::msg::Imu imu2;
  double media[2] = {0,0};
  bool f1, f2;
  rclcpp::Publisher<double>::SharedPtr publisher;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu1;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu2;


  void imu1_callback(const sensor_msgs::msg::Imu & msg)
  {
    imu1 = *msg;
    f1 = 1;
    if (f2) {
        this->media_kalman();
    }
  }
  void imu2_callback(const sensor_msgs::msg::Imu & msg)
  {
    imu2 = *msg;
    f2 = 1;
    if (f1) {
        this->media_kalman();
    }
  }

  double media_kalman(){
    double somma = imu1.linear_acceleration + imu2.linear_acceleration;
    
    if(f1 && f2){
        media[1] = (media[0]*9 + somma)/10;
        media[0] = media[1];
        f1 = 0;
        f2 = 0;
    }

    return media[1];
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuFusion>());
  rclcpp::shutdown();
  return 0;
}
