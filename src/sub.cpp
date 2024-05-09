#include "rclcpp/rclcpp.hpp" // Ros2의 C++라이브러리를 사용하기 위한 헤더 파일을 포함합니다.
#include "std_msgs/msg/int32.hpp" // std_msgs 패키지의 int형 메시지 타입을 사용하기 위한 헤더 파일을 사용합니다.
#include <memory> // make_shared 함수를 사용하기 위한 헤더파일
#include <functional> // 함수 객체를 만들고 바인딩하기 위한 헤더 파일

using std::placeholders::_1;
void sub_callback(rclcpp::Node::SharedPtr node, const std_msgs::msg::Int32::SharedPtr msg) // subsciber callback 함수를 정의
{
    RCLCPP_INFO(node->get_logger(), "node Received message: %d", msg->data); // publish한 메시지를 받아서 메시지로 출력
}

int main(int argc, char* argv[]) // main 함수
{
    rclcpp::init(argc, argv); // Ros2를 초기화
    auto node = std::make_shared<rclcpp::Node>("node_sub1_2"); // 노드 객체를 생성합니다.
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // subscribe에 사용할 QoS 프로파일을 설정합니다.
    std::function<void(const std_msgs::msg::Int32::SharedPtr msg)> fn; // 구독 콜백 함수를 저장하고 실행할 수 있는 함수 객체를 생성합니다.
    fn = std::bind(sub_callback, node, _1); // bind하여 실행할 함수 객체를 생성합니다.
    auto sub = node->create_subscription<std_msgs::msg::Int32>("topic_psub1_2",qos_profile,fn); // 구독자를 생성합니다.
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
