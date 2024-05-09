#include "rclcpp/rclcpp.hpp" // Ros2의 C++라이브러리를 사용하기 위한 헤더 파일을 포함합니다.
#include "std_msgs/msg/int32.hpp" // std_msgs 패키지의 int 메시지 타입을 사용하기 위한 헤더 파일을 사용합니다.
#include "rclcpp/time_source.hpp" // timer를 사용할 수 있는 헤더 파일을 가져온다.
#include <memory> // make_shared 함수를 사용하기 위한 헤더파일
#include <chrono> // 시간과 관련된 기능을 사용하기 위한 헤더파일
#include <functional> // 함수 객체를 만들고 바인딩하기 위한 헤더 파일
using namespace std::chrono_literals;

void pub_callback(rclcpp::Node::SharedPtr node, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub)
{ // callback 함수를 정의합니다.
    static int count = 0; // 정적 변수를 선언합니다.
    auto message = std_msgs::msg::Int32(); // int형 타입의 객체를 생성합니다.
    message.data = count++; // 메시지의 데이터를 생성합니다.
    RCLCPP_INFO(node->get_logger(), "number: %d", message.data); // RCLCPP_INFO를 사용하여 메시지를 출력합니다.
    pub->publish(message); // 메시지를 publish합니다
}

int main(int argc, char* argv[]) // main 함수
{
    rclcpp::init(argc, argv); // Ros2를 초기화 합니다.
    auto node = std::make_shared<rclcpp::Node>("node_pub1_2"); // 노드 객체를 생성합니다.
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // publish에 사용할 QoS 프로파일을 설정합니다.
    auto pub = node->create_publisher<std_msgs::msg::Int32 >("topic_psub1_2", qos_profile); // 발행자를 생성합니다.
    std::function<void()> fn = std::bind(pub_callback, node, pub); // publish함수를 bind하여 실행할 함수 객체를 생성합니다.
    auto timer = node->create_wall_timer(1s, fn); // 타이머를 생성합니다.
    rclcpp::spin(node); // 노드를 계속 실행시킵니다.
    rclcpp::shutdown(); // Ros2를 종료합니다.
    return 0;
}
