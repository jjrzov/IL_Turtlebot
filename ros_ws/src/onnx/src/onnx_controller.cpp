#include <algorithm>
#include <string>
#include <memory>

#include "onnx/onnx_controller.hpp"

#include "nav2_core/exceptions.hpp"


class ONNXController : public rclcpp::Node {
    public:
        ONNXController() : Node("onnx_controller") {
            // ROS2 publishers
        }
};