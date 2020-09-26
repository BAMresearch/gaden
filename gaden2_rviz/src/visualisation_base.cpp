#include <gaden2_rviz/visualisation_base.hpp>

#include <iostream>

namespace gaden2::rviz {

VisualisationBase::VisualisationBase(const std::string &node_name)
{
    if (!rclcpp::ok())
    {
        rclcpp::init(0, NULL);
    }

    node_ = std::make_shared<rclcpp::Node>(node_name);
    node_clock_ = node_->get_clock();

    thread_ros_spin_ = std::thread(&VisualisationBase::spinRosNode, this);

    std::cout << "VisualisationBase constructed. Node name " << node_name << std::endl;
}

VisualisationBase::~VisualisationBase()
{
    std::cout << "Destructing VisualisationBase" << std::endl;
    rclcpp::shutdown();
    thread_ros_spin_.join();
    std::cout << "Thread joined" << std::endl;
}

std::shared_ptr<rclcpp::Node> VisualisationBase::getNode()
{
    return node_;
}

void VisualisationBase::spinRosNode()
{
    std::cout << "Start spinning" << std::endl;
    rclcpp::spin(node_);
    std::cout << "End spinning" << std::endl;
}

} // namespace gaden2::rviz
