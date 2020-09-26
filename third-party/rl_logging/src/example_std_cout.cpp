#include <rl_logging/std_logging.hpp>

int main()
{
    rl::Logger log = rl::logging::StdLogger::create("rl_logging_example");

    log.info("rl_logging ROS2 example started");

    int x = 8;
    int y = 5;
    log.error() << "Stream logging: " << x << " != " << y;

    rl::Logger log_child = log.getChild("child");
    log_child.info("A child is born.");

    log.warn("rl_logging ROS2 example is going to end");

    return 0;
}
