# RobotLibraries Logging Interface: rl_logging

Unfortunately, C++ has no standard library for logging. Therefore, there are many logging libraries, such as [log4cxx](https://logging.apache.org/log4cxx), [spdlog](https://github.com/gabime/spdlog) or the logging library of [ROS2](https://index.ros.org/doc/ros2/).

This especially complicates library development because the developer does not know which logging library might be used in the host application. One way to overcome this limitation is to provide an interface in the library that allows the user to define her/his own logging functions. However, this would add a lot of extra code and limit readability.

rl_logging addresses this problem by providing a very thin generic logging interface that can be used with any logging library.

## Example of usage

```c++
#include <rl_logging/std_logging.hpp>

rl::Logger log = rl::logging::StdLogger::create("logger_name");
log.debug("Debug message");
log.error() << "Stream logging: " << 8;

rl::Logger log_child = log.getChild("child");
log_child.info("A child is born.");
```

## Interfaces

rl_logging is shipped with two example interfaces:

- `StdLogger` that uses `std::cout`
- `Ros2Logger` that uses the ROS2 logging backend

You can find the source code of these examples in the `src` folder.

## Implementing your own interface

Implementing an interface for the logging library of your choice is pretty simple:

1. Create the interface class that inherits from `public rl::logging::internal::LoggerInterface`
2. Implement the following methods:
      - getChild
      - debug
      - info
      - warn
      - error
      - fatal
3. A method that returns a `rl::Logger` object. The example classes use a static `create` method for this.

## Design details

The actual logger that should be used is a `rl::Logger` instance. This class holds a `std::shared_ptr` to the `rl::logging::internal::LoggerInterface` implementation and thus allows the user to simply copy it.

## Numbered instances

Sometimes you want to identify every instance of a class. For this purpose, the `NumberedInstance` class is provided in `rl_logging/numbered_instance.hpp`. Usage:

```c++
#include <rl_logging/numbered_instance.hpp>

class MyClass : public rl::logging::NumberedInstance
{
public:
    MyClass(rl::Logger &parent_logger)
        : NumberedInstance("MyClassName", parent_logger)
    {
        logger.info("Here I am.");
    }
};
```

As you can see, `NumberedInstance` has a member `logger`, which is a child of the provided `parent_logger`. You can query the instance number with `getInstanceNumber()` and its name with `getInstanceName()`, respectively.
