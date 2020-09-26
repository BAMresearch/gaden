#ifndef RL_LOGGING_STD_LOGGING_HPP_INCLUDED
#define RL_LOGGING_STD_LOGGING_HPP_INCLUDED

#include <iostream>
#include <string>

#include "logging_interface.hpp"

namespace rl::logging {

class StdLogger : public rl::logging::internal::LoggerInterface
{
public:
    // std::make_shared cannot access the private constructor.
    // Therefore, a helper struct is used that makes the constructor accessible.

    static Logger create(const std::string &logger_name)
    {
        struct MakeConstructorPublic : public StdLogger {
            MakeConstructorPublic(const std::string &logger_name)
                : StdLogger(logger_name)
            {}
        };
        return Logger(std::make_shared<MakeConstructorPublic>(logger_name));
    }

    std::shared_ptr<LoggerInterface> getChild(const std::string &child_name)
    {
        struct MakeConstructorPublic : public StdLogger {
            MakeConstructorPublic(const std::string &logger_name)
                : StdLogger(logger_name)
            {}
        };
        return std::make_shared<MakeConstructorPublic>(logger_name_ + "." + child_name);
    }

    void debug(const std::string &message) { output("DEBUG", message); }
    void info( const std::string &message) { output("INFO", message); }
    void warn( const std::string &message) { output("WARN", message); }
    void error(const std::string &message) { output("ERROR", message); }
    void fatal(const std::string &message) { output("FATAL", message); }

private:
    StdLogger(const std::string &logger_name) : logger_name_(logger_name) {}

    void output(const char *severity, const std::string &message) {
        std::cout << "[" << severity << "] [" << logger_name_ << "]: " << message << std::endl;
    }

    std::string logger_name_;
};

} // namespace rl::logging

#endif // RL_LOGGING_STD_LOGGING_HPP_INCLUDED
