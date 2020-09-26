#ifndef RL_LOGGING_LOGGING_INTERFACE_HPP_INCLUDED
#define RL_LOGGING_LOGGING_INTERFACE_HPP_INCLUDED

#include <functional>
#include <memory>
#include <sstream>
#include <string>

namespace rl {

namespace logging::internal {

class LoggerInterface
{
public:
    virtual ~LoggerInterface() {}

    virtual std::shared_ptr<LoggerInterface> getChild(const std::string &child_name) = 0;

    virtual void debug(const std::string &message) = 0;
    virtual void info(const std::string &message) = 0;
    virtual void warn(const std::string &message) = 0;
    virtual void error(const std::string &message) = 0;
    virtual void fatal(const std::string &message) = 0;
};

class StreamLogger
{
public:
    using LogFunction = std::function<void (const std::string &)>;
    StreamLogger(LogFunction log_function) : log_function_(log_function) {}

    ~StreamLogger()
    {
        log_function_(stream.str());
    }

    std::stringstream stream;

private:
    LogFunction log_function_;
};

template <typename T>
StreamLogger & operator <<(StreamLogger &logger, T &&entry)
{
    logger.stream << std::forward<T>(entry);
    return logger;
}

template <typename T>
StreamLogger & operator <<(StreamLogger &&logger, T &&entry)
{
    return logger << std::forward<T>(entry);
}

} // namespace logging::internal

class Logger
{
public:
    Logger(std::shared_ptr<rl::logging::internal::LoggerInterface> ptr)
        : impl_(ptr)
    {}

    Logger getChild(const std::string &child_name) {
        return impl_->getChild(child_name);
    }

    void debug(const std::string &message) { impl_->debug(message); }
    void info(const std::string &message) { impl_->info(message); }
    void warn(const std::string &message) { impl_->warn(message); }
    void error(const std::string &message) { impl_->error(message); }
    void fatal(const std::string &message) { impl_->fatal(message); }

    rl::logging::internal::StreamLogger debug() {
        return rl::logging::internal::StreamLogger([this](auto&& ...x) {
            impl_->debug(std::forward<decltype(x)>(x)...);
        });
    }

    rl::logging::internal::StreamLogger info() {
        return rl::logging::internal::StreamLogger([this](auto&& ...x) {
            impl_->info(std::forward<decltype(x)>(x)...);
        });
    }

    rl::logging::internal::StreamLogger warn() {
        return rl::logging::internal::StreamLogger([this](auto&& ...x) {
            impl_->warn(std::forward<decltype(x)>(x)...);
        });
    }

    rl::logging::internal::StreamLogger error() {
        return rl::logging::internal::StreamLogger([this](auto&& ...x) {
            impl_->error(std::forward<decltype(x)>(x)...);
        });
    }

    rl::logging::internal::StreamLogger fatal() {
        return rl::logging::internal::StreamLogger([this](auto&& ...x) {
            impl_->fatal(std::forward<decltype(x)>(x)...);
        });
    }

private:
    std::shared_ptr<rl::logging::internal::LoggerInterface> impl_;
};

} // namespace rl

#endif // RL_LOGGING_LOGGING_INTERFACE_HPP_INCLUDED
