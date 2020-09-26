#ifndef RL_LOGGING_NUMBERED_INSTANCE_HPP_INCLUDED
#define RL_LOGGING_NUMBERED_INSTANCE_HPP_INCLUDED

#include <string>

#include "logging_interface.hpp"

namespace rl::logging {

namespace internal {

struct InstanceOptionsImpl
{
    InstanceOptionsImpl();

    bool log_construction;
    bool log_destruction;
    bool hide_zero;
};

} // namespace internal

class InstanceOptions
{
public:
    InstanceOptions & logConstruction(bool value = true);
    InstanceOptions & logDestruction(bool value = true);
    InstanceOptions & logConAndDestruction(bool value = true);
    InstanceOptions & hideZero(bool value = true);

    const internal::InstanceOptionsImpl & getOptions() const {
        return options_;
    }

private:
    internal::InstanceOptionsImpl options_;
};

namespace internal {

class NamedInstance
{
public:
    NamedInstance(std::string instance_name,
                  rl::Logger &parent_logger,
                  InstanceOptions options = InstanceOptions());

    virtual ~NamedInstance();

    const std::string & getInstanceName() const { return instance_name_; }

private:
    std::string instance_name_;
    bool instance_log_destruction_;

protected:
    ::rl::Logger logger;
};

class InstanceNumber
{
public:
    InstanceNumber(const std::string &instance_base_name);
    size_t getInstanceNumber() const { return instance_number_; }

private:
    size_t instance_number_;
};

} // namespace internal

class NumberedInstance : public internal::InstanceNumber,
                         public internal::NamedInstance
{
public:
    NumberedInstance(const std::string &instance_base_name,
                     rl::Logger &parent_logger,
                     InstanceOptions options = InstanceOptions());
};

} // namespace rl::logging

#endif // RL_LOGGING_NUMBERED_INSTANCE_HPP_INCLUDED
