#include <rl_logging/numbered_instance.hpp>

#include <map>
#include <mutex>

namespace rl::logging {

namespace internal {

size_t getNextInstanceNumber(const std::string &instance_base_name)
{
    static std::map<std::string, size_t> instance_counters;
    static std::mutex instance_counters_mutex;

    std::lock_guard<std::mutex> map_lock(instance_counters_mutex);

    if (instance_counters.find(instance_base_name) == instance_counters.end())
        instance_counters.emplace(instance_base_name, 0);

    size_t &number_of_instances = instance_counters.at(instance_base_name);
    size_t instance_number = number_of_instances;
    ++number_of_instances;

    return instance_number;
}

std::string convertInstanceNumberToString(size_t instance_number, bool hide_zero)
{
    if (instance_number == 0 && hide_zero)
        return "";
    else
        return std::to_string(instance_number);
}

InstanceOptionsImpl::InstanceOptionsImpl()
    : log_construction(false)
    , log_destruction(false)
    , hide_zero(false)
{}

NamedInstance::NamedInstance(std::string instance_name,
                             Logger &parent_logger,
                             InstanceOptions options)
    : instance_name_(instance_name)
    , instance_log_destruction_(options.getOptions().log_destruction)
    , logger(parent_logger.getChild(instance_name_))
{
    if (options.getOptions().log_construction)
        logger.info("Constructing...");
}

NamedInstance::~NamedInstance()
{
    if (instance_log_destruction_)
        logger.info("Destructed.");
}

InstanceNumber::InstanceNumber(const std::string &instance_base_name)
    : instance_number_(getNextInstanceNumber(instance_base_name))
{}

} // namespace internal

InstanceOptions & InstanceOptions::logConstruction(bool value)
{
    options_.log_construction = value;
    return *this;
}

InstanceOptions & InstanceOptions::logDestruction(bool value)
{
    options_.log_destruction = value;
    return *this;
}

InstanceOptions & InstanceOptions::logConAndDestruction(bool value)
{
    return logConstruction(value).logDestruction(value);
}

InstanceOptions & InstanceOptions::hideZero(bool value)
{
    options_.hide_zero = value;
    return *this;
}

NumberedInstance::NumberedInstance(const std::string &instance_base_name,
                                   Logger &parent_logger,
                                   InstanceOptions options)
    : InstanceNumber(instance_base_name)
    , NamedInstance(instance_base_name +
                    internal::convertInstanceNumberToString(
                        getInstanceNumber(),
                        options.getOptions().hide_zero),
                    parent_logger)
{}

} // namespace rl::logging
