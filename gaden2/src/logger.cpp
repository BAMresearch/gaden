#include <gaden2/logger.hpp>

#include <rl_logging/std_logging.hpp>

namespace gaden2 {

rl::Logger getStandardLogger()
{
    rl::Logger logger = rl::logging::StdLogger::create("gaden2");
    return logger;
}

} // namespace gaden2
