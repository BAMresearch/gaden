#include <gaden2/simulator.hpp>
#include <gaden2/sensors/open_path.hpp>

namespace gaden2::sensors {

OpenPath::OpenPath(std::shared_ptr<gaden2::Simulator> simulator)
    : NumberedInstance("OpenPathSensor", simulator->getLogger())
    , simulator_(simulator)
{
    logger.info() << simulator_->getName();
}

OpenPath::~OpenPath()
{
    logger.info() << "Destructing " << simulator_->getName();
}

} // namespace gaden2::sensors
