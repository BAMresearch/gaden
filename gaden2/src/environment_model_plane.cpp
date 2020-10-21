#include <gaden2/environment_model_plane.hpp>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <stdexcept>

namespace gaden2 {

static constexpr double z_min = -1.0;

EnvironmentModelPlane::EnvironmentModelPlane(double x_min,
                                             double x_max,
                                             double y_min,
                                             double y_max,
                                             double z_max,
                                             rl::Logger parent_logger)
    : EnvironmentModel(parent_logger)
    , world_min_(x_min, y_min, z_min)
    , world_max_(x_max, y_max, z_max)
    , plane_min_(world_min_)
    , plane_max_(x_max, y_max, 0)
{
    logger.info() << "Created plane environment model.";
}

EnvironmentModelPlane::EnvironmentModelPlane(const std::string &file,
                                             rl::Logger parent_logger)
    : EnvironmentModel(parent_logger)
{
    YAML::Node yaml_node = YAML::LoadFile(file);

    if (!yaml_node["Application"] || yaml_node["Application"].as<std::string>() != "GADEN2")
        throw std::runtime_error("Application in " + file + " invalid, should be: GADEN2");

    if (!yaml_node["Type"] || yaml_node["Type"].as<std::string>() != "EnvironmentModel")
        throw std::runtime_error("Type in " + file + " invalid, should be: EnvironmentModel");

    if (!yaml_node["Subtype"] || yaml_node["Subtype"].as<std::string>() != "Plane")
        throw std::runtime_error("Type in " + file + " invalid, should be: Plane");

    if (!yaml_node["EnvironmentModelPlane"] || !yaml_node["EnvironmentModelPlane"].IsMap())
        throw std::runtime_error("EnvironmentModelPlane in " + file + " not present or not a map.");

    YAML::Node env_node = yaml_node["EnvironmentModelPlane"];

    // TODO Put these in a helper function
    if (!env_node["WorldMin"] || !env_node["WorldMin"].IsSequence())
        throw std::runtime_error("WorldMin in " + file + " not present or not a sequence.");

    if (!env_node["WorldMax"] || !env_node["WorldMax"].IsSequence())
        throw std::runtime_error("WorldMax in " + file + " not present or not a sequence.");

    if (!env_node["PlaneMin"] || !env_node["PlaneMin"].IsSequence())
        throw std::runtime_error("PlaneMin in " + file + " not present or not a sequence.");

    if (!env_node["PlaneMax"] || !env_node["PlaneMax"].IsSequence())
        throw std::runtime_error("PlaneMax in " + file + " not present or not a sequence.");

    world_min_(0) = env_node["WorldMin"][0].as<double>();
    world_min_(1) = env_node["WorldMin"][1].as<double>();
    world_min_(2) = env_node["WorldMin"][2].as<double>();

    world_max_(0) = env_node["WorldMax"][0].as<double>();
    world_max_(1) = env_node["WorldMax"][1].as<double>();
    world_max_(2) = env_node["WorldMax"][2].as<double>();

    plane_min_(0) = env_node["PlaneMin"][0].as<double>();
    plane_min_(1) = env_node["PlaneMin"][1].as<double>();
    plane_min_(2) = env_node["PlaneMin"][2].as<double>();

    plane_max_(0) = env_node["PlaneMax"][0].as<double>();
    plane_max_(1) = env_node["PlaneMax"][1].as<double>();
    plane_max_(2) = env_node["PlaneMax"][2].as<double>();

    logger.info() << "WorldMin = [" << world_min_(0) << ", " << world_min_(1) << ", " << world_min_(2) << "]"
                  << " WorldMax = [" << world_max_(0) << ", " << world_max_(1) << ", " << world_max_(2) << "]"
                  << " PlaneMin = [" << plane_min_(0) << ", " << plane_min_(1) << ", " << plane_min_(2) << "]"
                  << " PlaneMax = [" << plane_max_(0) << ", " << plane_max_(1) << ", " << plane_max_(2) << "]";
}

EnvironmentModelPlane::~EnvironmentModelPlane()
{}

void EnvironmentModelPlane::startRecord(const std::string &file)
{
    YAML::Node yaml_node;
    yaml_node["Application"] = "GADEN2";
    yaml_node["Type"] = "EnvironmentModel";
    yaml_node["Subtype"] = "Plane";

    //yaml_node["EnvironmentModelPlane"] = YAML::Node();
    yaml_node["EnvironmentModelPlane"]["WorldMin"].push_back(world_min_(0));
    yaml_node["EnvironmentModelPlane"]["WorldMin"].push_back(world_min_(1));
    yaml_node["EnvironmentModelPlane"]["WorldMin"].push_back(world_min_(2));

    yaml_node["EnvironmentModelPlane"]["WorldMax"].push_back(world_max_(0));
    yaml_node["EnvironmentModelPlane"]["WorldMax"].push_back(world_max_(1));
    yaml_node["EnvironmentModelPlane"]["WorldMax"].push_back(world_max_(2));

    yaml_node["EnvironmentModelPlane"]["PlaneMin"].push_back(plane_min_(0));
    yaml_node["EnvironmentModelPlane"]["PlaneMin"].push_back(plane_min_(1));
    yaml_node["EnvironmentModelPlane"]["PlaneMin"].push_back(plane_min_(2));

    yaml_node["EnvironmentModelPlane"]["PlaneMax"].push_back(plane_max_(0));
    yaml_node["EnvironmentModelPlane"]["PlaneMax"].push_back(plane_max_(1));
    yaml_node["EnvironmentModelPlane"]["PlaneMax"].push_back(plane_max_(2));

    std::ofstream fstream(file);
    fstream << yaml_node;
}

void EnvironmentModelPlane::stopRecord()
{
    // do nothing
}

Eigen::Vector3d EnvironmentModelPlane::getEnvironmentMin() const
{
    return world_min_;
}

Eigen::Vector3d EnvironmentModelPlane::getEnvironmentMax() const
{
    return world_max_;
}

Occupancy EnvironmentModelPlane::getOccupancy(const Eigen::Vector3d &p) const
{
    if ((p.array() < world_min_.array()).any() || (p.array() > world_max_.array()).any())
        return Occupancy::OutOfWorld;
    else if ((p.array() >= plane_min_.array()).all() && (p.array() <= plane_max_.array()).all())
        return Occupancy::Occupied;
    else
        return Occupancy::Free;
}

Eigen::Vector3d EnvironmentModelPlane::getPlaneCenterCoordinates() const
{
    return 0.5 * (plane_min_ + plane_max_);
}

Eigen::Vector3d EnvironmentModelPlane::getPlaneDimensions() const
{
    return plane_max_ - plane_min_;
}

} // namespace gaden2
