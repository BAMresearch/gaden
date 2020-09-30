#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <gaden2/environment_model.hpp>
#include <gaden2/environment_model_plane.hpp>
#include <gaden2/filament_model.hpp>
#include <gaden2/gas_dispersion_model.hpp>
#include <gaden2/gas_source.hpp>
#include <gaden2/gas_source_filament_model.hpp>
#include <gaden2/gases.hpp>
#include <gaden2/simulator.hpp>
#include <gaden2/wind_model_farrell.hpp>
#include <gaden2/sensors/open_path.hpp>

#include <gaden2_rviz/environment_visualisation_plane.hpp>
#include <gaden2_rviz/gas_source_visualisation.hpp>
#include <gaden2_rviz/visualisation_base.hpp>

int add(int i, int j)
{
    return i+j;
}

PYBIND11_MODULE(pygaden2, m)
{
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("add", &add, "A function which adds two numbers");

    /** ========================= ENVIRONMENT MODELS ========================= **/

    pybind11::class_<gaden2::EnvironmentModel, std::shared_ptr<gaden2::EnvironmentModel>>(m, "EnvironmentModel");

    pybind11::class_<gaden2::EnvironmentModelPlane, gaden2::EnvironmentModel, std::shared_ptr<gaden2::EnvironmentModelPlane>>(m, "EnvironmentModelPlane")
            .def(pybind11::init<>());

    /** ========================= WIND MODELS ========================= **/

    pybind11::class_<gaden2::FarrellsWindModel, std::shared_ptr<gaden2::FarrellsWindModel>>(m, "FarrellsWindModel")
            .def(pybind11::init<
                    const std::shared_ptr<gaden2::EnvironmentModel> &,
                    double, // grid_cell_size
                    double, // u0
                    double, // v0
                    double, // kx
                    double, // ky
                    double, // noise_gain
                    double, // noise_damp
                    double  // noise_bandwidth
                 >(),
                 pybind11::arg("environment_model"),
                 pybind11::arg("grid_cell_size") = gaden2::FarrellsWindModel::DEFAULT_GRID_CELL_SIZE,
                 pybind11::arg("u0") = gaden2::FarrellsWindModel::DEFAULT_U0,
                 pybind11::arg("v0") = gaden2::FarrellsWindModel::DEFAULT_V0,
                 pybind11::arg("kx") = gaden2::FarrellsWindModel::DEFAULT_KX,
                 pybind11::arg("ky") = gaden2::FarrellsWindModel::DEFAULT_KY,
                 pybind11::arg("noise_gain") = gaden2::FarrellsWindModel::DEFAULT_NOISE_GAIN,
                 pybind11::arg("noise_damp") = gaden2::FarrellsWindModel::DEFAULT_NOISE_DAMP,
                 pybind11::arg("noise_bandwidth") = gaden2::FarrellsWindModel::DEFAULT_NOISE_BANDWIDTH);

    /** ========================= GASES ========================= **/

    pybind11::class_<gaden2::gases::GasBase, std::shared_ptr<gaden2::gases::GasBase>>(m, "GasBase");

    pybind11::class_<gaden2::gases::Air, gaden2::gases::GasBase, std::shared_ptr<gaden2::gases::Air>>(m, "Air")
            .def(pybind11::init<>());

    pybind11::class_<gaden2::gases::Methane, gaden2::gases::GasBase, std::shared_ptr<gaden2::gases::Methane>>(m, "Methane")
            .def(pybind11::init<>());

    /** ========================= GAS SOURCES ========================= **/

    pybind11::class_<gaden2::GasSource, std::shared_ptr<gaden2::GasSource>>(m, "GasSource");

    pybind11::class_<gaden2::GasSourceFilamentModel, gaden2::GasSource, std::shared_ptr<gaden2::GasSourceFilamentModel>>(m, "FilamentGasSource")
            .def(pybind11::init<
                    Eigen::Vector3d,   // position
                    std::shared_ptr<gaden2::gases::GasBase>, // gas
                    double,            // release_rate
                    unsigned,          // num_filaments_per_second
                    double             // mol_per_filament
                 >(),
                 pybind11::arg("position"),
                 pybind11::arg("gas"),
                 pybind11::arg("release_rate"),
                 pybind11::arg("num_filaments_per_second"),
                 pybind11::arg("mol_per_filament"));

    /** ========================= GAS DISPERSION MODELS ========================= **/

    pybind11::class_<gaden2::GasDispersionModel, std::shared_ptr<gaden2::GasDispersionModel>>(m, "GasDispersionModel");

    pybind11::class_<gaden2::FilamentGasModel, gaden2::GasDispersionModel, std::shared_ptr<gaden2::FilamentGasModel>>(m, "FilamentModel")
            .def(pybind11::init<
                    std::vector<std::shared_ptr<gaden2::GasSourceFilamentModel>>, // gas_sources
                    std::shared_ptr<gaden2::gases::GasBase> // environment_gas
                 >(),
                 pybind11::arg("gas_sources"),
                 pybind11::arg("environment_gas") = gaden2::FilamentGasModel::getDefaultEnvironmentGas());

    /** ========================= SIMULATOR ========================= **/

    pybind11::class_<gaden2::Simulator, std::shared_ptr<gaden2::Simulator>>(m, "Simulator")
            .def(pybind11::init<
                    std::shared_ptr<gaden2::GasDispersionModel>, // dispersion_model
                    double // dt
                 >(),
                 pybind11::arg("dispersion_model"),
                 pybind11::arg("dt") = gaden2::Simulator::DEFAULT_DT);

    /** ========================= SENSORS ========================= **/

    pybind11::class_<gaden2::sensors::OpenPath>(m, "OpenPathSensor")
            .def(pybind11::init<std::shared_ptr<gaden2::Simulator>>());

    /** ========================= VISUALISATION ========================= **/

    pybind11::class_<gaden2::rviz::VisualisationBase, std::shared_ptr<gaden2::rviz::VisualisationBase>>(m, "RvizVisualisationBase")
            .def(pybind11::init<const std::string &>());

    pybind11::class_<gaden2::rviz::EnvironmentVisualisationPlane>(m, "RvizEnvironmentVisualisationPlane")
            .def(pybind11::init<
                    std::shared_ptr<gaden2::rviz::VisualisationBase>,
                    std::shared_ptr<gaden2::EnvironmentModelPlane>,
                    //const std::string &,    // topic name
                    int,                    // publication_interval, [ms], special values: 0 = do not publish, -1 = publish once on creation
                    const std::string &,    // marker_namespace
                    int,                    // marker_id
                    const std::string &     // marker_frame_id
                 >(),
                 pybind11::arg("visualisation_base"),
                 pybind11::arg("environment_model"),
                 //pybind11::arg("topic_name") = gaden2::rviz::EnvironmentVisualisationPlane::DEFAULT_TOPIC_NAME,
                 pybind11::arg("publication_interval") = gaden2::rviz::EnvironmentVisualisationPlane::DEFAULT_PUBLICATION_INTERVAL,
                 pybind11::arg("marker_namespace") = gaden2::rviz::EnvironmentVisualisationPlane::DEFAULT_MARKER_NAMESPACE,
                 pybind11::arg("marker_id") = gaden2::rviz::EnvironmentVisualisationPlane::DEFAULT_MARKER_ID,
                 pybind11::arg("marker_frame_id") = gaden2::rviz::EnvironmentVisualisationPlane::DEFAULT_MARKER_FRAME_ID);

    pybind11::class_<gaden2::rviz::GasSourceVisualisation>(m, "RvizGasSourceVisualisation")
            .def(pybind11::init<
                    std::shared_ptr<gaden2::rviz::VisualisationBase>,
                    const std::vector<std::shared_ptr<gaden2::GasSource>> &,
                    int,                    // publication_interval, [ms], special values: 0 = do not publish, -1 = publish once on creation
                    const std::string &,    // marker_namespace
                    int,                    // marker_id
                    const std::string &     // marker_frame_id
                 >(),
                 pybind11::arg("visualisation_base"),
                 pybind11::arg("gas_sources"),
                 pybind11::arg("publication_interval") = gaden2::rviz::GasSourceVisualisation::DEFAULT_PUBLICATION_INTERVAL,
                 pybind11::arg("marker_namespace") = gaden2::rviz::GasSourceVisualisation::DEFAULT_MARKER_NAMESPACE,
                 pybind11::arg("marker_id") = gaden2::rviz::GasSourceVisualisation::DEFAULT_MARKER_ID,
                 pybind11::arg("marker_frame_id") = gaden2::rviz::GasSourceVisualisation::DEFAULT_MARKER_FRAME_ID);
}
