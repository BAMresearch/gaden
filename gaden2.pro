TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

#DEFINES += "__cplusplus=201703L"

CONFIG += link_pkgconfig
#PKGCONFIG += eigen3
PKGCONFIG += python3

INCLUDEPATH += \
    /usr/lib/gcc/x86_64-linux-gnu/10/include
#    /opt/ros/foxy/include \
#    $$PWD/../install/octree/include \
#    $$PWD/../install/OpenVDB/include \
#    $$PWD/../install/rl_logging/include \
#    /usr/lib/gcc/x86_64-linux-gnu/10/include

include(gaden2/gaden2.pri)
include(gaden2_eigen/gaden2_eigen.pri)
include(gaden2_rviz/gaden2_rviz.pri)
include(pygaden2/pygaden2.pri)
include(third-party/rl_logging/rl_logging.pri)

#include(gaden_common/gaden_common.pri)
#include(gaden_environment/gaden_environment.pri)
#include(gaden_filament_simulator/gaden_filament_simulator.pri)
#include(gaden_preprocessing/gaden_preprocessing.pri)
#include(olfaction_msgs/olfaction_msgs.pri)
