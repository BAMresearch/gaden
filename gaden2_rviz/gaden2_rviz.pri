INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD

INCLUDEPATH += \
    /opt/ros/foxy/include

SOURCES += \
    $$PWD/src/environment_visualisation_plane.cpp \
    $$PWD/src/filament_visualisation.cpp \
    $$PWD/src/gas_source_visualisation.cpp \
    $$PWD/src/helpers/ros_type_conversions.cpp \
    $$PWD/src/visualisation_base.cpp \
    $$PWD/src/wind2d_visualisation.cpp

HEADERS += \
    $$PWD/include/gaden2_rviz/environment_visualisation_plane.hpp \
    $$PWD/include/gaden2_rviz/filament_visualisation.hpp \
    $$PWD/include/gaden2_rviz/gas_source_visualisation.hpp \
    $$PWD/include/gaden2_rviz/helpers/ros_type_conversions.hpp \
    $$PWD/include/gaden2_rviz/visualisation_base.hpp \
    $$PWD/include/gaden2_rviz/wind2d_visualisation.hpp
