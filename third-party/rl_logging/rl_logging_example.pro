TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += "__cplusplus=201703L"

include(rl_logging.pri)

SOURCES += \
    src/example_ros2.cpp \
    src/example_std_cout.cpp
