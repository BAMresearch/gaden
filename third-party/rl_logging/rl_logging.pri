INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD

HEADERS += \
    $$PWD/include/rl_logging/logging_interface.hpp \
    $$PWD/include/rl_logging/numbered_instance.hpp \
    $$PWD/include/rl_logging/ros2_logging.hpp \
    $$PWD/include/rl_logging/std_logging.hpp

SOURCES += \
    $$PWD/src/numbered_instance.cpp
