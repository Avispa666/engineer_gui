TEMPLATE = subdirs
SUBDIRS = robot_control \
    robot_gui
robot_gui.depends = robot_control
CONFIG += ordered c++1z
QMAKE_LFLAGS += -Wl,--no-allow-shlib-undefined,--no-undefined
QMAKE_LFLAGS_RELEASE += -static -static-libgcc -static-libc -static-libstdc++
QMAKE_CXXFLAGS_DEBUG += -Wall -Wextra
