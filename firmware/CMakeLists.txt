# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
# Shown by GET /version and tools/ota.py. Bump it when you want to tell builds apart by name;
# every build also gets a unique build id (hash of the firmware) regardless.
set(PROJECT_VER "1.1")
project(blink)
