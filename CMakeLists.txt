cmake_minimum_required(VERSION 3.7.1)
set(CMAKE_CXX_STANDARD 14)

project(linklayer VERSION 1.0)

include(GNUInstallDirs)

find_package(Git QUIET)
if (GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    # Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if (GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if (NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif ()
    endif ()
endif ()

if (NOT EXISTS "${PROJECT_SOURCE_DIR}/libs/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif ()

add_subdirectory(libs)

set(HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/linklayer/linkmodel.h)
#set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--export-all-symbols")

add_library(linklayer SHARED
        $<TARGET_OBJECTS:common>
        $<TARGET_OBJECTS:geo>
        src/linkmodel.cpp
        src/gpslog.h src/gpslog.cpp
        src/model.h src/model.cpp
        src/node.h src/node.cpp
        src/link.h src/link.cpp
        src/action.h src/action.cpp)

set_target_properties(linklayer PROPERTIES PUBLIC_HEADER ${HEADER_FILES})

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(linklayer PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src test)

    #include_directories(include)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS linklayer EXPORT LinkLayerLibraryConfig
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}  # This is for Windows
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from install directory.
install(EXPORT LinkLayerLibraryConfig DESTINATION cmake)

# This makes the project importable from the build directory
export(TARGETS linklayer FILE LinkLayerLibraryConfig.cmake)

target_link_libraries(linklayer geo common)

enable_testing()
add_subdirectory(test)

