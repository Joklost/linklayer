add_subdirectory(libs)
add_executable(test_linklayer main.cpp test.cpp)

target_link_libraries(test_linklayer PUBLIC linklayer Catch2)
target_include_directories(test_linklayer PUBLIC ${PROJECT_SOURCE_DIR}/test)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gpslog.txt ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gpslog_rssi.txt ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)

add_test(NAME TestLinkAiders COMMAND test_linklayer)