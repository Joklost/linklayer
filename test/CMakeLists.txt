add_subdirectory(libs)
add_executable(test_lm main.cpp test.cpp)

target_link_libraries(test_lm PUBLIC lm Catch2)
target_include_directories(test_lm PUBLIC ${PROJECT_SOURCE_DIR}/test)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/gpslog.txt ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)

add_test(NAME TestLinkAiders COMMAND test_lm)