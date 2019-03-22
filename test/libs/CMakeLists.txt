set(CATCH_BUILD_TESTING OFF CACHE BOOL "Don't build Catch2 tests")

if (NOT TARGET Catch2)
    add_subdirectory(Catch2)
endif()