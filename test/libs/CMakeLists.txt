if (NOT TARGET Catch2)
    set(CATCH_BUILD_TESTING OFF CACHE BOOL "Don't build Catch2 tests")
    add_subdirectory(Catch2)
endif()