if (NOT TARGET common)
    add_subdirectory(common)
endif()

if (NOT TARGET geo)
    add_subdirectory(geo)
endif()

if (NOT TARGET sims)
    add_subdirectory(sims)
endif()
