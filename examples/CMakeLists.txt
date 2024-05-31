if(MSVC)
  set(WARNING_FLAGS /W4 /WX)
else()
  set(WARNING_FLAGS -Wall -Wextra -pedantic -Werror)
endif()

add_subdirectory(ceres_simple_bundle_adjuster)
add_subdirectory(gtsam_bal)
