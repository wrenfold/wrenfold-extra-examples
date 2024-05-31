if(MSVC)
  set(WARNING_FLAGS /W4 /WX /wd4244 /wd4702)
else()
  set(WARNING_FLAGS -Wall -Wextra -pedantic -Werror -Wno-sign-compare
                    -Wno-unused-parameter)
endif()

add_subdirectory(ceres_simple_bundle_adjuster)
