# - Try to find libzmqpp
# Once done, this will define
#
#  ZEROMQPP_FOUND - system has libzmqpp
#  ZEROMQPP_INCLUDE_DIRS - the libzmqpp include directories
#  ZEROMQPP_LIBRARIES - link these to use libzmqpp

find_path(ZEROMQPP_ROOT_DIR
    NAMES include/zmqpp/zmqpp.hpp
)

find_library(ZEROMQPP_LIBRARIES
    NAMES zmqpp libzmqpp
    HINTS ${ZEROMQPP_ROOT_DIR}/lib
)

find_path(ZEROMQPP_INCLUDE_DIRS
    NAMES zmqpp/zmqpp.hpp
    HINTS ${ZEROMQPP_ROOT_DIR}/include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZEROMQPP DEFAULT_MSG
    ZEROMQPP_LIBRARIES
    ZEROMQPP_INCLUDE_DIRS
)

mark_as_advanced(
    ZEROMQPP_ROOT_DIR
    ZEROMQPP_LIBRARIES
    ZEROMQPP_INCLUDE_DIRS
)
