# Find freedesktop.org's libevdev headers and library 
# This module defines the following variables:
#
#  EVDEV_FOUND		- true if libevdev header and library was found
#  EVDEV_INCLUDE_DIR	- include path for libevdev
#  EVDEV_LIBRARY	- library path for libevdev
#

find_package(PkgConfig)
pkg_check_modules(Libevdev libevdev>=1.0)

SET(Libevdev_SEARCH_PATHS
	/usr/local
	/usr
)

FIND_PATH(Libevdev_INCLUDE_DIR libevdev.h
	HINTS $ENV{EVDEVDIR}
	PATH_SUFFIXES include/libevdev-1.0/libevdev include/libevdev
	PATHS ${EVDEV_SEARCH_PATHS}
)

FIND_LIBRARY(Libevdev_LIBRARY
	NAMES libevdev.a evdev
	HINTS
	$ENV{EVDEVDIR}
	PATH_SUFFIXES ${PATH_SUFFIXES}
	PATHS ${EVDEV_SEARCH_PATHS}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Libevdev REQUIRED_VARS Libevdev_LIBRARY Libevdev_INCLUDE_DIR)

mark_as_advanced(Libevdev_INCLUDE_DIR)