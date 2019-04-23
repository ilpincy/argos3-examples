# Try to find GALIB
# See http://lancet.mit.edu/ga/
#
# Once run this will define: 
# 
# GALIB_FOUND        = system has GALIB lib
#
# GALIB_LIBRARIES    = full path to the libraries
# GALIB_INCLUDE_DIR  = where to find headers 

FIND_PATH(GALIB_INCLUDE_DIR
  NAMES
  ga.h
  ga/ga.h
  PATHS
  /usr/include
  /usr/local/include
  DOC "Google PerfTools header location"
)

#
# Find the library
#
FIND_LIBRARY(GALIB_LIBRARY
  NAMES
  ga
  PATHS
  /usr/lib
  /usr/local/lib
  DOC "Google PerfTools library location"
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GALIB DEFAULT_MSG
	GALIB_LIBRARY GALIB_INCLUDE_DIR)

IF(GALIB_INCLUDE_DIR AND GALIB_LIBRARY)
	SET(GALIB_INCLUDE_DIRS ${GALIB_INCLUDE_DIR})
	SET(GALIB_LIBRARIES	  ${GALIB_LIBRARY})
ENDIF(GALIB_INCLUDE_DIR AND GALIB_LIBRARY)

MARK_AS_ADVANCED(GALIB_INCLUDE_DIR)
MARK_AS_ADVANCED(GALIB_LIBRARY)
