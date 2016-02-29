# Copyright (c) 2015 CNRS
# Author: Anna Seppala
#
# This file is part of hpp-dlr-ipa.
# hpp-dlr-ipa is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-dlr-ipa is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-dlr-ipa.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp-ompl-benchmark)
SET(PROJECT_DESCRIPTION "Benchmark data needed to compare HPP and OMPL")
SET(PROJECT_URL "")

SETUP_PROJECT()

# ADD_REQUIRED_DEPENDENCY (ur_description)
ADD_REQUIRED_DEPENDENCY (xacro)

INSTALL(FILES
  urdf/cubicles_env.urdf
  urdf/cubicles_robot.urdf
  urdf/Abstract_env.urdf
  urdf/Abstract_robot.urdf
  urdf/pipedream_env.urdf
  urdf/ring.urdf
DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/hpp-ompl-benchmark/urdf
)
INSTALL(FILES
  srdf/cubicles_env.srdf
  srdf/cubicles_robot.srdf
  srdf/Abstract_env.srdf
  srdf/Abstract_robot.srdf
  srdf/pipedream_env.srdf
  srdf/ring.srdf
 DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/hpp-ompl-benchmark/srdf
)

INSTALL (FILES
  package.xml
 DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/hpp-ompl-benchmark
)

# Copy mesh files to ROS package
SET (MESHES
  mesh/cubicles_env.stl
  mesh/cubicles_robot.stl
  mesh/Abstract_env.stl
  mesh/Abstract_robot.stl
  mesh/pipedream_env.stl
  mesh/ring.stl
  mesh/ring.dae
)

install (FILES
  ${MESHES}
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/hpp-ompl-benchmark/mesh)

# message("install data root dir: ${CMAKE_INSTALL_DATAROOTDIR}")
# message("catkin pkg share destination: ${CATKIN_PACKAGE_SHARE_DESTINATION}")
# message("PYTHON_PACKAGES_DIR: ${PYTHON_PACKAGES_DIR}")
ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
