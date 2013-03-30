# Copyright 2013
#
# Sébastien Barthélémy (Aldebaran Robotics)
#
# This file is part of metapod.
# metapod is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# metapod is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with metapod.  If not, see <http://www.gnu.org/licenses/>.

QI_CREATE_BIN(metapod_binarytreemodel src/binarytreemodel.cc)
QI_USE_LIB(metapod_binarytreemodel metapod_robotbuilder)
QI_STAGE_BIN(metapod_binarytreemodel)
GET_PROPERTY(METAPOD_BINARYTREEMODEL_EXECUTABLE
  TARGET metapod_binarytreemodel PROPERTY LOCATION)
SET(METAPOD_BINARYTREEMODEL_EXECUTABLE ${METAPOD_BINARYTREEMODEL_EXECUTABLE}
  CACHE STRING
  "Set by cmake if metapod_binarytreemodel is built, by the user otherwise")
