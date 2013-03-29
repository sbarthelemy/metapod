/* Copyright 2011, 2012,
*
* Olivier STASSE, LAAS/CNRS
*
* This file is part of metapod.
* metapod is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* metapod is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Lesser Public License for more details.
* You should have received a copy of the GNU Lesser General Public License
* along with metapod.  If not, see <http://www.gnu.org/licenses/>.
*
* Creation: 11/12/2012
*/

/**
\mainpage

This is the documentation of Metapod, a template based library for efficient dynamic computation.
Metapod is targeted for complex redundant mechanical systems such as humanoid robots.

\section main_introduction Introduction

The metapod package aims at providing an efficient template based C++ implementation of the rigid body dynamics algorithms described by Roy Featherstone in his book "Rigid Body Dynamics". The goal is to include this software in high-frequency control loop up to 1 Khz.

Metapod provides the support for the two following algorithms:
<ul>
  <li> Composite Rigid Body Algorithm (CRBA) </li>
  <li> Recursive Newton Euler Algorithm (RNEA) </li>
</ul>

Metapod may optionally create efficient C++ robot models based on their URDF file.
It relies on the URDF parser provided by ROS.

\section main_installation Installation

The installation procedure is currently only available by compiling the source code.
See \ref installation for further explanations.

\section Authors
Maxime Reis
Olivier Stasse
Sébastion Barthélemy
Antonio El Khoury
*/

