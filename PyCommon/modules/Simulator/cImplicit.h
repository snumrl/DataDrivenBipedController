// +-------------------------------------------------------------------------
// | cImplicit.h
// |
// | Author: Yoonsang Lee
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Yoonsang Lee 2013
// |    See the included COPYRIGHT.txt file for further details.
// |    
// |    This file is part of the DataDrivenBipedController.
// |    DataDrivenBipedController is free software: you can redistribute it and/or modify
// |    it under the terms of the MIT License.
// |
// |    You should have received a copy of the MIT License
// |    along with DataDrivenBipedController.  If not, see <mit-license.org>.
// +-------------------------------------------------------------------------

#pragma once

#include "csIMSModel.h"

BOOST_PYTHON_MODULE(csIMSModel)
{
	class_<ParticleConfig>("ParticleConfig", init<>())
		.def(init<const object&, const double, const object&, const double, const double>())
		.def_readwrite("position", &ParticleConfig::position)
		.def_readwrite("initialVelocity", &ParticleConfig::initialVelocity)
		.def_readwrite("mass", &ParticleConfig::mass)
		.def_readwrite("dynamicMu", &ParticleConfig::dynamicMu)
		.def_readwrite("staticMu", &ParticleConfig::staticMu)
		.def("__str__", &ParticleConfig::__str__)
		.def_pickle(ParticleConfig_pickle_suite())
		;
	class_<SpringConfig>("SpringConfig", init<int, int, double, double>())
		.def_readwrite("particleIndex0", &SpringConfig::particleIndex0)
		.def_readwrite("particleIndex1", &SpringConfig::particleIndex1)
		.def_readwrite("Ks", &SpringConfig::Ks)
		.def_readwrite("Kd", &SpringConfig::Kd)
		.def_readwrite("subspringsName", &SpringConfig::subspringsName)
		.def("__str__", &SpringConfig::__str__)
		.def_pickle(SpringConfig_pickle_suite())
		;
	class_<SystemConfig>("SystemConfig")
		.def_readwrite("g", &SystemConfig::g)
		.def_readwrite("tangentLockingVel", &SystemConfig::tangentLockingVel)
		.def("__str__", &SystemConfig::__str__)
		.def_pickle(SystemConfig_pickle_suite())
		;
	class_<IMSModel>("IMSModel", init<const list&, const list&, const SystemConfig&>())
		.def("step", &IMSModel::step)
		.def("setMu", &IMSModel::setMu)
		.def("updateSprings", &IMSModel::updateSprings)
		.def("getPosition", &IMSModel::getPosition)
		.def("getVelocity", &IMSModel::getVelocity)
		.def("getPositions", &IMSModel::getPositions)
		.def("getVelocities", &IMSModel::getVelocities)
		.def("getContactParticleIndices", &IMSModel::getContactParticleIndices)
//		.def("setVelocity", &IMSModel::setVelocity)
		.def("getParticleNum", &IMSModel::getParticleNum)
		.def("getState", &IMSModel::getState)
		.def("setState", &IMSModel::setState)
		;
}
