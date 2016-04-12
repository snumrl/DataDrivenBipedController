// +-------------------------------------------------------------------------
// | csVpWorld.h
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

#include <VP/vphysics.h>

class VpWorld
{
public:
	vpWorld _world;
	double _timeStep;
	vpBody _ground;
	scalar _planeHeight;
	scalar _lockingVel;

private:
	bool _calcPenaltyForce(vpBody* pBody, const Vec3& position, const Vec3& velocity, Vec3& force, scalar Ks, scalar Ds, scalar mu);
	bool _calcPenaltyForce_Boxes( const Vec3& boxSize, const SE3& boxFrame, const vpBody* pBody, const Vec3& position, const Vec3& velocity, Vec3& force, scalar Ks, scalar Ds, scalar mu );

public:	// expose to python
	VpWorld(const object& config);
	void step();
	void initialize();
	tuple calcPenaltyForce(const bp::list& bodyIDsToCheck, const bp::list& mus, scalar Ks, scalar Ds);
	void applyPenaltyForce(const bp::list& bodyIDs, const bp::list& positions, const bp::list& forces);
	int getBodyNum() { return _world.GetNumBody(); }
	tuple calcPenaltyForce_Boxes( const bp::list& boxSizes, const bp::list boxFrames, const bp::list& bodyIDsToCheck, const bp::list& mus, scalar Ks, scalar Ds );
};
