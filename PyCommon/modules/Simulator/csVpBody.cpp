// +-------------------------------------------------------------------------
// | csVpBody.cpp
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

#include "stdafx.h"

#include "../../common_sources/bputil.h"

#include "../../common_sources/vputil.h"
#include "csVpWorld.h"
#include "csVpBody.h"
#include "myGeom.h"

BOOST_PYTHON_MODULE(csVpBody)
{
	numeric::array::set_module_and_type("numpy", "ndarray");

	class_<VpBody>("VpBody", init<VpWorld*>())
		.def("addBoxGeom", &VpBody::addBoxGeom)
		.def("getShape", &VpBody::getShape)
		.def("setFrame", &VpBody::setFrame)
		.def("getFrame", &VpBody::getFrame)
		.def("setPosition", &VpBody::setPosition)
		.def("setOrientation", &VpBody::setOrientation)
		;
}

VpBody::VpBody( VpWorld* pWorld )
{
	pWorld->_world.AddBody(&_body);
}

void VpBody::addBoxGeom( const object& size, scalar density )
{
	_shape = size;

	Vec3 vSize = pyVec3_2_Vec3(size);

//	_body.AddGeometry(new MyBox(vSize));
	_body.AddGeometry(new vpBox(vSize));
	_body.SetInertia(BoxInertia(density, vSize));
}

object VpBody::getShape()
{
//	static numeric::array O_Vec3(make_tuple(0.,0.,0.));
//	pyV = O.copy();
	return _shape;	
}

void VpBody::setPosition( const object& pyV )
{
	SE3 T = _body.GetFrame();
	T.SetPosition(pyVec3_2_Vec3(pyV));
	_body.SetFrame(T);
}

void VpBody::setOrientation( const object& pyR )
{
	SE3 T = _body.GetFrame();
	Vec3 p = T.GetPosition();
	T.SetOrientation(pySO3_2_SE3(pyR));
	T.SetPosition(p);
	_body.SetFrame(T);
}

void VpBody::setFrame( const object& pyT )
{
	SE3 T;
	pySE3_2_SE3(pyT, T);
	_body.SetFrame(T);
}

object VpBody::getFrame()
{
	static numeric::array I_SE3( make_tuple(make_tuple(1.,0.,0.,0.), make_tuple(0.,1.,0.,0.), make_tuple(0.,0.,1.,0.), make_tuple(0.,0.,0.,1.)) );

	object pyT = I_SE3.copy();
	SE3_2_pySE3(_body.GetFrame(), pyT);
	return pyT;
}


//
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getBodyPositionGlobal_py_overloads, getBodyPositionGlobal_py, 1, 2);
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getBodyVelocityGlobal_py_overloads, getBodyVelocityGlobal_py, 1, 2);
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpMotionModel_recordVelByFiniteDiff_overloads, recordVelByFiniteDiff, 0, 2);
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpControlModel_applyBodyGenForceGlobal_overloads, applyBodyGenForceGlobal, 3, 4);
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpControlModel_applyBodyForceGlobal_overloads, applyBodyForceGlobal, 2, 3);
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(initializeHybridDynamics_overloads, initializeHybridDynamics, 0, 1);
//
//BOOST_PYTHON_MODULE(csVpModel)
//{
//	numeric::array::set_module_and_type("numpy", "ndarray");
//
//	class_<VpModel>("VpModel", init<VpWorld*, object, object>())
//		.def("__str__", &VpModel::__str__)
//		.def("getBodyNum", &VpModel::getBodyNum)
//		.def("getBodyMasses", &VpModel::getBodyMasses)
//		.def("getTotalMass", &VpModel::getTotalMass)
//		.def("getBodyShape", &VpModel::getBodyShape)
//		.def("getBodyVerticesGlobal", &VpModel::getBodyVerticesGlobal_py)
//
//		.def("index2name", &VpModel::index2name)
//		.def("index2id", &VpModel::index2id)
//		.def("name2index", &VpModel::name2index)
//		.def("name2id", &VpModel::name2id)
//
//		.def("getBodyInertiaLocal", &VpModel::getBodyInertiaLocal_py)
//		.def("getBodyInertiaGlobal", &VpModel::getBodyInertiaGlobal_py)
//
//		.def("getBodyInertiasLocal", &VpModel::getBodyInertiasLocal)
//		.def("getBodyInertiasGlobal", &VpModel::getBodyInertiasGlobal)
//		
//		.def("getBodyPositionGlobal", &VpModel::getBodyPositionGlobal_py, getBodyPositionGlobal_py_overloads())
//		.def("getBodyVelocityGlobal", &VpModel::getBodyVelocityGlobal_py, getBodyVelocityGlobal_py_overloads())
//		.def("getBodyAccelerationGlobal", &VpModel::getBodyAccelerationGlobal_py)
//		.def("getBodyAngVelocityGlobal", &VpModel::getBodyAngVelocityGlobal)
//		.def("getBodyAngAccelerationGlobal", &VpModel::getBodyAngAccelerationGlobal)
//
//		.def("getBodyPositionsGlobal", &VpModel::getBodyPositionsGlobal)
//		.def("getBodyVelocitiesGlobal", &VpModel::getBodyVelocitiesGlobal)
//		.def("getBodyAccelerationsGlobal", &VpModel::getBodyAccelerationsGlobal)
//		.def("getBodyAngVelocitiesGlobal", &VpModel::getBodyAngVelocitiesGlobal)
//		.def("getBodyAngAccelerationsGlobal", &VpModel::getBodyAngAccelerationsGlobal)
//
//		.def("setBodyPositionGlobal", &VpModel::setBodyPositionGlobal_py)
//		.def("setBodyVelocityGlobal", &VpModel::setBodyVelocityGlobal_py)
//		.def("setBodyAccelerationGlobal", &VpModel::setBodyAccelerationGlobal_py)
//		.def("setBodyAngVelocityGlobal", &VpModel::setBodyAngVelocityGlobal)
//		.def("setBodyAngAccelerationGlobal", &VpModel::setBodyAngAccelerationGlobal)
//
//		.def("translateByOffset", &VpModel::translateByOffset)
//		.def("rotate", &VpModel::rotate)
//		;
//
//	class_<VpMotionModel, bases<VpModel> >("VpMotionModel", init<VpWorld*, object, object>())
//		.def("update", &VpMotionModel::update)
//		.def("recordVelByFiniteDiff", &VpMotionModel::recordVelByFiniteDiff, VpMotionModel_recordVelByFiniteDiff_overloads())
//		;
//
//	class_<VpControlModel, bases<VpModel> >("VpControlModel", init<VpWorld*, object, object>())
//		.def("__str__", &VpControlModel::__str__)
//		.def("getJointNum", &VpControlModel::getJointNum)
//		.def("getInternalJointNum", &VpControlModel::getInternalJointNum)
//		.def("getDOFs", &VpControlModel::getDOFs)
//		.def("getInternalJointDOFs", &VpControlModel::getInternalJointDOFs)
//		.def("getTotalDOF", &VpControlModel::getTotalDOF)
//		.def("getTotalInternalJointDOF", &VpControlModel::getTotalInternalJointDOF)
//
//		.def("update", &VpControlModel::update)
//		.def("fixBody", &VpControlModel::fixBody)
//
//		.def("initializeHybridDynamics", &VpControlModel::initializeHybridDynamics, initializeHybridDynamics_overloads())
//		.def("solveHybridDynamics", &VpControlModel::solveHybridDynamics)
//
//		.def("getDOFPositions", &VpControlModel::getDOFPositions)
//		.def("getDOFVelocities", &VpControlModel::getDOFVelocities)
//		.def("getDOFAccelerations", &VpControlModel::getDOFAccelerations)
//		.def("getDOFAxeses", &VpControlModel::getDOFAxeses)
//
//		.def("setDOFAccelerations", &VpControlModel::setDOFAccelerations)
//
//		.def("getJointOrientationLocal", &VpControlModel::getJointOrientationLocal)
//		.def("getJointAngVelocityLocal", &VpControlModel::getJointAngVelocityLocal)
//		.def("getJointAngAccelerationLocal", &VpControlModel::getJointAngAccelerationLocal)
//
//		.def("getJointPositionGlobal", &VpControlModel::getJointPositionGlobal)
//		.def("getJointVelocityGlobal", &VpControlModel::getJointVelocityGlobal)
//		.def("getJointAccelerationGlobal", &VpControlModel::getJointAccelerationGlobal)
//		.def("getJointOrientationGlobal", &VpControlModel::getJointOrientationGlobal)
//		.def("getJointAngVelocityGlobal", &VpControlModel::getJointAngVelocityGlobal)
//		.def("getJointAngAccelerationGlobal", &VpControlModel::getJointAngAccelerationGlobal)
//
//		.def("getJointOrientationsLocal", &VpControlModel::getJointOrientationsLocal)
//		.def("getJointAngVelocitiesLocal", &VpControlModel::getJointAngVelocitiesLocal)
//		.def("getJointAngAccelerationsLocal", &VpControlModel::getJointAngAccelerationsLocal)
//
//		.def("getJointPositionsGlobal", &VpControlModel::getJointPositionsGlobal)
//		.def("getJointVelocitiesGlobal", &VpControlModel::getJointVelocitiesGlobal)
//		.def("getJointAccelerationsGlobal", &VpControlModel::getJointAccelerationsGlobal)
//		.def("getJointOrientationsGlobal", &VpControlModel::getJointOrientationsGlobal)
//		.def("getJointAngVelocitiesGlobal", &VpControlModel::getJointAngVelocitiesGlobal)
//		.def("getJointAngAccelerationsGlobal", &VpControlModel::getJointAngAccelerationsGlobal)
//
//		.def("getInternalJointOrientationsLocal", &VpControlModel::getInternalJointOrientationsLocal)
//		.def("getInternalJointAngVelocitiesLocal", &VpControlModel::getInternalJointAngVelocitiesLocal)
//		.def("getInternalJointAngAccelerationsLocal", &VpControlModel::getInternalJointAngAccelerationsLocal)
//
//		.def("getInternalJointPositionsGlobal", &VpControlModel::getInternalJointPositionsGlobal)
//		.def("getInternalJointOrientationsGlobal", &VpControlModel::getInternalJointOrientationsGlobal)
//
//		.def("setJointAngVelocityLocal", &VpControlModel::setJointAngVelocityLocal)
//		.def("setJointAngAccelerationLocal", &VpControlModel::setJointAngAccelerationLocal)
//
//		.def("setJointAccelerationGlobal", &VpControlModel::setJointAccelerationGlobal)
//		.def("setJointAngAccelerationGlobal", &VpControlModel::setJointAngAccelerationGlobal)
//
//		.def("setJointAngAccelerationsLocal", &VpControlModel::setJointAngAccelerationsLocal)
//		
//		.def("setInternalJointAngAccelerationsLocal", &VpControlModel::setInternalJointAngAccelerationsLocal)
//		
//		.def("getJointFrame", &VpControlModel::getJointFrame)
//
//
//		.def("applyBodyGenForceGlobal", &VpControlModel::applyBodyGenForceGlobal, VpControlModel_applyBodyGenForceGlobal_overloads())
//		.def("applyBodyForceGlobal", &VpControlModel::applyBodyForceGlobal, VpControlModel_applyBodyForceGlobal_overloads())
//		.def("applyBodyTorqueGlobal", &VpControlModel::applyBodyTorqueGlobal)
//
//		.def("getBodyForceLocal", &VpControlModel::getBodyForceLocal)
//		.def("getBodyNetForceLocal", &VpControlModel::getBodyNetForceLocal)
//		.def("getBodyGravityForceLocal", &VpControlModel::getBodyGravityForceLocal)
//
//		.def("getJointTorqueLocal", &VpControlModel::getJointTorqueLocal)
//		.def("getInternalJointTorquesLocal", &VpControlModel::getInternalJointTorquesLocal)
//
//		.def("setJointTorqueLocal", &VpControlModel::setJointTorqueLocal)
//		.def("setInternalJointTorquesLocal", &VpControlModel::setInternalJointTorquesLocal)
//		;
//}
//
// 
//VpModel::VpModel( VpWorld* pWorld, const object& createPosture, const object& config ) 
//:_pWorld(&pWorld->_world), _config(config), _skeleton(createPosture.attr("skeleton"))
//{
//	int num = XI(createPosture.attr("skeleton").attr("getJointNum")());
//	_nodes.resize(num, NULL);
//	_boneTs.resize(num, SE3());
//
//	createBodies(createPosture);
//	build_name2index();
//}
//
//VpModel::~VpModel()
//{
//	for( NODES_ITOR it=_nodes.begin(); it!=_nodes.end(); ++it)
//		if(*it)
//			delete *it;
//}
//
//void VpModel::createBodies( const object& posture )
//{
//	object joint = posture.attr("skeleton").attr("root");
//
//	object rootPos = posture.attr("rootPos");
//	SE3 T = SE3(pyVec3_2_Vec3(rootPos));
//
//	object tpose = posture.attr("getTPose")();
//	_createBody(joint, T, tpose);
//}
//
//void VpModel::_createBody( const object& joint, const SE3& parentT, const object& posture )
//{
//	int len_joint_children = len(joint.attr("children")); 
//	if (len_joint_children == 0 )
//		return;
//
//	SE3 T = parentT;
//
//	SE3 P = SE3(pyVec3_2_Vec3(joint.attr("offset")));
//	T = T * P;
//
//	string joint_name = XS(joint.attr("name"));
//	//	int joint_index = XI(posture.attr("skeleton").attr("getElementIndex")(joint_name));
//	//	SE3 R = pySO3_2_SE3(posture.attr("getLocalR")(joint_index));
//	int joint_index = XI(posture.attr("skeleton").attr("getJointIndex")(joint_name));
//	SE3 R = pySO3_2_SE3(posture.attr("getJointOrientationLocal")(joint_index));
//	T = T * R;
//
////	if (len_joint_children > 0 && _config.attr("hasNode")(joint_name))
//	if (_config.attr("hasNode")(joint_name))
//	{
//
//		Vec3 offset(0);
//		for( int i=0 ; i<len_joint_children; ++i)
//			offset += pyVec3_2_Vec3(joint.attr("children")[i].attr("offset"));
//		offset *= (1./len_joint_children);
//
//		SE3 boneT(offset*.5);
//
//		Vec3 defaultBoneV(0,0,1);
//		SE3 boneR = getSE3FromVectors(defaultBoneV, offset);
//		boneT = boneT * boneR;
//
//		Node* pNode = new Node(joint_name);
//		_nodes[joint_index] = pNode;
//
//		object cfgNode = _config.attr("getNode")(joint_name);
//		scalar length;
//		if( cfgNode.attr("length") != object() )
//			length = XD(cfgNode.attr("length")) * XD(cfgNode.attr("boneRatio"));
//		else
//			length = Norm(offset) * XD(cfgNode.attr("boneRatio"));
//
//		scalar density = XD(cfgNode.attr("density"));
//		scalar width, height;
//		if( cfgNode.attr("width") != object() )
//		{
//			width = XD(cfgNode.attr("width"));
//			if( cfgNode.attr("mass") != object() )
//				height = (XD(cfgNode.attr("mass")) / (density * length)) / width;
//			else
//				height = .1;
//		}
//		else
//		{
//			if( cfgNode.attr("mass") != object() )
//				width = sqrt( (XD(cfgNode.attr("mass")) / (density * length)) );
//			else
//				width = .1;
//			height = width;
//		}
//
//		string geomType = XS(cfgNode.attr("geom"));
//		if(geomType == "MyBox")
//			pNode->body.AddGeometry(new MyBox(Vec3(width, height, length)));
//		else if(geomType == "MyFoot1")
//			pNode->body.AddGeometry(new MyFoot1(Vec3(width, height, length)));
//		else if(geomType == "MyFoot2")
//			pNode->body.AddGeometry(new MyFoot2(Vec3(width, height, length)));
////		else if(geomType == "MyShin")
////			pNode->body.AddGeometry(new MyShin(Vec3(width, height, length)));
//		else
//			cout << geomType << " : undefined geom type" << endl;
//
//		pNode->body.SetInertia(BoxInertia(density, Vec3(width/2.,height/2.,length/2.)));
//
//		boneT = boneT * SE3(pyVec3_2_Vec3(cfgNode.attr("offset")));
//		_boneTs[joint_index] = boneT;
//		SE3 newT = T * boneT;
//
//		pNode->body.SetFrame(newT);
//	}
//
//	for( int i=0 ; i<len_joint_children; ++i)
//		_createBody(joint.attr("children")[i], T, posture);
//}
//
//const vector<Vec3>& VpModel::getBodyVerticesGlobal( int index )
//{
//	static vector<Vec3> verticesGlobal;
//	const vpGeom *pGeom;
//	char type;
//	scalar data[3];
//
//	verticesGlobal.clear();
//
//	for( int j=0; j<_nodes[index]->body.GetNumGeometry(); ++j)
//	{
//		pGeom = _nodes[index]->body.GetGeometry(j);
//		const vector<Vec3>& geomVertices = pGeom->getVerticesGlobal();
//		verticesGlobal.insert(verticesGlobal.end(), geomVertices.begin(), geomVertices.end());
//	}
//	return verticesGlobal;
//}
//
//void VpModel::getBodyInertiaLocal(int index, SE3& Tin)
//{
////	ss << "I11 I22 I33 I12 I13 I23 offset.x offset.y offset.z mass" << endl;
//
////	pyIn[make_tuple(0,0)] = in[0];
////	pyIn[make_tuple(1,1)] = in[1];
////	pyIn[make_tuple(2,2)] = in[2];
////	pyIn[make_tuple(0,1)] = pyIn[make_tuple(1,0)] = in[3];
////	pyIn[make_tuple(0,2)] = pyIn[make_tuple(2,0)] = in[4];
////	pyIn[make_tuple(1,2)] = pyIn[make_tuple(2,1)] = in[5];
//	
////		| T[0]	T[3]	T[6]	T[ 9] |
////		| T[1]	T[4]	T[7]	T[10] |
////		| T[2]	T[5]	T[8]	T[11] |
//
//	const Inertia& in = _nodes[index]->body.GetInertia();
//
//	Tin[0] = in[0];
//	Tin[4] = in[1];
//	Tin[8] = in[2];
//	Tin[3] = Tin[1] = in[3];
//	Tin[6] = Tin[2] = in[4];
//	Tin[7] = Tin[5] = in[5];
//}
//
//Vec3 VpModel::getBodyPositionGlobal( int index, const Vec3* pPositionLocal )
//{
//	static SE3 bodyFrame;
//	bodyFrame = _nodes[index]->body.GetFrame();
//	if(!pPositionLocal)
//		return bodyFrame.GetPosition();
//	else
//		return bodyFrame * (*pPositionLocal);
//}
//Vec3 VpModel::getBodyVelocityGlobal( int index, const Vec3& positionLocal)
//{
//	return _nodes[index]->body.GetLinVelocity(positionLocal);
//
////	static se3 genAccLocal, genAccGlobal;
////	genAccLocal = _nodes[index]->body.GetGenVelocityLocal();
////	genAccLocal = MinusLinearAd(positionLocal, genAccLocal);
////	genAccGlobal = Rotate(_nodes[index]->body.GetFrame(), genAccLocal);
////	return Vec3(genAccGlobal[3], genAccGlobal[4], genAccGlobal[5]);
//}
//
//Vec3 VpModel::getBodyAccelerationGlobal( int index, const Vec3* pPositionLocal)
//{
//	static se3 genAccLocal, genAccGlobal;
//
//	genAccLocal = _nodes[index]->body.GetGenAccelerationLocal();
//	if(pPositionLocal)
//		genAccLocal = MinusLinearAd(*pPositionLocal, genAccLocal);
// 
//	genAccGlobal = Rotate(_nodes[index]->body.GetFrame(), genAccLocal);
//
//	return Vec3(genAccGlobal[3], genAccGlobal[4], genAccGlobal[5]);
//}
//
//void VpModel::setBodyPositionGlobal( int index, const Vec3& position )
//{
//	static SE3 bodyFrame;
//	bodyFrame = _nodes[index]->body.GetFrame();
//	bodyFrame.SetPosition(position);
//	_nodes[index]->body.SetFrame(bodyFrame);
//}
//
//void VpModel::setBodyAccelerationGlobal( int index, const Vec3& acc, const Vec3* pPositionLocal)
//{
////	if(pPositionLocal)
////		cout << "pPositionLocal : not implemented functionality yet" << endl;
//
//	static se3 genAcc;
//	genAcc = _nodes[index]->body.GetGenAcceleration();
//	genAcc[3] = acc[0];
//	genAcc[4] = acc[1];
//	genAcc[5] = acc[2];
//
//	_nodes[index]->body.SetGenAcceleration(genAcc);
//}
//
////int VpModel::getParentIndex( int index )
////{
////	object parent = _skeleton.attr("getParentJointIndex")(index);
////	if(parent==object())
////		return -1;
////	else
////		return XI(parent);
////}
//
//std::string VpModel::__str__()
//{
//	static numeric::array O(make_tuple(0.,0.,0.));
//
//	stringstream ss;
////	ss << "<NODES>" << endl;
////	for(int i=0; i<_nodes.size(); ++i)
////	{
////		ss << "[" << i << "]:";
//////		if(_nodes[i]==NULL)
//////			ss << "NULL, ";
//////		else
////			ss << _nodes[i]->name << ", ";
////	}
////	ss << endl;
////
////	ss << "<BODIES INDEX:(NODE INDEX) NODE NAME>\n";
////	for(int i=0; i<_bodyElementIndexes.size(); ++i)
////		ss << "[" << i << "]:(" << _bodyElementIndexes[i] << ") " << _nodes[_bodyElementIndexes[i]]->name << ", ";
////	ss << endl;
//
//	ss << "<BODIES & JOINTS>" << endl;
//	for(int i=0; i<_nodes.size(); ++i)
//		ss << "[" << i << "]:" << _nodes[i]->name << ", ";
//	ss << endl;
//
//	ss << "<BODY MASSES>" << endl;
//	for(int i=0; i<_nodes.size(); ++i)
//		ss << "[" << i << "]:" << _nodes[i]->body.GetInertia().GetMass() << ", ";
//	ss << endl;
//
//	ss << "<BODY SHAPES>" << endl;
//	for(int i=0; i<_nodes.size(); ++i)
//	{
//		object pyV = getBodyShape(i);
//		ss << "[" << i << "]:" << "(" << XD(pyV[0]) << " " << XD(pyV[1]) << " " << XD(pyV[2]) << ")" << ", ";
//	}
//	ss << endl;
//
////	ss << "<BODY INERTIAS>" << endl;
////	ss << "I11 I22 I33 I12 I13 I23 offset.x offset.y offset.z mass" << endl;
////	for(int i=0; i<_nodes.size(); ++i)
////		if(_nodes[i])
////		{
////			ss << "[" << i << "]:";
////			for(int j=0; j<10; ++j)
////				ss << _nodes[i]->body.GetInertia()[j] << " ";
////			ss << endl;
////		}
////	ss << endl;
//
//	return ss.str();
//}
//
//bp::list VpModel::getBodyMasses()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(_nodes[i]->body.GetInertia().GetMass());
//	return ls;
//}
//
//scalar VpModel::getTotalMass()
//{
//	scalar mass = 0.;
//	for(int i=0; i<_nodes.size(); ++i)
//		mass += _nodes[i]->body.GetInertia().GetMass();
//	return mass;
//}
//
//object VpModel::getBodyShape(int index)
//{
//	static numeric::array O(make_tuple(0.,0.,0.));
//	char type;
//	scalar data[3];
//
//	_nodes[index]->body.GetGeometry(0)->GetShape(&type, data);
//
//	object pyV = O.copy();
//	pyV[0] = data[0];
//	pyV[1] = data[1];
//	pyV[2] = data[2];
//
//	return pyV;
//}
//
//bp::list VpModel::getBodyVerticesGlobal_py(int index)
//{
//	static numeric::array O(make_tuple(0.,0.,0.));
//	bp::list ls_point;
//
//	const vector<Vec3>& verticesGlobal = getBodyVerticesGlobal(index);
//	for(int i=0; i<verticesGlobal.size(); ++i)
//	{
//		object pyV = O.copy();
//		Vec3_2_pyVec3(verticesGlobal[i], pyV);
//		ls_point.append(pyV);
//	}
//
//	return ls_point;
//}
//
//boost::python::object VpModel::getBodyInertiaLocal_py( int index )
//{
//	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
//	static SE3 Tin;
//	object pyIn = I.copy();
//
//	getBodyInertiaLocal(index, Tin);
//	SE3_2_pySO3(Tin, pyIn);
//	return pyIn;
//}
//
//boost::python::object VpModel::getBodyInertiaGlobal_py( int index )
//{
//	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
//	static SE3 Tin_local, bodyFrame;
//	object pyIn = I.copy();
//
//	getBodyInertiaLocal(index, Tin_local);
//	bodyFrame = _nodes[index]->body.GetFrame();
//	SE3_2_pySO3(bodyFrame * Tin_local * Inv(bodyFrame), pyIn);
//	return pyIn;
//	
//}
//
//bp::list VpModel::getBodyInertiasLocal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getBodyInertiaLocal_py(i));
//	return ls;
//}
//
//bp::list VpModel::getBodyInertiasGlobal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getBodyInertiaGlobal_py(i));
//	return ls;
//}
//
//object VpModel::getBodyPositionGlobal_py( int index, const object& positionLocal/*=object() */ )
//{
//	static numeric::array O(make_tuple(0.,0.,0.));
//	object pyV = O.copy();
//	static Vec3 positionLocal_;
//
//	if(positionLocal==object())
//		Vec3_2_pyVec3(getBodyPositionGlobal(index), pyV);
//	else
//	{
//		pyVec3_2_Vec3(positionLocal, positionLocal_);
//		Vec3_2_pyVec3(getBodyPositionGlobal(index, &positionLocal_), pyV);
//	}
//	return pyV;
//}
//object VpModel::getBodyVelocityGlobal_py( int index, const object& positionLocal/*=object() */ )
//{
//	static numeric::array O(make_tuple(0.,0.,0.));
//	object pyV = O.copy();
//	static Vec3 positionLocal_;
//	
//	if(positionLocal==object())
//		Vec3_2_pyVec3(getBodyVelocityGlobal(index), pyV);
//	else
//	{
//		pyVec3_2_Vec3(positionLocal, positionLocal_);
//		Vec3_2_pyVec3(getBodyVelocityGlobal(index, positionLocal_), pyV);
//	}
//	return pyV;
//}
//
//bp::list VpModel::getBodyVelocitiesGlobal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getBodyVelocityGlobal_py(i));
//	return ls;
//}
//
//object VpModel::getBodyAngVelocityGlobal( int index )
//{
//	static se3 genVel;
//	static numeric::array O(make_tuple(0.,0.,0.));
//	object pyV = O.copy();
//
//	genVel = _nodes[index]->body.GetGenVelocity();
//	pyV[0] = genVel[0];
//	pyV[1] = genVel[1];
//	pyV[2] = genVel[2];
//	return pyV;
//}
//
//bp::list VpModel::getBodyAngVelocitiesGlobal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getBodyAngVelocityGlobal(i));
//	return ls;
//}
//
//object VpModel::getBodyAccelerationGlobal_py(int index, const object& positionLocal )
//{
//	static se3 genAcc;
//	static Vec3 positionLocal_;
//	static numeric::array O(make_tuple(0.,0.,0.));
//	object pyV = O.copy();
//
//	if(positionLocal==object())
//		Vec3_2_pyVec3(getBodyAccelerationGlobal(index), pyV);
//	else
//	{
//		pyVec3_2_Vec3(positionLocal, positionLocal_);
//		Vec3_2_pyVec3(getBodyAccelerationGlobal(index, &positionLocal_), pyV);
//	}
//	return pyV;
//}
//
//bp::list VpModel::getBodyAccelerationsGlobal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getBodyAccelerationGlobal_py(i));
//	return ls;
//}
//
//void VpModel::setBodyPositionGlobal_py( int index, const object& pos )
//{
//	static Vec3 position;
//
//	pyVec3_2_Vec3(pos, position);
//	setBodyPositionGlobal(index, position); 
//}
//
//void VpModel::setBodyVelocityGlobal_py( int index, const object& vel )
//{
//	static se3 genVel;
//	genVel = _nodes[index]->body.GetGenVelocity();
//	genVel[3] = XD(vel[0]);
//	genVel[4] = XD(vel[1]);
//	genVel[5] = XD(vel[2]);
//	_nodes[index]->body.SetGenVelocity(genVel);
//}
//
//void VpModel::setBodyAccelerationGlobal_py( int index, const object& acc )
//{
//	static se3 genAcc;
//	genAcc = _nodes[index]->body.GetGenAcceleration();
//	genAcc[3] = XD(acc[0]);
//	genAcc[4] = XD(acc[1]);
//	genAcc[5] = XD(acc[2]);
//	_nodes[index]->body.SetGenAcceleration(genAcc);
//}
//
//void VpModel::setBodyAngVelocityGlobal( int index, const object& angvel )
//{
//	static se3 genVel;
//	genVel = _nodes[index]->body.GetGenVelocity();
//	genVel[0] = XD(angvel[0]);
//	genVel[1] = XD(angvel[1]);
//	genVel[2] = XD(angvel[2]);
//	_nodes[index]->body.SetGenVelocity(genVel);
//}
//
//void VpModel::setBodyAngAccelerationGlobal( int index, const object& angacc )
//{
//	static se3 genAcc;
//	genAcc = _nodes[index]->body.GetGenAcceleration();
//	genAcc[0] = XD(angacc[0]);
//	genAcc[1] = XD(angacc[1]);
//	genAcc[2] = XD(angacc[2]);
//	_nodes[index]->body.SetGenAcceleration(genAcc);
//}
//
//bp::list VpModel::getBodyPositionsGlobal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getBodyPositionGlobal_py(i));
//	return ls;
//}
//
//object VpModel::getBodyAngAccelerationGlobal( int index )
//{
//	static se3 genAcc;
//	static numeric::array O(make_tuple(0.,0.,0.));
//	object pyV = O.copy();
//
//	genAcc = _nodes[index]->body.GetGenAcceleration();
//	pyV[0] = genAcc[0];
//	pyV[1] = genAcc[1];
//	pyV[2] = genAcc[2];
//	return pyV;
//}
//
//bp::list VpModel::getBodyAngAccelerationsGlobal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getBodyAngAccelerationGlobal(i));
//	return ls;
//}
//
//void VpModel::translateByOffset( const object& offset )
//{
//	static Vec3 v;
//	pyVec3_2_Vec3(offset, v);
//
//	for(int i=0; i<_nodes.size(); ++i)
//		setBodyPositionGlobal(i, getBodyPositionGlobal(i) + v);
//}
//
//void VpModel::rotate( const object& rotation )
//{
//	static SE3 R, bodyFrame;
//	pySO3_2_SE3(rotation, R);
//
//	bodyFrame = _nodes[0]->body.GetFrame();
//	_nodes[0]->body.SetFrame(bodyFrame * R);
//
//	// 바뀐 root body frame에 따라 joint로 연결된 나머지 body들 frame 업데이트. 이것을 하지 않으면 root body 하나만 rotation이 적용된다. 
//	_pWorld->UpdateFrame();
//}
//
//VpMotionModel::VpMotionModel( VpWorld* pWorld, const object& createPosture, const object& config )
//	:VpModel(pWorld, createPosture, config), _recordVelByFiniteDiff(false), _inverseMotionTimeStep(30.)
//{
//	// OdeMotionModel의 node.body.disable()은 VpMotionModel에서는 pWorld->AddBody()를
//	// 안 해주는 것으로 그 역할을 하도록 한다.
//
//	update(createPosture);
//}
//
//
//void VpMotionModel::update( const object& posture)
//{
//	object joint = posture.attr("skeleton").attr("root");
//	object rootPos = posture.attr("rootPos");
//	SE3 T = SE3(pyVec3_2_Vec3(rootPos));
//	_updateBody(joint, T, posture);
//}
//
//
//void VpMotionModel::_updateBody( const object& joint, const SE3& parentT, const object& posture)
//{
//	int len_joint_children = len(joint.attr("children")); 
//	if (len_joint_children == 0 )
//		return;
//
//	SE3 T = parentT;
//
//	SE3 P = SE3(pyVec3_2_Vec3(joint.attr("offset")));
//	T = T * P;
//
//	string joint_name = XS(joint.attr("name"));
////	int joint_index = XI(posture.attr("skeleton").attr("getElementIndex")(joint_name));
////	SE3 R = pySO3_2_SE3(posture.attr("getLocalR")(joint_index));
//	int joint_index = XI(posture.attr("skeleton").attr("getJointIndex")(joint_name));
//	SE3 R = pySO3_2_SE3(posture.attr("getJointOrientationLocal")(joint_index));
//	T = T * R;
//
////	int len_joint_children = len(joint.attr("children")); 
////	if (len_joint_children > 0 && _config.attr("hasNode")(joint_name))
//	if (_config.attr("hasNode")(joint_name))
//	{
//		SE3 boneT = _boneTs[joint_index];
//		SE3 newT = T * boneT;
//
//		Node* pNode = _nodes[joint_index];
//
//		if(_recordVelByFiniteDiff)
//		{
//			static SE3 oldT, diffT;
//			oldT = pNode->body.GetFrame();
//			diffT = newT * Inv(oldT);
//
//			Vec3 p = newT.GetPosition() - oldT.GetPosition();
//			diffT.SetPosition(p);
//
//			pNode->body.SetGenVelocity(Log(diffT) * _inverseMotionTimeStep);
//		}
//
//		pNode->body.SetFrame(newT);
//	}
//
//	for( int i=0 ; i<len_joint_children; ++i)
//		_updateBody(joint.attr("children")[i], T, posture);	
//}
//
//
//VpControlModel::VpControlModel( VpWorld* pWorld, const object& createPosture, const object& config )
//	:VpModel(pWorld, createPosture, config)
//{
//	addBodiesToWorld(createPosture);
//	ignoreCollisionBtwnBodies();
//
//	object tpose = createPosture.attr("getTPose")();
//	createJoints(tpose);
//
//	update(createPosture);
//}
//
//std::string VpControlModel::__str__()
//{
//	string s1 = VpModel::__str__();
//
//	stringstream ss;
//
////	ss << "<INTERNAL JOINTS>" << endl;
////	for(int i=1; i<_nodes.size(); ++i)
////		ss << "[" << i-1 << "]:" << _nodes[i]->name << ", ";
////	ss << endl;
//
//	return s1 + ss.str();
//}
//
//bp::list VpControlModel::getInternalJointDOFs()
//{
//	bp::list ls;
//	for(int i=1; i<_nodes.size(); ++i)
//		ls.append(3);
//	return ls;
//}
//
//int VpControlModel::getTotalInternalJointDOF()
//{
//	int dof = 0;
//	for(int i=1; i<_nodes.size(); ++i)
//		dof += 3;
//	return dof;
//}
//
//bp::list VpControlModel::getDOFs()
//{
//	bp::list ls;
//	ls.append(6);
//	for(int i=1; i<_nodes.size(); ++i)
//		ls.append(3);
//	return ls;
//}
//int VpControlModel::getTotalDOF()
//{
//	int dof = 0;
//	dof += 6;
//	for(int i=1; i<_nodes.size(); ++i)
//		dof += 3;
//	return dof;
//}
//
//void VpControlModel::createJoints( const object& posture )
//{
//	object joint = posture.attr("skeleton").attr("root");
//	_createJoint(joint, posture);
//}
//
//void VpControlModel::_createJoint( const object& joint, const object& posture )
//{
//	int len_joint_children = len(joint.attr("children")); 
//	if (len_joint_children == 0 )
//		return;
//
//	SE3 invLocalT;
//
//	object offset = joint.attr("offset");
//	SE3 P = SE3(pyVec3_2_Vec3(joint.attr("offset")));
//
//	string joint_name = XS(joint.attr("name"));
////	int joint_index = XI(posture.attr("skeleton").attr("getElementIndex")(joint_name));
////	SE3 R = pySO3_2_SE3(posture.attr("getLocalR")(joint_index));
//	int joint_index = XI(posture.attr("skeleton").attr("getJointIndex")(joint_name));
//	SE3 R = pySO3_2_SE3(posture.attr("getJointOrientationLocal")(joint_index));
//
//	// parent      <--------->        child
//	// link     L1      L2      L3      L4
//	// L4_M =  P1*R1 * P2*R2 * P3*R3 * P4*R4  (forward kinematics matrix of L4)
//	// 으로 나타내지지만 여기에선 while loop를 썼기 때문에 back tracking이 어려워
//	// L4_M = Inv( Inv(R4)*Inv(P4) * Inv(R3)*Inv(P3) * ...)
//	// 으로 코딩했음.
//
//	invLocalT = invLocalT * Inv(R);
//	invLocalT = invLocalT * Inv(P);
//
//	object temp_joint = joint;
//	object nodeExistParentJoint = object();
//	string temp_parent_name;
//	int temp_parent_index;
//	while(true)
//	{
//		if(temp_joint.attr("parent") == object())
//		{
//			nodeExistParentJoint = object();
//			break;
//		}
//		else
//		{
//			temp_parent_name = XS(temp_joint.attr("parent").attr("name"));
////			temp_parent_index = XI(posture.attr("skeleton").attr("getElementIndex")(temp_parent_name));
//			temp_parent_index = XI(posture.attr("skeleton").attr("getJointIndex")(temp_parent_name));
//
//			if(_nodes[temp_parent_index] != NULL) 
//			{
//				nodeExistParentJoint = temp_joint.attr("parent");
//				break;
//			}
//			else
//			{
//				temp_joint = temp_joint.attr("parent");
//
//				object offset = temp_joint.attr("offset");
//				SE3 P = SE3(pyVec3_2_Vec3(offset));
//
//				string joint_name = XS(temp_joint.attr("name"));
//				object localSO3 = posture.attr("localRs")[joint_index];
//				SE3 R = pySO3_2_SE3(localSO3);
//
//				invLocalT = invLocalT * Inv(R);
//				invLocalT = invLocalT * Inv(P);
//			}
//		}
//	}
//
////	int len_joint_children = len(joint.attr("children")); 
//
////	if ( nodeExistParentJoint!=object() && len_joint_children > 0  &&
////		_config.attr("hasNode")(joint_name))
//	if ( nodeExistParentJoint!=object() && _config.attr("hasNode")(joint_name))
//	{
//		Node* pNode = _nodes[joint_index];
//		object cfgNode = _config.attr("getNode")(joint_name);
//
//		string parent_name = XS(nodeExistParentJoint.attr("name"));
////		int parent_index = XI(posture.attr("skeleton").attr("getElementIndex")(parent_name));
//		int parent_index = XI(posture.attr("skeleton").attr("getJointIndex")(parent_name));
//		Node* pParentNode = _nodes[parent_index];
//		object parentCfgNode = _config.attr("getNode")(parent_name);
//
//		object offset = cfgNode.attr("offset");
//		SE3 offsetT = SE3(pyVec3_2_Vec3(offset));
//
//		object parentOffset = parentCfgNode.attr("offset");
//		SE3 parentOffsetT = SE3(pyVec3_2_Vec3(parentOffset));
//
//		pParentNode->body.SetJoint(&pNode->joint, Inv(_boneTs[parent_index])*Inv(invLocalT));
//		pNode->body.SetJoint(&pNode->joint, Inv(_boneTs[joint_index]));
//		pNode->use_joint = true;
//	}
//
//	for( int i=0 ; i<len_joint_children; ++i)
//		_createJoint(joint.attr("children")[i], posture);
//}
//
//void VpControlModel::ignoreCollisionBtwnBodies()
//{
//	for( VpModel::NODES_ITOR it=_nodes.begin(); it!=_nodes.end(); ++it)
//	{
//		for( VpModel::NODES_ITOR it2=_nodes.begin(); it2!=_nodes.end(); ++it2)
//		{
//			Node* pNode0 = *it;
//			Node* pNode1 = *it2;
////			if(pNode0 && pNode1)
//				_pWorld->IgnoreCollision(&pNode0->body, &pNode1->body);
//		}
//	}	
//}
//
//void VpControlModel::addBodiesToWorld( const object& createPosture )
//{
////	object joint = createPosture.attr("skeleton").attr("root");
////	string root_name = XS(joint.attr("name"));
////	int root_index = XI(createPosture.attr("skeleton").attr("getElementIndex")(root_name));
////	vpBody* pRootBody = &_nodes[root_index]->body;
//	vpBody* pRootBody = &_nodes[0]->body;
//	_pWorld->AddBody(pRootBody);
//}
//
//void VpControlModel::update( const object& posture )
//{
//	object joint = posture.attr("skeleton").attr("root");
//	_updateJoint(joint, posture);
//}
//
//void VpControlModel::_updateJoint( const object& joint, const object& posture )
//{
//	int len_joint_children = len(joint.attr("children")); 
//	if (len_joint_children == 0 )
//		return;
//
//	SE3 invLocalT;
//
//	SE3 P = SE3(pyVec3_2_Vec3(joint.attr("offset")));
//
//	string joint_name = XS(joint.attr("name"));
////	int joint_index = XI(posture.attr("skeleton").attr("getElementIndex")(joint_name));
////	SE3 R = pySO3_2_SE3(posture.attr("getLocalR")(joint_index));
//	int joint_index = XI(posture.attr("skeleton").attr("getJointIndex")(joint_name));
//	SE3 R = pySO3_2_SE3(posture.attr("getJointOrientationLocal")(joint_index));
//
//	// parent      <--------->        child
//	// link     L1      L2      L3      L4
//	// L4_M =  P1*R1 * P2*R2 * P3*R3 * P4*R4  (forward kinematics matrix of L4)
//	// 으로 나타내지지만 여기에선 while loop를 썼기 때문에 back tracking이 어려워
//	// L4_M = Inv( Inv(R4)*Inv(P4) * Inv(R3)*Inv(P3) * ...)
//	// 으로 코딩했음.
//
//	invLocalT = invLocalT * Inv(R);
//	invLocalT = invLocalT * Inv(P);
//
//	object temp_joint = joint;
//	object nodeExistParentJoint = object();
//	string temp_parent_name;
//	int temp_parent_index;
//	while(true)
//	{
//		if(temp_joint.attr("parent") == object())
//		{
//			nodeExistParentJoint = object();
//			break;
//		}
//		else
//		{
//			temp_parent_name = XS(temp_joint.attr("parent").attr("name"));
////			temp_parent_index = XI(posture.attr("skeleton").attr("getElementIndex")(temp_parent_name));
//			temp_parent_index = XI(posture.attr("skeleton").attr("getJointIndex")(temp_parent_name));
//
//			if(_nodes[temp_parent_index] != NULL) 
//			{
//				nodeExistParentJoint = temp_joint.attr("parent");
//				break;
//			}
//			else
//			{
//				temp_joint = temp_joint.attr("parent");
//
//				object offset = temp_joint.attr("offset");
//				SE3 P = SE3(pyVec3_2_Vec3(offset));
//
//				string joint_name = XS(temp_joint.attr("name"));
//				object localSO3 = posture.attr("localRs")[joint_index];
//				SE3 R = pySO3_2_SE3(localSO3);
//
//				invLocalT = invLocalT * Inv(R);
//				invLocalT = invLocalT * Inv(P);
//			}
//		}
//	}
//
////	int len_joint_children = len(joint.attr("children")); 
//
////	if(len_joint_children > 0 && _config.attr("hasNode")(joint_name))
//	if(_config.attr("hasNode")(joint_name))
//	{
//		Node* pNode = _nodes[joint_index];
//
//		if(nodeExistParentJoint!=object())
//			pNode->joint.SetOrientation(R);
//		else
//			// root의 경우는 body를 직접 SetFrame() 해준다.
//			pNode->body.SetFrame(SE3(pyVec3_2_Vec3(posture.attr("rootPos")))*P*R*_boneTs[joint_index]);
//	}
//
//	for( int i=0 ; i<len_joint_children; ++i)
//		_updateJoint(joint.attr("children")[i], posture);
//}
//
//void VpControlModel::fixBody( int index )
//{
//	_nodes[index]->body.SetGround();
//}
//
//void VpControlModel::initializeHybridDynamics(bool floatingBase)
//{
//	int rootIndex = 0;
//	
//	for(int i=0; i<_nodes.size(); ++i)
//	{
//		if(i == 0)
//		{
//			if(floatingBase)
//				_nodes[i]->body.SetHybridDynamicsType(VP::DYNAMIC);
//			else
//				_nodes[i]->body.SetHybridDynamicsType(VP::KINEMATIC);
//		}
//		else
//			_nodes[i]->joint.SetHybridDynamicsType(VP::KINEMATIC);
//	}
//}
//
//void VpControlModel::solveHybridDynamics()
//{
//	_nodes[0]->body.GetSystem()->HybridDynamics();	
//}
//
//bp::list VpControlModel::getDOFPositions()
//{
////	static numeric::array rootFrame( make_tuple(make_tuple(1.,0.,0.,0.), make_tuple(0.,1.,0.,0.), make_tuple(0.,0.,1.,0.), make_tuple(0.,0.,0.,1.)) );
////
////	bp::list ls = getInternalJointOrientationsLocal();
////	SE3_2_pySE3(_nodes[0]->body.GetFrame() * Inv(_boneTs[0]), rootFrame);
////	ls.insert(0, rootFrame );
////	return ls;
//
//	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
//	static numeric::array O(make_tuple(0.,0.,0.));
//	static SE3 rootFrame;
//
//	object pyR = I.copy();
//	object pyV = O.copy();
//
//	bp::list ls = getInternalJointOrientationsLocal();
//
//	rootFrame = _nodes[0]->body.GetFrame() * Inv(_boneTs[0]);
//
//	Vec3_2_pyVec3(rootFrame.GetPosition(), pyV);
//	SE3_2_pySO3(rootFrame, pyR);
//
//	ls.insert(0, make_tuple(pyV, pyR));
//	return ls;
//}
//
//bp::list VpControlModel::getDOFVelocities()
//{
//	static numeric::array rootGenVel(make_tuple(0.,0.,0.,0.,0.,0.));
//	
//	rootGenVel.slice(0,3) = getJointVelocityGlobal(0);
////	rootGenVel.slice(3,6) = getJointAngVelocityGlobal(0);
//	rootGenVel.slice(3,6) = getJointAngVelocityLocal(0);
//
//	bp::list ls = getInternalJointAngVelocitiesLocal();
//	ls.insert(0, rootGenVel);
//	return ls;
//}
//
//bp::list VpControlModel::getDOFAccelerations()
//{
//	static numeric::array rootGenAcc(make_tuple(0.,0.,0.,0.,0.,0.));
//	
//	rootGenAcc.slice(0,3) = getJointAccelerationGlobal(0);
////	rootGenAcc.slice(3,6) = getJointAngAccelerationGlobal(0);
//	rootGenAcc.slice(3,6) = getJointAngAccelerationLocal(0);
//
//	bp::list ls = getInternalJointAngAccelerationsLocal();
//
//	ls.insert(0, rootGenAcc);
//	return ls;
//}
//
//bp::list VpControlModel::getDOFAxeses()
//{
//	static numeric::array rootAxeses( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.),
//										make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
//
//	numeric::array rootAxes = transpose_pySO3(((numeric::array)getJointOrientationGlobal(0)));
//	rootAxeses[3] = rootAxes[0];
//	rootAxeses[4] = rootAxes[1];
//	rootAxeses[5] = rootAxes[2];
//
//	bp::list ls = getInternalJointOrientationsGlobal();
//	for(int i=0; i<len(ls); ++i)
//		ls[i] = transpose_pySO3(((numeric::array)ls[i]));
//
//	ls.insert(0, rootAxeses);
//	return ls;
//}
//
//void VpControlModel::setDOFAccelerations( const bp::list& dofaccs)
//{
//	static numeric::array O(make_tuple(0.,0.,0.));
//
//	setJointAccelerationGlobal(0, dofaccs[0].slice(0,3));
//
////	setJointAngAccelerationGlobal(0, dofaccs[0].slice(3,6));
//	setJointAngAccelerationLocal(0, dofaccs[0].slice(3,6));
//
//	setInternalJointAngAccelerationsLocal( ((bp::list)dofaccs.slice(1,_)) );
//}
//
//boost::python::object VpControlModel::getJointOrientationLocal( int index )
//{
//	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
//
//	if(index == 0)
//		return getJointOrientationGlobal(index);
//	else
//	{
//		object pyR = I.copy();
//		SE3_2_pySO3(_nodes[index]->joint.GetOrientation(), pyR);
//		return pyR;
//	}
//}
//
//boost::python::object VpControlModel::getJointAngVelocityLocal( int index )
//{
//	static numeric::array O(make_tuple(0.,0.,0.));
//	object pyV = O.copy();
//
//	if(index == 0)
//	{
//		static se3 genVelBodyLocal, genVelJointLocal;
//
//		genVelBodyLocal = _nodes[index]->body.GetGenVelocityLocal();
//		genVelJointLocal = InvAd(Inv(_boneTs[index]), genVelBodyLocal);
////		genVelJointLocal = Ad(_boneTs[index], genVelBodyLocal);	// 윗 라인과 같은 역할
//		pyV[0] = genVelJointLocal[0];
//		pyV[1] = genVelJointLocal[1];
//		pyV[2] = genVelJointLocal[2]; 
//	}
//	else
//		Vec3_2_pyVec3(_nodes[index]->joint.GetVelocity(), pyV);
//	
//	return pyV;
//}
//
//boost::python::object VpControlModel::getJointAngAccelerationLocal( int index )
//{
//	static numeric::array O(make_tuple(0.,0.,0.));
//	object pyV = O.copy();
//
//	if(index == 0)
//	{
//		static se3 genAccBodyLocal, genAccJointLocal;
//
//		genAccBodyLocal = _nodes[index]->body.GetGenAccelerationLocal();
//		genAccJointLocal = InvAd(Inv(_boneTs[index]), genAccBodyLocal);
//		pyV[0] = genAccJointLocal[0]; 
//		pyV[1] = genAccJointLocal[1];
//		pyV[2] = genAccJointLocal[2]; 
//	}
//	else
//		Vec3_2_pyVec3(_nodes[index]->joint.GetAcceleration(), pyV);
//	
//	return pyV;
//}
//
//object VpControlModel::getJointPositionGlobal( int index )
//{
//	static numeric::array O(make_tuple(0.,0.,0.));
//	static SE3 bodyFrame;
//	object pyV = O.copy();
//
//	// body frame에 Inv(boneT)로 원래 joint 위치 찾는다.
//	bodyFrame = _nodes[index]->body.GetFrame();
//	Vec3_2_pyVec3((bodyFrame * Inv(_boneTs[index])).GetPosition(), pyV);
//	return pyV;
//
////	if(!_nodes[index])	// 말단인 경우 parent joint frame을 찾아 offset만큼 transformation 시킨다.
////	{
////		static SE3 parentJointFrame;
////		static Vec3 offset;
//////		int parent = XI(_skeleton.attr("getParentIndex")(index));
////		int parent = XI(_skeleton.attr("getParentJointIndex")(index));
////		parentJointFrame = _nodes[parent]->body.GetFrame() * Inv(_boneTs[parent]);
////		offset = pyVec3_2_Vec3(_skeleton.attr("getOffset")(index));
////		Vec3_2_pyVec3(parentJointFrame * offset, pyV);
////	}
////	else	// 말단이 아닌 경우 body frame에 Inv(boneT)로 원래 joint 위치 찾는다.
////	{
////		static SE3 bodyFrame;
////		bodyFrame = _nodes[index]->body.GetFrame();
////		Vec3_2_pyVec3((bodyFrame * Inv(_boneTs[index])).GetPosition(), pyV);
////	}
////	return pyV;
//}
//object VpControlModel::getJointVelocityGlobal( int index )
//{
//	static numeric::array O(make_tuple(0.,0.,0.));
//	object pyV = O.copy();
//
//	Vec3_2_pyVec3(getBodyVelocityGlobal(index, Inv(_boneTs[index]).GetPosition()), pyV);
//	return pyV;
//}
//
//object VpControlModel::getJointAccelerationGlobal( int index )
//{
//	static numeric::array O(make_tuple(0.,0.,0.));
//	object pyV = O.copy();
//
//	Vec3_2_pyVec3(getBodyAccelerationGlobal(index, &(Inv(_boneTs[index]).GetPosition())), pyV);
//	return pyV;
//}
//
//boost::python::object VpControlModel::getJointOrientationGlobal( int index )
//{
//	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
//	static SE3 bodyFrame;
//	object pyR = I.copy();
//
//	// body frame에 Inv(boneT)로 원래 joint frame 구한다
//	bodyFrame = _nodes[index]->body.GetFrame();
//	SE3_2_pySO3(bodyFrame * Inv(_boneTs[index]), pyR);
//	return pyR;
//}
//
//boost::python::object VpControlModel::getJointAngVelocityGlobal( int index )
//{
//	return getBodyAngVelocityGlobal(index);
//
////	static numeric::array O(make_tuple(0.,0.,0.));
////	static Vec3 angVel, parentAngVel;
////	object pyV = O.copy();
////
////	angVel = _nodes[index]->body.GetAngVelocity();
////
////	int parentIndex = getParentIndex(index);
////	if(parentIndex==-1)
////		parentAngVel = Vec3(0.,0.,0.);
////	else
////		parentAngVel = _nodes[parentIndex]->body.GetAngVelocity();
////
////	Vec3_2_pyVec3(angVel - parentAngVel, pyV);
////	return pyV;
//}
//
//boost::python::object VpControlModel::getJointAngAccelerationGlobal( int index )
//{
//	return getBodyAngAccelerationGlobal(index);
//}
//
//boost::python::object VpControlModel::getJointFrame( int index )
//{
//	static numeric::array frame( make_tuple(make_tuple(1.,0.,0.,0.), make_tuple(0.,1.,0.,0.), make_tuple(0.,0.,1.,0.), make_tuple(0.,0.,0.,1.)) );
//	
//	SE3_2_pySE3(_nodes[index]->body.GetFrame() * Inv(_boneTs[0]), frame);
//	return frame;
//}
//
//bp::list VpControlModel::getJointOrientationsLocal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getJointOrientationLocal(i));
//	return ls;
//}
//
//bp::list VpControlModel::getJointAngVelocitiesLocal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getJointAngVelocityLocal(i));
//	return ls;
//}
//
//bp::list VpControlModel::getJointAngAccelerationsLocal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getJointAngAccelerationLocal(i));
//	return ls;
//}
//
//bp::list VpControlModel::getJointPositionsGlobal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getJointPositionGlobal(i));
//	return ls;
//}
//
//bp::list VpControlModel::getJointVelocitiesGlobal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getJointVelocityGlobal(i));
//	return ls;
//}
//
//bp::list VpControlModel::getJointAccelerationsGlobal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getJointAccelerationGlobal(i));
//	return ls;
//}
//
//bp::list VpControlModel::getJointOrientationsGlobal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getJointOrientationGlobal(i));
//	return ls;
//}
//
//bp::list VpControlModel::getJointAngVelocitiesGlobal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getJointAngVelocityGlobal(i));
//	return ls;
//}
//
//bp::list VpControlModel::getJointAngAccelerationsGlobal()
//{
//	bp::list ls;
//	for(int i=0; i<_nodes.size(); ++i)
//		ls.append(getJointAngAccelerationGlobal(i));
//	return ls;
//}
//
//bp::list VpControlModel::getInternalJointOrientationsLocal()
//{
//	bp::list ls;
//	for(int i=1; i<_nodes.size(); ++i)
//		ls.append(getJointOrientationLocal(i));
//	return ls;
//}
//
//bp::list VpControlModel::getInternalJointAngVelocitiesLocal()
//{
//	bp::list ls;
//	for(int i=1; i<_nodes.size(); ++i)
//		ls.append(getJointAngVelocityLocal(i));
//	return ls;
//}
//
//bp::list VpControlModel::getInternalJointAngAccelerationsLocal()
//{
//	bp::list ls;
//	for(int i=1; i<_nodes.size(); ++i)
//		ls.append(getJointAngAccelerationLocal(i));
//	return ls;
//}
//
//bp::list VpControlModel::getInternalJointPositionsGlobal()
//{
//	bp::list ls;
////	for(int i=1; i<_jointElementIndexes.size(); ++i)
//	for(int i=1; i<_nodes.size(); ++i)
//		ls.append(getJointPositionGlobal(i));
//	return ls;
//}
//
//bp::list VpControlModel::getInternalJointOrientationsGlobal()
//{
//	bp::list ls;
//	for(int i=1; i<_nodes.size(); ++i)
//		ls.append(getJointOrientationGlobal(i));
//	return ls;
//}
//void VpControlModel::setJointAngVelocityLocal( int index, const object& angvel )
//{
//	if(index == 0)
//	{
//		static se3 genVelBodyLocal, genVelJointLocal;
//		
//		genVelBodyLocal = _nodes[index]->body.GetGenVelocityLocal();
//
//		genVelJointLocal = InvAd(Inv(_boneTs[index]), genVelBodyLocal);
//		genVelJointLocal[0] = XD(angvel[0]);
//		genVelJointLocal[1] = XD(angvel[1]);
//		genVelJointLocal[2] = XD(angvel[2]); 
//
//		genVelBodyLocal = Ad(Inv(_boneTs[index]), genVelJointLocal);;
//		_nodes[index]->body.SetGenVelocityLocal(genVelBodyLocal);
//	}
//	else
//		_nodes[index]->joint.SetVelocity(pyVec3_2_Vec3(angvel));
//}
//
//void VpControlModel::setJointAngAccelerationLocal( int index, const object& angacc )
//{
//	if(index == 0)
//	{
//		static se3 genAccBodyLocal, genAccJointLocal;
//		
//		genAccBodyLocal = _nodes[index]->body.GetGenAccelerationLocal();
//
//		genAccJointLocal = InvAd(Inv(_boneTs[index]), genAccBodyLocal);
//		genAccJointLocal[0] = XD(angacc[0]);
//		genAccJointLocal[1] = XD(angacc[1]);
//		genAccJointLocal[2] = XD(angacc[2]); 
//
//		genAccBodyLocal = Ad(Inv(_boneTs[index]), genAccJointLocal);;
//		_nodes[index]->body.SetGenAccelerationLocal(genAccBodyLocal);
//	}
//	else
//		_nodes[index]->joint.SetAcceleration(pyVec3_2_Vec3(angacc));
//}
//
//void VpControlModel::setJointAccelerationGlobal( int index, const object& acc )
//{
//	if(index == 0)
//		setBodyAccelerationGlobal(index, pyVec3_2_Vec3(acc), &(Inv(_boneTs[index]).GetPosition()));
//	else
//		cout << "setJointAccelerationGlobal() : not completely implemented" << endl;
//}
//
//void VpControlModel::setJointAngAccelerationGlobal( int index, const object& angacc )
//{
//	setBodyAngAccelerationGlobal(index, angacc);
//}
//
//void VpControlModel::setJointAngAccelerationsLocal( const bp::list& angaccs )
//{
//	for(int i=0; i<_nodes.size(); ++i)
//		setJointAngAccelerationLocal(i, angaccs[i]);
//}
//
//void VpControlModel::setInternalJointAngAccelerationsLocal( const bp::list& angaccs )
//{
//	for(int i=1; i<_nodes.size(); ++i)
//		_nodes[i]->joint.SetAcceleration(pyVec3_2_Vec3(angaccs[i-1]));
//}
//
//boost::python::object VpControlModel::getJointTorqueLocal( int index )
//{
//	static numeric::array O(make_tuple(0.,0.,0.));
//	object pyV = O.copy();
//
//	if(index==0) return pyV;
//
//	Vec3_2_pyVec3(_nodes[index]->joint.GetTorque(), pyV);
//	return pyV;
//}
//
//bp::list VpControlModel::getInternalJointTorquesLocal()
//{
//	bp::list ls;
////	for(int i=1; i<_jointElementIndexes.size(); ++i)
//	for(int i=1; i<_nodes.size(); ++i)
//		ls.append(getJointTorqueLocal(i));
//	return ls;
//}
//
//void VpControlModel::setJointTorqueLocal( int index, const object& torque )
//{
////	int index = _jointElementIndexes[jointIndex];
//	_nodes[index]->joint.SetTorque(pyVec3_2_Vec3(torque));
//}
//
//void VpControlModel::setInternalJointTorquesLocal( const bp::list& torques )
//{
////	int index;
////	for(int i=1; i<_jointElementIndexes.size(); ++i)
////	{
////		index = _jointElementIndexes[i];
////		_nodes[index]->joint.SetTorque(pyVec3_2_Vec3(torques[i]));
////	}
//	for(int i=1; i<_nodes.size(); ++i)
//		_nodes[i]->joint.SetTorque(pyVec3_2_Vec3(torques[i-1]));
//}
//
//
//void VpControlModel::applyBodyGenForceGlobal( int index, const object& torque, const object& force, const object& positionLocal/*=object()*/ )
//{
//	static Vec3 zero(0,0,0);
//	if(positionLocal==object())
//		_nodes[index]->body.ApplyGlobalForce(dse3(XD(torque[0]), XD(torque[1]), XD(torque[2]), XD(force[0]), XD(force[1]), XD(force[2])), zero);
//	else
//		_nodes[index]->body.ApplyGlobalForce(dse3(XD(torque[0]), XD(torque[1]), XD(torque[2]), XD(force[0]), XD(force[1]), XD(force[2])), pyVec3_2_Vec3(positionLocal));
//}
//
//void VpControlModel::applyBodyForceGlobal( int index, const object& force, const object& positionLocal/*=object()*/ )
//{
//	static Vec3 zero(0,0,0);
//	if(positionLocal==object())
//		_nodes[index]->body.ApplyGlobalForce(dse3(0.,0.,0., XD(force[0]), XD(force[1]), XD(force[2])), zero);
//	else
//		_nodes[index]->body.ApplyGlobalForce(dse3(0.,0.,0., XD(force[0]), XD(force[1]), XD(force[2])), pyVec3_2_Vec3(positionLocal));
//}
//
//void VpControlModel::applyBodyTorqueGlobal( int index, const object& torque )
//{
//	static Vec3 zero(0,0,0);
//	_nodes[index]->body.ApplyGlobalForce(dse3(XD(torque[0]), XD(torque[1]), XD(torque[2]), 0.,0.,0.), zero);
//}
//
//object VpControlModel::getBodyForceLocal( int index )
//{
//	static dse3 genForce;
//	static numeric::array O(make_tuple(0.,0.,0.));
//	object pyV = O.copy();
//
//	genForce = _nodes[index]->body.GetForce();
//	pyV[0] = genForce[3];
//	pyV[1] = genForce[4];
//	pyV[2] = genForce[5];
//	return pyV;
//
//}
//
//object VpControlModel::getBodyNetForceLocal( int index )
//{
//	static dse3 genForce;
//	static numeric::array O(make_tuple(0.,0.,0.));
//	object pyV = O.copy();
//
//	genForce = _nodes[index]->body.GetNetForce();
//	pyV[0] = genForce[3];
//	pyV[1] = genForce[4];
//	pyV[2] = genForce[5];
//	return pyV;
//}
//
//object VpControlModel::getBodyGravityForceLocal( int index )
//{
//	static dse3 genForce;
//	static numeric::array O(make_tuple(0.,0.,0.));
//	object pyV = O.copy();
//
//	genForce = _nodes[index]->body.GetGravityForce();
//	pyV[0] = genForce[3];
//	pyV[1] = genForce[4];
//	pyV[2] = genForce[5];
//	return pyV;
//}
//
//

