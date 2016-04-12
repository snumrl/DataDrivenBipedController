#include <VP/vphysics.h>
#include <fstream>
#include <list>
#include <map>
#include <string>

void vpWorld::report(ostream &out)
{
	out << "WORLD\tBODY\t";
	for ( int i = 0; i < m_pBody.size(); i++ ) out << m_pBody[i]->m_szName << " ";
	out << endl;

	out << "\t\tJOINT\t";
	for ( int i = 0; i < m_pJoint.size(); i++ ) out << m_pJoint[i]->m_szName << " ";
	out << endl;

	for ( int i = 0; i < m_pSystem.size(); i++ )
	{
		out << "SYSTEM" << i;
		if ( m_pSystem[i]->m_pRoot->m_bIsGround ) out << "\tGROUND\t";
		else out << "\tROOT\t";
		out << m_pSystem[i]->m_pRoot->m_szName << endl << "\t\tBODY\t";
		for ( int j = 0; j < m_pSystem[i]->m_pBody.size(); j++ ) 
			out << m_pSystem[i]->m_pBody[j]->m_szName << " ";
			
		out << endl << "\t\tJOINT\t";
		for ( int j = 0; j < m_pSystem[i]->m_pJoint.size(); j++ ) 
			out << m_pSystem[i]->m_pJoint[j]->m_szName << " ";
		out << endl;
	}

	for ( int i = 0; i < m_pBody.size(); i++ )
	{
		out << m_pBody[i]->m_szName << "\tJOINT\t";
		for ( int j = 0; j < m_pBody[i]->m_pJoint.size(); j++ ) out << m_pBody[i]->m_pJoint[j]->m_szName << " ";
		out << endl;
	}

	for ( int i = 0; i < m_pJoint.size(); i++ )
	{
		out << m_pJoint[i]->m_szName << "\tBODY\t" << m_pJoint[i]->m_pLeftBody->m_szName << " " << m_pJoint[i]->m_pRightBody->m_szName << endl;
		if ( m_pJoint[i]->m_pParentJoint ) out << "\t\tPARENT\t" << m_pJoint[i]->m_pParentJoint->m_szName << endl;
		if ( m_pJoint[i]->m_pChildJoints.size() )
		{
			out << "\t\tCHILD\t";
			for ( int j = 0; j < m_pJoint[i]->m_pChildJoints.size(); j++ ) out << m_pJoint[i]->m_pChildJoints[j]->m_szName << " ";
			out << endl;
		}
	}
}

ostream &operator << (ostream &os, const vpBody *pBody)
{
	if ( pBody->IsGround() ) os << "<ground";
	else os << "<body";

	os << " id = '" << pBody->GetID() << "'";

	if ( !pBody->m_szName.empty() ) os << " name = '" << pBody->m_szName << "'";
	if ( !pBody->IsCollidable() ) os << " collidable = 'true'";
	
	for ( int i = 0; i < pBody->GetWorld()->GetNumMaterial(); i++ )
		if ( pBody->GetMaterial() == pBody->GetWorld()->GetMaterial(i) )
			os << " material_id = '" << i << "'";

	if ( pBody->IsSetInertia()) os << " inertia = '" << pBody->GetInertia().GetMass() << " " << pBody->GetInertia().GetDiag()[0] << " " << pBody->GetInertia().GetDiag()[1] << " " << pBody->GetInertia().GetDiag()[2] << "'";
	os << " frame = '";
	for ( int i = 0; i < 11; i++ ) os << pBody->GetFrame()[i] << " ";
	os << pBody->GetFrame()[11] << "'>" << endl;
	
	for ( int i = 0; i < pBody->GetNumGeometry(); i++ )
	{
		char type;
		scalar val[3];
		pBody->GetGeometry(i)->GetShape(&type, NULL);
		switch ( type )
		{
		case 'S':
			pBody->GetGeometry(i)->GetShape(&type, val);
			os << "<sphere radius = '" << val[0] << "'";
			break;
		case 'B':
			pBody->GetGeometry(i)->GetShape(&type, val);
			os << "<box size = '" << val[0] << " " << val[1] << " " << val[2] << "'";
			break;
		case 'C':
			pBody->GetGeometry(i)->GetShape(&type, val);
			os << "<capsule radius = '" << val[0] << "' height = '" << val[1] << "'";
			break;
		}

		os << " localframe = '";
		for ( int j = 0; j < 11; j++ ) os << pBody->GetGeometry(i)->GetLocalFrame()[j] << " ";
		os << pBody->GetGeometry(i)->GetLocalFrame()[11] << "'/>" << endl;
	}
	
	if ( pBody->IsGround() ) os << "</ground>" << endl;
	else os << "</body>" << endl;

	return os;
}

void vpJoint::streamOut(ostream &os) const
{
	assert(m_pLeftBody && m_pRightBody && "vpJoint::streamOut() -> world is not initialized or the joint is not attached to the body");
	if ( !m_szName.empty() ) os << " name = '" << m_szName << "'";
	os << " left_body_id = '" << m_pLeftBody->GetID() << "' right_body_id = '" << m_pRightBody->GetID();
	os << "' left_frame = '";
	for ( int i = 0; i < 11; i++ ) os << m_sLeftBodyFrame[i] << " ";
	os << m_sLeftBodyFrame[11] << "' right_frame = '";
	for ( int i = 0; i < 11; i++ ) os << m_sRightBodyFrame[i] << " ";
	os << m_sRightBodyFrame[11] << "'/>" << endl;
}

void vpRJoint::streamOut(ostream &os) const
{
	os << "<rotational_joint axis = '" << GetAxis()[0] << " " << GetAxis()[1] << " " << GetAxis()[2];
	os << "' q = '" << GetAngle() << "' dq = '" << GetVelocity();
	os << "' q_i = '" << GetInitialAngle() << "' k = '" << GetElasticity() << "' c = '" << GetDamping() << "'";
	if ( IsEnabledUpperLimit() ) os << " qul = '" << GetUpperLimit() << "'";
	if ( IsEnabledLowerLimit() ) os << " qll = '" << GetLowerLimit() << "'";
	vpJoint::streamOut(os);
}

void vpPJoint::streamOut(ostream &os) const
{
	os << "<prismatic_joint direction = '" << GetDirection()[0] << " " << GetDirection()[1] << " " << GetDirection()[2];
	os << "' q = '" << GetDisplacement() << "' dq = '" << GetVelocity();
	os << "' q_i = '" << GetInitialDisplacement() << "' k = '" << GetElasticity() << "' c = '" << GetDamping() << "'";
	if ( IsEnabledUpperLimit() ) os << " qul = '" << GetUpperLimit() << "'";
	if ( IsEnabledLowerLimit() ) os << " qll = '" << GetLowerLimit() << "'";
	vpJoint::streamOut(os);
}

void vpUJoint::streamOut(ostream &os) const
{
	os << "<universal_joint axis0 = '" << GetAxis(0)[0] << " " << GetAxis(0)[1] << " " << GetAxis(0)[2];
	os << "' axis1 = '" << GetAxis(1)[0] << " " << GetAxis(1)[1] << " " << GetAxis(1)[2];
	os << "' q0 = '" << GetAngle(0) << "' q1 = '" << GetAngle(1) << "' dq0 = '" << GetVelocity(0) << "' dq1 = '" << GetVelocity(1);
	os << "' q_i0 = '" << GetInitialAngle(0) << "' q_i1 = '" << GetInitialAngle(1);
	os << "' k0 = '" << GetElasticity(0) << "' k1 = '" << GetElasticity(1) << "' c0 = '" << GetDamping(0) << "' c1 = '" << GetDamping(1) << "'";
	if ( IsEnabledUpperLimit(0) ) os << " qul0 = '" << GetUpperLimit(0) << "'";
	if ( IsEnabledUpperLimit(1) ) os << " qul1 = '" << GetUpperLimit(1) << "'";
	if ( IsEnabledLowerLimit(0) ) os << " qll0 = '" << GetLowerLimit(0) << "'";
	if ( IsEnabledLowerLimit(1) ) os << " qll1 = '" << GetLowerLimit(1) << "'";
	vpJoint::streamOut(os);
}

void vpSJoint::streamOut(ostream &os) const
{
	os << "<sliding_joint axis = '" << GetAxis()[0] << " " << GetAxis()[1] << " " << GetAxis()[2];
	os << "' q = '" << GetAngle() << "' x = '" << GetDisplacement() << "' dq = '" << GetAngularVelocity() << "' dx = '" << GetVelocity();
	os << "' q_i = '" << GetInitialAngle() << "' x_i = '" << GetInitialDisplacement();
	os << "' kr = '" << GetRotationalElasticity() << "' ks = '" << GetSlidingElasticity() << "' cr = '" << GetRotationalDamping() << "' cs = '" << GetSlidingDamping() << "'";
	if ( IsEnabledRotationalUpperLimit() ) os << " qulr = '" << GetRotationalUpperLimit() << "'";
	if ( IsEnabledSlidingUpperLimit() ) os << " quls = '" << GetSlidingUpperLimit() << "'";
	if ( IsEnabledRotationalLowerLimit() ) os << " qllr = '" << GetRotationalLowerLimit() << "'";
	if ( IsEnabledSlidingLowerLimit() ) os << " qlls = '" << GetSlidingLowerLimit() << "'";
	vpJoint::streamOut(os);
}

void vpBJoint::streamOut(ostream &os) const
{
	Axis q = LogR(GetOrientation());
	os << "<ball_joint orientation = '" << q[0] << " " << q[1] << " " << q[2];
	os << "' K = '" << m_sSpringCoef.GetMass() << " " << m_sSpringCoef.GetDiag()[0] << " " << m_sSpringCoef.GetDiag()[1] << " " << m_sSpringCoef.GetDiag()[2];
	os << "' C = '" << m_sDampingCoef.GetMass() << " " << m_sDampingCoef.GetDiag()[0] << " " << m_sDampingCoef.GetDiag()[1] << " " << m_sDampingCoef.GetDiag()[2];
	os << "' initial_orientation = '";
	for ( int i = 0; i < 11; i++ ) os << m_sTi[i] << " ";
	os << m_sTi[11] << "'";

	vpJoint::streamOut(os);
}

void vpWJoint::streamOut(ostream &os) const
{
	os << "<welding_joint";
	vpJoint::streamOut(os);
}

void vp1DOFJoint::streamOut(ostream &os) const
{
	os << "<onedof_joint";
	vpJoint::streamOut(os);
}

void vpNDOFJoint::streamOut(ostream &os) const
{
	os << "<ndof_joint";
	vpJoint::streamOut(os);
}

void vpHelixJoint::streamOut(ostream &os) const
{
	os << "<helix_joint radius = '" << GetRadius() << "' pitch = '" << GetPitch();
	os << "' q = '" << GetPosition() << "' dq = '" << GetVelocity();
	os << "' q_i = '" << GetInitialPosition() << "' k = '" << GetElasticity() << "' c = '" << GetDamping() << "'";
	if ( IsEnabledUpperLimit() ) os << " qul = '" << GetUpperLimit() << "'";
	if ( IsEnabledLowerLimit() ) os << " qll = '" << GetLowerLimit() << "'";
	vpJoint::streamOut(os);
}

ostream &operator << (ostream &os, const vpJoint *pJ)
{
	pJ->streamOut(os);
	return os;
}

ostream &operator << (ostream &os, const vpWorld &world)
{
	int i;

	os << "<?xml version = '1.0'?>" << endl;
	os << "<!-- Virtual Physics v0.82 -->" << endl; 
	os << "<world gravity = '" << world.GetGravity()[0] << " " << world.GetGravity()[1] << " " << world.GetGravity()[2];
	os << "' time_step = '" << world.GetTimeStep() << "' integrator = '" << (world.IntegrateDynamics == &vpSystem::IntegrateDynamicsRK4 ? "RK4" : "Euler");
	os << "' global_frame = '";
	for ( int i = 0; i < 11; i++ ) os << world.GetGlobalFrame()[i] << " ";
	os << world.GetGlobalFrame()[11] << "' ";

	for ( i = 0; i < world.GetNumMaterial(); i++ )
	{
		const vpMaterial *pMaterial = world.GetMaterial(i);		
		os << "<material id = '" << i << "' density = '" << pMaterial->GetDensity();
		if ( !pMaterial->m_szName.empty() ) os << " name = '" << pMaterial->m_szName;
		os << "' restitution = '" << pMaterial->GetRestitution() << "' static_friction = '" << pMaterial->GetStaticFriction() << "' dynamic_friction = '" << pMaterial->GetDynamicFriction() << "' spinning_friction = '" << pMaterial->GetSpinningFriction() << "'/>" << endl;
	}

	for ( i = 0; i < world.m_pBody.size(); i++ ) os << world.m_pBody[i];
	
	for ( i = 0; i < world.m_pJoint.size(); i++ ) os << world.m_pJoint[i];
	
	//vpBodyPairSet::const_iterator itor = world.m_pCollisionDetector->m_sNonCollidablePair.begin();
	//while ( itor != world.m_pCollisionDetector->m_sNonCollidablePair.end() )
	//	os << "<ignore_collision left_body_id = '" << itor->pLeftBody->GetID() << "' right_body_id = '" << itor->pRightBody->GetID() << "'/>" << endl;

	os << "</world>" << endl;

	return os;
}

typedef list<string> string_list;

struct xml_node
{
	string						 name;
	xml_node					*parent;
	list<xml_node *>			 child;
	map<string, string_list>	 attrib;
};

ostream &operator << (ostream &os, const xml_node &node)
{
	os << "<" << node.name;

	map <string, string_list>::const_iterator attrib_itor = node.attrib.begin();

	while ( attrib_itor != node.attrib.end() )
	{
		os << " " << attrib_itor->first << " = '";
		list<string>::const_iterator attrib_value_itor = attrib_itor->second.begin();
		while ( attrib_value_itor != attrib_itor->second.end() )
		{
			os << *attrib_value_itor << " ";
			attrib_value_itor++;
		}
		os << "' ";

		attrib_itor++;
	}
	os << ">" << endl;

	list<xml_node *>::const_iterator child_itor = node.child.begin();
	while ( child_itor != node.child.end() )
	{
		os << **child_itor;
		child_itor++;
	}
	
	os << "</" << node.name << ">" << endl;
	return os;
}

istream &operator >> (istream &is, xml_node &root)
{
	int state = 0x00;
	char c;
	string buf, attrib_name;
	xml_node *current_node, *parent_node;
	
	current_node = &root;
	parent_node = NULL;

	while ( is.good() )
	{
		switch ( state )
		{
		case 0x00:			// start
			is >> c;
			if ( !is.good() ) return is;
			if ( c == '<' ) state = 0x01;
			break;
		case 0x01:			// open node
			is >> buf;
			if ( buf[0] == '/' )
			{
				state = 0x04;
			} else if ( buf[0] == '?' )
			{
				char tmp[1024];
				is.getline(tmp, 1024);
				state = 0x00;
			} else if ( buf[0] == '!' )
			{
				if ( buf[1] == '-' && buf[2] == '-' )
				{
					if ( buf.length() >= 6 && buf[buf.length() - 3] == '-' && buf[buf.length() - 2] == '-' && buf[buf.length() - 1] == '>' ) state = 0x00;
					else state = 0x05;
				}
			} else
			{
				parent_node = current_node;
				current_node = new xml_node;
				current_node->name = buf;
				parent_node->child.push_back(current_node);
				current_node->parent = parent_node;
				state = 0x03;
			}
			break;
		case 0x03:			// attribute
			is >> buf;
			if ( buf == ">" )
			{
				state = 0x00;
				break;
			}
			
			attrib_name = buf;
			current_node->attrib[attrib_name];

			is >> buf;		// '='
			is >> c;		// '''
			while ( true )
			{
				is >> buf;
				if ( buf[buf.length() - 1] == '\'')
				{
					buf.erase(buf.length() - 1, 1);
					current_node->attrib[attrib_name].push_back(buf);
					break;
				} else if ( buf[buf.length() - 1] == '>')
				{
					if ( buf[buf.length() - 2] == '/' )
					{
						buf.erase(buf.length() - 3, 3);
						current_node->attrib[attrib_name].push_back(buf);
						state = 0x04;
					} else
					{
						buf.erase(buf.length() - 2, 2);
						current_node->attrib[attrib_name].push_back(buf);
						state = 0x00;
					}
					break;
				}
				current_node->attrib[attrib_name].push_back(buf);
			}
			break;
		case 0x04:			// close node
			current_node = current_node->parent;
			parent_node = current_node->parent;
			state = 0x00;
			break;
		case 0x05:			 // comment
			while ( true )
			{
				is >> buf;
				if ( buf.length() >= 3 && buf[buf.length() - 3] == '-' && buf[buf.length() - 2] == '-' && buf[buf.length() - 1] == '>' ) break;
			}
			state = 0x00;
			break;
		}
	}

	return is;
}

Vec3 strlist2Vec3(const string_list &val)
{
	int idx = 0;
	Vec3 re;

	string_list::const_iterator itor = val.begin();

	while ( idx < 3 && itor != val.end() )
		re[idx++] = (scalar)atof((itor++)->c_str());

	return re;
}

Inertia strlist2Inertia(const string_list &val)
{
	int idx = 0;
	scalar inertia[4];

	string_list::const_iterator itor = val.begin();

	while ( idx < 4 && itor != val.end() )
		inertia[idx++] = (scalar)atof((*itor++).c_str());

	return Inertia(inertia[0], inertia[1], inertia[2], inertia[3]);
}

SE3 strlist2SE3(const string_list &val)
{
	int idx = 0;
	SE3 T;

	string_list::const_iterator itor = val.begin();

	while ( idx < 12 && itor != val.end() ) T[idx++] = (scalar)atof((itor++)->c_str());
	
	return T;
}

scalar strlist2scalar(const string_list &val)
{
	return (scalar)atof(val.begin()->c_str());
}

int strlist2int(const string_list &val)
{
	return atoi(val.begin()->c_str());
}

istream &operator >> (istream &is, vpWorld &world)
{
	xml_node *node, *current;
	xml_node root;
	string_list val;
	_array<vpBody *> body_array;
	_array<vpMaterial *> material_array;

	if ( !is.good() )
	{
		cerr << "invalid input stream" << endl;
		return is;
	}

	is >> root;
	
	node = *root.child.begin();

	if ( node->name == "world" )
	{
		world.Clear();

		world.SetGravity(strlist2Vec3(node->attrib["gravity"]));
		world.SetTimeStep(strlist2scalar(node->attrib["time_step"]));
		
		if ( *node->attrib["integrator"].begin() == "RK4" ) world.SetIntegrator(VP::RK4);
		else if ( *node->attrib["integrator"].begin() == "Euler" ) world.SetIntegrator(VP::EULER);

		world.SetGlobalFrame(strlist2SE3(node->attrib["global_frame"]));

		list<xml_node *>::const_iterator itor = node->child.begin();
		
		while ( itor != node->child.end() )
		{
			current = *itor;

			if ( current->name == "material" )
			{
				vpMaterial *pMaterial = new vpMaterial;
				pMaterial->SetDensity(strlist2scalar(current->attrib["density"]));
				pMaterial->SetRestitution(strlist2scalar(current->attrib["restitution"]));
				pMaterial->SetStaticFriction(strlist2scalar(current->attrib["static_friction"]));
				pMaterial->SetDynamicFriction(strlist2scalar(current->attrib["dynamic_friction"]));
				if ( current->attrib.find("spinning_friction") != current->attrib.end() ) pMaterial->SetSpinningFriction(strlist2scalar(current->attrib["spinning_friction"]));
				
				if ( current->attrib.find("name") != current->attrib.end() ) pMaterial->m_szName = current->attrib["name"].front();

				int id = strlist2int(current->attrib["id"]);
				if ( material_array.size() <= id ) material_array.resize(id + 1, true);
				material_array[id] = pMaterial;
			} else if ( current->name == "ground" || current->name == "body" )
			{
				vpBody *pBody = new vpBody;
				pBody->SetGround(current->name == "ground");

				int id = strlist2int(current->attrib["id"]);
				if ( body_array.size() <= id ) body_array.resize(id + 1, true);
				body_array[id] = pBody;

				if ( current->attrib.find("name") != current->attrib.end() ) pBody->m_szName = current->attrib["name"].front();

				pBody->SetMaterial(material_array[strlist2int(current->attrib["material_id"])]);

				if ( current->attrib.find("collidable") != current->attrib.end() &&	*current->attrib["collidable"].begin() == "true" ) pBody->SetCollidable(false);
				if ( current->attrib.find("inertia") != current->attrib.end() ) pBody->SetInertia(strlist2Inertia(current->attrib["inertia"]));

				pBody->SetFrame(strlist2SE3((*itor)->attrib["frame"]));

				list<xml_node *>::const_iterator sub_itor = current->child.begin();
				while ( sub_itor != current->child.end() )
				{
					if ( (*sub_itor)->name == "box" )
						pBody->AddGeometry(new vpBox(strlist2Vec3((*sub_itor)->attrib["size"])), strlist2SE3((*sub_itor)->attrib["localframe"]));
					else if ( (*sub_itor)->name == "sphere" )
						pBody->AddGeometry(new vpSphere(strlist2scalar((*sub_itor)->attrib["radius"])), strlist2SE3((*sub_itor)->attrib["localframe"]));
					else if ( (*sub_itor)->name == "capsule" )
						pBody->AddGeometry(new vpCapsule(strlist2scalar((*sub_itor)->attrib["radius"]), strlist2scalar((*sub_itor)->attrib["height"])), strlist2SE3((*sub_itor)->attrib["localframe"]));
					sub_itor++;
				}
				world.AddBody(pBody);
			} else if ( current->name == "rotational_joint" )
			{
				vpRJoint *pJoint = new vpRJoint;
				pJoint->SetAxis(strlist2Vec3(current->attrib["axis"]));
				pJoint->SetAngle(strlist2scalar(current->attrib["q"]));
				pJoint->SetVelocity(strlist2scalar(current->attrib["dq"]));
				pJoint->SetInitialAngle(strlist2scalar(current->attrib["q_i"]));
				pJoint->SetElasticity(strlist2scalar(current->attrib["k"]));
				pJoint->SetDamping(strlist2scalar(current->attrib["c"]));
				if ( current->attrib.find("qul") != current->attrib.end() ) pJoint->SetUpperLimit(strlist2scalar(current->attrib["qul"]));
				if ( current->attrib.find("qll") != current->attrib.end() ) pJoint->SetLowerLimit(strlist2scalar(current->attrib["qll"]));
				body_array[strlist2int(current->attrib["left_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["left_frame"]));
				body_array[strlist2int(current->attrib["right_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["right_frame"]));
				if ( current->attrib.find("name") != current->attrib.end() ) pJoint->m_szName = current->attrib["name"].front();
			} else if ( current->name == "prismatic_joint" )
			{
				vpPJoint *pJoint = new vpPJoint;				
				pJoint->SetDirection(strlist2Vec3(current->attrib["direection"]));
				pJoint->SetDisplacement(strlist2scalar(current->attrib["q"]));
				pJoint->SetVelocity(strlist2scalar(current->attrib["dq"]));
				pJoint->SetInitialDisplacement(strlist2scalar(current->attrib["q_i"]));
				pJoint->SetElasticity(strlist2scalar(current->attrib["k"]));
				pJoint->SetDamping(strlist2scalar(current->attrib["c"]));
				if ( current->attrib.find("qul") != current->attrib.end() ) pJoint->SetUpperLimit(strlist2scalar(current->attrib["qul"]));
				if ( current->attrib.find("qll") != current->attrib.end() ) pJoint->SetLowerLimit(strlist2scalar(current->attrib["qll"]));
				body_array[strlist2int(current->attrib["left_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["left_frame"]));
				body_array[strlist2int(current->attrib["right_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["right_frame"]));
				if ( current->attrib.find("name") != current->attrib.end() ) pJoint->m_szName = current->attrib["name"].front();
			} else if ( current->name == "universal_joint" )
			{
				vpUJoint *pJoint = new vpUJoint;				
				pJoint->SetAxis(0, strlist2Vec3(current->attrib["axis0"]));
				pJoint->SetAngle(0, strlist2scalar(current->attrib["q0"]));
				pJoint->SetVelocity(0, strlist2scalar(current->attrib["dq0"]));
				pJoint->SetInitialAngle(0, strlist2scalar(current->attrib["q_i0"]));
				pJoint->SetElasticity(0, strlist2scalar(current->attrib["k0"]));
				pJoint->SetDamping(0, strlist2scalar(current->attrib["c0"]));
				pJoint->SetAxis(1, strlist2Vec3(current->attrib["axis1"]));
				pJoint->SetAngle(1, strlist2scalar(current->attrib["q1"]));
				pJoint->SetVelocity(1, strlist2scalar(current->attrib["dq1"]));
				pJoint->SetInitialAngle(1, strlist2scalar(current->attrib["q_i1"]));
				pJoint->SetElasticity(1, strlist2scalar(current->attrib["k1"]));
				pJoint->SetDamping(1, strlist2scalar(current->attrib["c1"]));
				if ( current->attrib.find("qul0") != current->attrib.end() ) pJoint->SetUpperLimit(0, strlist2scalar(current->attrib["qul0"]));
				if ( current->attrib.find("qul1") != current->attrib.end() ) pJoint->SetUpperLimit(1, strlist2scalar(current->attrib["qul1"]));
				if ( current->attrib.find("qll0") != current->attrib.end() ) pJoint->SetLowerLimit(0, strlist2scalar(current->attrib["qll0"]));
				if ( current->attrib.find("qll1") != current->attrib.end() ) pJoint->SetLowerLimit(1, strlist2scalar(current->attrib["qll1"]));
				body_array[strlist2int(current->attrib["left_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["left_frame"]));
				body_array[strlist2int(current->attrib["right_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["right_frame"]));
				if ( current->attrib.find("name") != current->attrib.end() ) pJoint->m_szName = current->attrib["name"].front();
			}  else if ( current->name == "sliding_joint" )
			{
				vpSJoint *pJoint = new vpSJoint;				
				pJoint->SetAxis(strlist2Vec3(current->attrib["axis"]));
				pJoint->SetAngle(strlist2scalar(current->attrib["q"]));
				pJoint->SetAngularVelocity(strlist2scalar(current->attrib["dq"]));
				pJoint->SetInitialAngle(strlist2scalar(current->attrib["q_i"]));
				pJoint->SetRotationalElasticity(strlist2scalar(current->attrib["kr"]));
				pJoint->SetRotationalDamping(strlist2scalar(current->attrib["cr"]));
				pJoint->SetDisplacement(strlist2scalar(current->attrib["x"]));
				pJoint->SetVelocity(strlist2scalar(current->attrib["dx"]));
				pJoint->SetInitialDisplacement(strlist2scalar(current->attrib["x_i"]));
				pJoint->SetSlidingElasticity(strlist2scalar(current->attrib["ks"]));
				pJoint->SetSlidingDamping(strlist2scalar(current->attrib["cs"]));
				if ( current->attrib.find("qulr") != current->attrib.end() ) pJoint->SetRotationalUpperLimit(strlist2scalar(current->attrib["qulr"]));
				if ( current->attrib.find("quls") != current->attrib.end() ) pJoint->SetSlidingUpperLimit(strlist2scalar(current->attrib["quls"]));
				if ( current->attrib.find("qllr") != current->attrib.end() ) pJoint->SetRotationalLowerLimit(strlist2scalar(current->attrib["qllr"]));
				if ( current->attrib.find("qlls") != current->attrib.end() ) pJoint->SetSlidingLowerLimit(strlist2scalar(current->attrib["qlls"]));
				body_array[strlist2int(current->attrib["left_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["left_frame"]));
				body_array[strlist2int(current->attrib["right_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["right_frame"]));
				if ( current->attrib.find("name") != current->attrib.end() ) pJoint->m_szName = current->attrib["name"].front();
			} else if ( current->name == "ball_joint" )
			{
				vpBJoint *pJoint = new vpBJoint;
				body_array[strlist2int(current->attrib["left_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["left_frame"]));
				body_array[strlist2int(current->attrib["right_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["right_frame"]));
				if ( current->attrib.find("name") != current->attrib.end() ) pJoint->m_szName = current->attrib["name"].front();
				pJoint->SetOrientation(Exp(Axis(&strlist2Vec3(current->attrib["orientation"])[0])));
				pJoint->SetInitialOrientation(strlist2SE3(current->attrib["initial_orientation"]));
				pJoint->SetElasticity(strlist2Inertia(current->attrib["K"]));
				pJoint->SetDamping(strlist2Inertia(current->attrib["C"]));
			} else if ( current->name == "welding_joint" )
			{
				vpWJoint *pJoint = new vpWJoint;

				body_array[strlist2int(current->attrib["left_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["left_frame"]));
				body_array[strlist2int(current->attrib["right_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["right_frame"]));
				if ( current->attrib.find("name") != current->attrib.end() ) pJoint->m_szName = current->attrib["name"].front();
			} else if ( current->name == "ignore_collision" )
			{
				int LIdx = strlist2int(current->attrib["left_body_id"]);
				int RIdx = strlist2int(current->attrib["right_body_id"]);
				if ( LIdx != -1 && RIdx != -1 ) world.IgnoreCollision(body_array[LIdx], body_array[RIdx]);
			} else if ( current->name == "onedof_joint" )
			{
				cerr << "operator >> : reading 1 DOF joint is not impelemented yet";
			} else if ( current->name == "ndof_joint" )
			{
				cerr << "operator >> : reading n DOF joint is not impelemented yet";
			} else if ( current->name == "helix_joint" )
			{
				vpHelixJoint *pJoint = new vpHelixJoint;
				pJoint->SetShape(strlist2scalar(current->attrib["radius"]), strlist2scalar(current->attrib["pitch"]));
				pJoint->SetPosition(strlist2scalar(current->attrib["q"]));
				pJoint->SetVelocity(strlist2scalar(current->attrib["dq"]));
				pJoint->SetInitialPosition(strlist2scalar(current->attrib["q_i"]));
				pJoint->SetElasticity(strlist2scalar(current->attrib["k"]));
				pJoint->SetDamping(strlist2scalar(current->attrib["c"]));
				if ( current->attrib.find("qul") != current->attrib.end() ) pJoint->SetUpperLimit(strlist2scalar(current->attrib["qul"]));
				if ( current->attrib.find("qll") != current->attrib.end() ) pJoint->SetLowerLimit(strlist2scalar(current->attrib["qll"]));
				body_array[strlist2int(current->attrib["left_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["left_frame"]));
				body_array[strlist2int(current->attrib["right_body_id"])]->SetJoint(pJoint, strlist2SE3(current->attrib["right_frame"]));
				if ( current->attrib.find("name") != current->attrib.end() ) pJoint->m_szName = current->attrib["name"].front();
			}

			itor++;
		}
	}
	return is;
}
