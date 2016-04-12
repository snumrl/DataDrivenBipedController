/*
	VirtualPhysics v0.82

	2005.Dec.29.
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/


inline void vpForce::ApplyLocalForce(const Vec3 &f)
{
	m_sF.InvdAd(m_sApplyPoint, f);
}

inline void vpForce::ApplyLocalForce(const dse3 &f)
{
	m_sF.dAd(-m_sApplyPoint, f);
}

inline void vpForce::ApplyForce(const dse3 &f, VP::FORCE_TYPE type)
{
	switch ( type )
	{
	case VP::GLOBAL :
		ApplyGlobalForce(f);
		break;
	case VP::LOCAL :
		ApplyLocalForce(f);
		break;
	}
}

inline void vpForce::ApplyForce(const Vec3 &f, VP::FORCE_TYPE type)
{
	switch ( type )
	{
	case VP::GLOBAL :
		ApplyGlobalForce(f);
		break;
	case VP::LOCAL :
		ApplyLocalForce(f);
		break;
	}
}

inline const vpBody *vpForce::GetBody(void) const
{
	return m_pBody;
}

inline const Vec3 &vpForce::GetApplyPoint(void) const
{
	return m_sApplyPoint;
}

inline const dse3 &vpForce::GetValue(void) const
{
	return m_sF;
}
