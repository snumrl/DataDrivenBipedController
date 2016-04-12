/*
	VirtualPhysics v0.81

	2004.July.23.
	Imaging Media Research Center, KIST
	zinook@kist.re.kr
*/

#ifndef VP_TIMER
#define VP_TIMER

#ifdef _WIN32
	#include <windows.h>
#else
	#include <time.h>
#endif

class vpTimer
{
public:
					vpTimer();

	void			Tic();
	double			Toc();
	void			Suspend();
	void			Resume();

protected:

#ifdef _WIN32
	static double	m_fResolution;
	static int		m_nLowshift;
	LONGLONG		m_sStart;
	LONGLONG		m_sSuspend;
#else
	clock_t			m_sStart;
	clock_t			m_sSuspend;
#endif
	bool			m_bSuspend;
};

#endif
