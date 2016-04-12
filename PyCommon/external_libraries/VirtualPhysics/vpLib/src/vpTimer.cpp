#include <VP/vpTimer.h>

#ifdef _WIN32
	double	vpTimer::m_fResolution = 0;
	int		vpTimer::m_nLowshift = 0;
#endif

#ifdef _WIN32
	inline LONGLONG __clock()
	{
		LARGE_INTEGER Count;
		QueryPerformanceCounter(&Count);
		return Count.QuadPart;
	}
#else
	inline clock_t __clock() { return clock(); }
#endif


vpTimer::vpTimer()
{
#ifdef _WIN32
	if ( m_fResolution == 0 || m_nLowshift == 0 )
	{
		LARGE_INTEGER m_Frequency;
		QueryPerformanceFrequency(&m_Frequency);
		LONGLONG nShift = m_Frequency.QuadPart;
		m_nLowshift = 0;
		while ( nShift > 1000000 )
		{
			m_nLowshift++;
			nShift >>= 1;
		}
		m_fResolution = 1.0 / (double)nShift;
	}
#endif

	m_sStart = __clock();
	m_sSuspend = __clock();
	m_bSuspend = false;
}

void vpTimer::Tic(void)
{
	m_sStart = __clock();
}

double vpTimer::Toc(void )
{
	if ( !m_bSuspend )
	{
#ifdef _WIN32
	return (double)((__clock() - m_sStart) >> m_nLowshift) * m_fResolution;	
#else
	return (double)(__clock() - m_sStart) / (double) CLOCKS_PER_SEC;
#endif
	} else
	{
#ifdef _WIN32
	return (double)((m_sSuspend - m_sStart) >> m_nLowshift) * m_fResolution;	
#else
	return (double)(m_sSuspend - m_sStart) / (double) CLOCKS_PER_SEC;
#endif
	}
}

void vpTimer::Suspend(void)
{
	if ( !m_bSuspend )
	{
		m_sSuspend = __clock();
		m_bSuspend = true;
	}
}

void vpTimer::Resume(void)
{
	if ( m_bSuspend ) 
	{
		m_sStart += (__clock() - m_sSuspend);
		m_bSuspend = false;
	}
}
