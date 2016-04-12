#include <VP/rmatrix3.h>
#include <math.h>

#define dsign(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define SVD_EPS	1.0E-6

//----------------------------------------------------------------------
//
//		title		:	EISPACK C Version
//						
//		version		:	v1.5
//		author		:	Jinwook Kim (zinook@kist.re.kr)
//		last update	:	2001.7.23
//
//----------------------------------------------------------------------
//     THIS SUBROUTINE CALLS THE RECOMMENDED SEQUENCE OF
//     SUBROUTINES FROM THE EIGENSYSTEM SUBROUTINE PACKAGE (EISPACK)
//     TO FIND THE EIGENVALUES AND EIGENVECTORS (IF DESIRED)
//     OF A double GENERAL MATRIX.
//
//     ON INPUT
//
//        NM  MUST BE SET TO THE ROW DIMENSION OF THE TWO-DIMENSIONAL
//        ARRAY PARAMETERS AS DECLARED IN THE CALLING PROGRAM
//        DIMENSION STATEMENT.
//
//        N  IS THE ORDER OF THE MATRIX  A.
//
//        A  CONTAINS THE double GENERAL MATRIX.
//
//        MATZ  IS AN INTEGER VARIABLE SET EQUAL TO ZERO IF
//        ONLY EIGENVALUES ARE DESIRED.  OTHERWISE IT IS SET TO
//        ANY NON-ZERO INTEGER FOR BOTH EIGENVALUES AND EIGENVECTORS.
//
//     ON OUTPUT
//
//        WR  AND  WI  CONTAIN THE double AND IMAGINARY PARTS,
//        RESPECTIVELY, OF THE EIGENVALUES.  COMPLEX CONJUGATE
//        PAIRS OF EIGENVALUES APPEAR CONSECUTIVELY WITH THE
//        EIGENVALUE HAVING THE POSITIVE IMAGINARY PART FIRST.
//
//        Z  CONTAINS THE double AND IMAGINARY PARTS OF THE EIGENVECTORS
//        IF MATZ IS NOT ZERO.  IF THE J-TH EIGENVALUE IS double, THE
//        J-TH COLUMN OF  Z  CONTAINS ITS EIGENVECTOR.  IF THE J-TH
//        EIGENVALUE IS COMPLEX WITH POSITIVE IMAGINARY PART, THE
//        J-TH AND (J+1)-TH COLUMNS OF  Z  CONTAIN THE double AND
//        IMAGINARY PARTS OF ITS EIGENVECTOR.  THE CONJUGATE OF THIS
//        VECTOR IS THE EIGENVECTOR FOR THE CONJUGATE EIGENVALUE.
//
//        IERR  IS AN INTEGER OUTPUT VARIABLE SET EQUAL TO AN
//        ERROR COMPLETION CODE DESCRIBED IN SECTION 2B OF THE
//        DOCUMENTATION.  THE NORMAL COMPLETION CODE IS ZERO.
//
//        IV1  AND  FV1  ARE TEMPORARY STORAGE ARRAYS.
//----------------------------------------------------------------------

void _balanc(int nm, int n, double *a, int& low, int& igh, double* scale)
{
	int i, j, k, l, m, jj, iexc;
	double c, f, g, r, s, b2, radix;
	int noconv;
	radix = 16.0;
	b2 = radix * radix;
	k = 0;
	l = n-1;
	goto L100;
L20: ;
	scale[m] = j+1;
	if ( j == m ) goto L50;
	for ( i = 0; i <= l; i++ )
	{
		f = a[i+j*nm];
		a[i+j*nm] = a[i+m*nm];
		a[i+m*nm] = f;
	}
	for ( i = k; i <= n-1; i++ )
	{
		f = a[j+i*nm];
		a[j+i*nm] = a[m+i*nm];
		a[m+i*nm] = f;
	}
L50: ;
	if ( iexc == 1 ) goto L80;
	else goto L130;
L80: ;
	if ( l == 0 ) goto L280;
	l--;
L100: ;
	for ( jj = 1; jj <= l+1; jj++ )
	{
		j = l+1 - jj;
		for ( i = 0; i <= l; i++ )
		{
			if ( i == j ) goto L110;
			if ( a[j+i*nm] != 0.0 ) goto L120;
L110: ;
		}
		m = l;
		iexc = 1;
		goto L20;
L120: ;
	}
    goto L140;
L130: ;
	k++;
L140: ;
	for ( j = k; j <= l; j++)
	{
		for ( i = k; i <= l; i++ )
		{
			if ( i == j ) goto L150;
			if ( a[i+j*nm] != 0.0 ) goto L170;
L150: ;
		}
		m = k;
		iexc = 2;
		goto L20;
L170: ;
	  }
	for( i = k; i <= l; i++ ) scale[i] = 1.0;
	
L190: ;
	noconv = 0;
	for ( i = k; i <= l; i++ )
	{
		c = 0.0;
		r = 0.0;
		for ( j = k; j <= l; j++ )
		{
			if ( j == i ) goto L200;
			c += fabs(a[j+i*nm]);
			r += fabs(a[i+j*nm]);
L200: ;
		}
		if ( c == 0.0 || r == 0.0 ) goto L270;
		g = r / radix;
		f = 1.0;
		s = c + r;
L210: ;
		if ( c >= g ) goto L220;
		f *= radix;
		c *= b2;
		goto L210;
L220: ;
		g = r * radix;
L230: ;
		if ( c < g ) goto L240;
		f /= radix;
		c /= b2;
		goto L230;
L240: ;
		if ( (c + r) / f >= 0.95 * s ) goto L270;
		g = 1.0 / f;
		scale[i] *= f;
		noconv = 1;
		for ( j = k; j <= n-1; j++ ) a[i+j*nm] *= g;
		for ( j = 0; j <= l; j++ ) a[j+i*nm] *= f;
L270: ;
	}
	if ( noconv ) goto L190;
L280: ;
	low = k;
	igh = l;
	return;
}

void _balbak(int nm, int n, int low, int igh, double* scale, int m, double* z)
{
	int i, j, k, ii;
	double s;
	if ( m == 0 ) return;
	if ( igh != low ) 
	{
		for ( i = low; i <= igh; i++ )
		{
			s = scale[i];
			for ( j = 0; j <= m-1; j++) z[i+j*nm] *= s;
		}
	}
	for ( ii = 1; ii <= n; ii++)
	{
		i = ii-1;
		if ( i >= low && i <= igh ) goto L140;
		if ( i < low ) i = low - ii;
		k = (int)scale[i] - 1;
		if ( k == i ) goto L140;
		for ( j = 0; j <= m-1; j++ )
		{
			s = z[i+j*nm];
			z[i+j*nm] = z[k+j*nm];
			z[k+j*nm] = s;
		}
L140: ;
	}
	return;
}

void _elmhes(int nm, int n, int low, int igh, double* a, int* inf)
{
	int im1, jm1, m, mm1;
	double x, y;

	if ( igh < low + 2 ) goto L200;
	for ( m = low+1; m < igh; m++ )
	{
		mm1 = m - 1;
		x = 0.0;
		im1 = m;
		for ( jm1 = m; jm1 <= igh; jm1++)
		{
			if ( fabs(a[jm1+mm1*nm]) <= fabs(x) ) goto L100;
			x = a[jm1+mm1*nm];
            im1 = jm1;
L100: ;
		}
		inf[m] = im1+1;
		if ( im1 == m ) goto L130;
		for ( jm1 = mm1; jm1 < n; jm1++ )
		{
			y = a[im1+jm1*nm];
			a[im1+jm1*nm] = a[m+jm1*nm];
			a[m+jm1*nm] = y;
		}
		for ( jm1 = 0; jm1 <= igh; jm1++ )
		{
			y = a[jm1+im1*nm];
			a[jm1+im1*nm] = a[jm1+m*nm];
			a[jm1+m*nm] = y;
		}
L130: ;
		if ( x == 0.0 ) goto L180;
		
		for ( im1 = m+1; im1 <= igh; im1++ )
		{
			y = a[im1+mm1*nm];
			if (y == 0.0) goto L160;
			y /= x;
			a[im1+mm1*nm] = y;
			for ( jm1 = m; jm1 < n; jm1++ ) a[im1+jm1*nm] -= y * a[m+jm1*nm];
			for ( jm1 = 0; jm1 <= igh; jm1++ ) a[jm1+m*nm] += y * a[jm1+im1*nm];
L160: ;
		}
L180: ;
	}
L200: ;
	return;
}

void _eltran(int nm, int n, int low, int igh, double* a, int* inf, double* z)
{
	int i, j, kl, mm, m;

	for ( j = 0; j < n; j++ )
	{
		for ( i = 0; i < n; i++ ) z[i+j*nm] = 0.0;
		z[j+j*nm] = 1.0;
	}
	kl = igh - low - 1;
	if ( kl < 1 ) return;
	for ( mm = 1; mm <= kl; mm++ )
	{
		m = igh - mm;
		for ( i = m+1; i <= igh; i++ ) z[i+m*nm] = a[i+(m-1)*nm];
		i = inf[m]-1; 
		if ( i != m )
		{
			for ( j = m; j <= igh; j++ )
			{
				z[m+j*nm] = z[i+j*nm];
				z[i+j*nm] = 0.0;
			}
			z[i+m*nm] = 1.0;
		}
	}
	return;
}

void _hqr(int nm, int n, int low, int igh, double* h, double* wr, double* wi, int& ierr)
{
	int i, j, k, l, ll, m, mm, en, na, itn, its, mp2, enm2;
	double p, q, r, s, t, w, x, y, zz, norm, tst1, tst2;
	int notlas;
	ierr = 0;
	norm = 0.0;
	k = 0;
	for ( i = 0; i < n; i++ )
	{
		for ( j = k; j < n; j++ ) norm += fabs(h[i+j*nm]);
		k = i;
		if ( i >= low && i <= igh ) goto L50;
		wr[i] = h[i+i*nm];
		wi[i] = 0.0;
L50: ;
	}	
	en = igh;
	t = 0.0;
	itn = 30*n;
L60: ;
	if ( en < low ) goto L1001;
	its = 0;
	na = en-1;
	enm2 = en-2;	
L70: ;
	for ( ll = low+1; ll <= en+1; ll++ )
	{
		l = en + low+1 - ll;
		if ( l == low ) goto L100;
		s = fabs(h[l-1+(l-1)*nm]) + fabs(h[l+l*nm]);
		if ( s == 0.0 ) s = norm;
		tst1 = s;
		tst2 = tst1 + fabs(h[l+(l-1)*nm]);
		if ( tst2 == tst1 ) goto L100;
	}
L100: ;
	x = h[en+en*nm];
	if ( l == en ) goto L270;
	y = h[na+na*nm];
	w = h[en+na*nm] * h[na+en*nm];
	if ( l == na ) goto L280;
	if ( itn == 0 ) goto L1000;
	if ( its != 10 && its != 20 ) goto L130;
	t += x;
	for ( i = low; i <= en; i++ ) h[i+i*nm] -= x;
	
	s = fabs(h[en+na*nm]) + fabs(h[na+(enm2)*nm]);
	x = 0.75 * s;
	y = x;
	w = -0.4375 * s * s;
L130: ;
	its++;
	itn--;
	
	for ( mm = l; mm <= enm2; mm++ )
	{
		m = enm2 + l - mm;

		zz = h[m+m*nm];
		r = x - zz;
		s = y - zz;
		p = (r * s - w) / h[m+1+m*nm] + h[m+(m+1)*nm];
		q = h[m+1+(m+1)*nm] - zz - r - s;
		r = h[m+2+(m+1)*nm];
		s = fabs(p) + fabs(q) + fabs(r);
		p /= s;
		q /= s;
		r /= s;
		if ( m == l ) goto L150;
		tst1 = fabs(p)*(fabs(h[m-1+(m-1)*nm]) + fabs(zz) + fabs(h[m+1+(m+1)*nm]));
		tst2 = tst1 + fabs(h[m+(m-1)*nm])*(fabs(q) + fabs(r));
		if ( tst2 == tst1 ) goto L150;
	}
L150: ;
	mp2 = m+2;
	for ( i = mp2; i <= en; i++ )
	{
		h[i+(i-2)*nm] = 0.0;
		if ( i == mp2 ) goto L160;
		h[i+(i-3)*nm] = 0.0;
L160: ;
	}
	for ( k = m; k <= na; k++)
	{
		notlas = (k != na);
		if ( k == m ) goto L170;
		p = h[k+(k-1)*nm];
		q = h[k+1+(k-1)*nm];
		r = 0.0;
		if ( notlas ) r = h[k+2+(k-1)*nm];
		x = fabs(p) + fabs(q) + fabs(r);
		if ( x == 0.0 ) goto L260;
		p /= x;
		q /= x;
		r /= x;
L170: ;
		s = dsign(sqrt(p*p+q*q+r*r), p);
		if ( k == m ) goto L180;
		h[k+(k-1)*nm] = -s * x;
		goto L190;
L180: ;
		if ( l != m ) h[k+(k-1)*nm] = -h[k+(k-1)*nm];
L190: ;
		p += s;
		x = p / s;
		y = q / s;
		zz = r / s;
		q /= p;
		r /= p;
		if ( notlas ) goto L225;
		for ( j = k; j <= en; j++ )
		{
			p = h[k+j*nm] + q * h[k+1+j*nm];
			h[k+j*nm] -= p * x;
			h[k+1+j*nm] -= p * y;
		}
		j = min(en,k+3);
		for ( i = l; i <= j; i++ )
		{
            p = x * h[i+k*nm] + y * h[i+(k+1)*nm];
            h[i+k*nm] -= p;
            h[i+(k+1)*nm] -= p * q;
		}
		goto L255;
L225: ;
	 	for ( j = k; j <= en; j++ )
		{
            p = h[k+j*nm] + q * h[k+1+j*nm] + r * h[k+2+j*nm];
            h[k+j*nm] -= p * x;
            h[k+1+j*nm] -= p * y;
            h[k+2+j*nm] -= p * zz;
		}
		j = min(en,k+3);
		for ( i = l; i <= j; i++ )
		{
            p = x * h[i+k*nm] + y * h[i+(k+1)*nm] + zz * h[i+(k+2)*nm];
            h[i+k*nm] -= p;
            h[i+(k+1)*nm] -= p * q;
            h[i+(k+2)*nm] -= p * r;
		}
L255: ;
L260: ;
	}
	goto L70;
L270: ;
	wr[en] = x + t;
	wi[en] = 0.0;
	en = na;
	goto L60;
L280: ;
	p = (y - x) / 2.0;
	q = p * p + w;
	zz = sqrt(fabs(q));
	x += t;
	if ( q < 0.0 ) goto L320;
	zz = p + dsign(zz,p);
	wr[na] = x + zz;
	wr[en] = wr[na];
	if ( zz != 0.0 ) wr[en] = x - w / zz;
	wi[na] = 0.0;
	wi[en] = 0.0;
	goto L330;
L320: ;
	wr[na] = x + p;
	wr[en] = x + p;
	wi[na] = zz;
	wi[en] = -zz;
L330: ;
	en = enm2;
	goto L60;
L1000: ;
	ierr = en+1;
L1001: ;
	return;
}

void _cdiv(double ar, double ai, double br, double bi, double& cr, double& ci)
{
	double is, ars, ais, brs, bis;
	is = 1.0 / ( fabs(br) + fabs(bi) );
	ars = ar * is;
	ais = ai * is;
	brs = br * is;
	bis = bi * is;
	is = 1.0 / ( brs * brs + bis * bis );
	cr = (ars * brs + ais * bis) * is;
	ci = (ais * brs - ars * bis) * is;
	return;
}

void _hqr2(int nm, int n, int low, int igh, double* h, double* wr, double* wi, double* z, int& ierr)
{
	int i, j, k, l, m, en, na, ii, jj, ll, mm, nn, itn, its, mp2, enm2;
	double p, q, r, s, t, w, x, y, ra, sa, vi, vr, zz, norm, tst1, tst2;
	int notlas;
	ierr = 0;
	norm = 0.0;
	k = 0;
	for ( i = 0; i < n; i++ )
	{
		for ( j = k; j <= n-1; j++ ) norm += fabs(h[i+j*nm]);
		
		k = i;
		if ( i >= low && i <= igh ) goto L50;
		wr[i] = h[i+i*nm];
		wi[i] = 0.0;
L50: ;
	}
	en = igh;
	t = 0.0;
	itn = 30*n;
L60: ;
	if ( en < low ) goto L340;
	its = 0;
	na = en - 1;
	enm2 = en - 1;
L70: ;
	for ( ll = low+1; ll <= en+1; ll++ )
	{
		l = en + low+1 - ll;
		if ( l == low ) goto L100;
		s = fabs(h[l-1+(l-1)*nm]) + fabs(h[l+l*nm]);
		if ( s == 0.0 ) s = norm;
		tst1 = s;
		tst2 = tst1 + fabs(h[l+(l-1)*nm]);
		if ( tst2 == tst1 ) goto L100;
	}
L100: ;
	x = h[en+en*nm];
	if ( l == en ) goto L270;
	y = h[na+na*nm];
	w = h[en+na*nm] * h[na+en*nm];
	if ( l == na ) goto L280;
	if ( itn == 0 ) goto L1000;
	if ( its != 10 && its != 20 ) goto L130;
	t += x;
	for (  i = low; i <= en; i++ ) h[i+i*nm] -= x;
	
	s = fabs(h[en+na*nm]) + fabs(h[na+(enm2-1)*nm]);
	x = 0.75 * s;
	y = x;
	w = -0.4375 * s * s;
L130: ;
	its++;
	itn--;
	for ( mm = l+1; mm <= enm2; mm++ )
	{
		m = enm2 + l - mm;
		zz = h[m+m*nm];
		r = x - zz;
		s = y - zz;
		p = (r * s - w) / h[m+1+m*nm] + h[m+(m+1)*nm];
		q = h[m+1+(m+1)*nm] - zz - r - s;
		r = h[m+2+(m+1)*nm];
		s = fabs(p) + fabs(q) + fabs(r);
		p /= s;
		q /= s;
		r /= s;
		if ( m == l ) goto L150;
		tst1 = fabs(p)*(fabs(h[m-1+(m-1)*nm]) + fabs(zz) + fabs(h[m+1+(m+1)*nm]));
		tst2 = tst1 + fabs(h[m+(m-1)*nm])*(fabs(q) + fabs(r));
		if ( tst2 == tst1 ) goto L150;
	}
L150: ;
	mp2 = m+2;
	for ( i = mp2; i <= en; i++ )
	{
		h[i+(i-2)*nm] = 0.0;
		if ( i == mp2 ) goto L160;
		h[i+(i-3)*nm] = 0.0;
L160: ;
	}
	for ( k = m; k <= na; k++ )
	{
		notlas = (k != na);
		if ( k == m ) goto L170;
		p = h[k+(k-1)*nm];
		q = h[k+1+(k-1)*nm];
		r = 0.0;
		if ( notlas ) r = h[k+2+(k-1)*nm];
		x = fabs(p) + fabs(q) + fabs(r);
		if ( x == 0.0 ) goto L260;
		p /= x;
		q /= x;
		r /= x;
L170: ;
		s = dsign(sqrt(p*p+q*q+r*r),p);
		if ( k == m ) goto L180;
		h[k+(k-1)*nm] = -s * x;
		goto L190;
L180: ;
		if ( l != m ) h[k+(k-1)*nm] = -h[k+(k-1)*nm];
L190: ;
		p += s;
		x = p / s;
		y = q / s;
		zz = r / s;
		q /= p;
		r /= p;
		if ( notlas ) goto L225;
		for ( j = k; j <= n-1; j++ )
		{
			p = h[k+j*nm] + q * h[k+1+j*nm];
			h[k+j*nm] -= p * x;
			h[k+1+j*nm] -= p * y;
		}
		j = min(en,k+3);
		for ( i = 0; i <= j; i++ )
		{
			p = x * h[i+k*nm] + y * h[i+(k+1)*nm];
			h[i+k*nm] -= p;
			h[i+(k+1)*nm] -= p * q;
		}
		for ( i = low; i <= igh; i++ )
		{
			p = x * z[i+k*nm] + y * z[i+(k+1)*nm];
			z[i+k*nm] -= p;
			z[i+(k+1)*nm] -= p * q;
		}
		goto L255;
L225: ;
		for ( j = k; j <= n-1; j++ )
		{
			p = h[k+j*nm] + q * h[k+1+j*nm] + r * h[k+2+j*nm];
			h[k+j*nm] -= p * x;
			h[k+1+j*nm] -= p * y;
			h[k+2+j*nm] -= p * zz;
		}
		j = min(en,k+3);
		for ( i = 0; i <= j; i++ )
		{
			p = x * h[i+k*nm] + y * h[i+(k+1)*nm] + zz * h[i+(k+2)*nm];
			h[i+k*nm] -= p;
			h[i+(k+1)*nm] -= p * q;
			h[i+(k+2)*nm] -= p * r;
		}
		for ( i = low; i <= igh; i++ )
		{
			p = x * z[i+k*nm] + y * z[i+(k+1)*nm] + zz * z[i+(k+2)*nm];
			z[i+k*nm] -= p;
			z[i+(k+1)*nm] -= p * q;
			z[i+(k+2)*nm] -= p * r;
		}
L255: ;
L260: ;
	}
	goto L70;
L270: ;
	h[en+en*nm] = x + t;
	wr[en] = h[en+en*nm];
	wi[en] = 0.0;
	en = na;
	goto L60;
L280: ;
	p = (y - x) / 2.0;
	q = p * p + w;
	zz = sqrt(fabs(q));
	h[en+en*nm] = x + t;
	x = h[en+en*nm];
	h[na+na*nm] = y + t;
	if ( q < 0.0 ) goto L320;
	zz = p + dsign(zz,p);
	wr[na] = x + zz;
	wr[en] = wr[na];
	if ( zz != 0.0 ) wr[en] = x - w / zz;
	wi[na] = 0.0;
	wi[en] = 0.0;
	x = h[en+na*nm];
	s = fabs(x) + fabs(zz);
	p = x / s;
	q = zz / s;
	r = sqrt(p*p+q*q);
	p /= r;
	q /= r;
	for ( j = na; j < n; j++ )
	{
		zz = h[na+j*nm];
		h[na+j*nm] = q * zz + p * h[en+j*nm];
		h[en+j*nm] = q * h[en+j*nm] - p * zz;
	}
	for ( i = 0; i <= en; i++ )
	{
		zz = h[i+na*nm];
		h[i+na*nm] = q * zz + p * h[i+en*nm];
		h[i+en*nm] = q * h[i+en*nm] - p * zz;
	}
	for ( i = low; i <= igh; i++ )
	{
		zz = z[i+na*nm];
		z[i+na*nm] = q * zz + p * z[i+en*nm];
		z[i+en*nm] = q * z[i+en*nm] - p * zz;
	}
	goto L330;
L320: ;
	wr[na] = x + p;
	wr[en] = x + p;
	wi[na] = zz;
	wi[en] = -zz;
L330: ;
	en = enm2-1;
	goto L60;
L340: ;
	if ( norm == 0.0 ) goto L1001;
	for ( nn = 1; nn <= n; nn++ )
	{
		en = n - nn;
		p = wr[en];
		q = wi[en];
		na = en-1;
		if ( q < 0 ) goto L710;
		else if ( q == 0 ) goto L600;
		else goto L800;
L600: ;
		m = en;
		h[en+en*nm] = 1.0;
		if ( na+1 == 0 ) goto L800;
		for ( ii = 1; ii <= na+1; ii++ )
		{
			i = en - ii ;
			w = h[i+i*nm] - p;
			r = 0.0;
            for ( j = m; j <= en; j++ ) r += h[i+j*nm] * h[j+en*nm];
			
			if ( wi[i] >= 0.0 ) goto L630;
            zz = w;
            s = r;
            goto L700;
L630: ;   
			m = i;
            if ( wi[i] != 0.0 ) goto L640;
            t = w;
            if ( t != 0.0 ) goto L635;
			tst1 = norm;
			t = tst1;
L632: ;
			t = 0.01 * t;
			tst2 = norm + t;
			if ( tst2 > tst1 ) goto L632;
L635: ;
			h[i+en*nm] = -r / t;
            goto L680;
L640: ;
			x = h[i+(i+1)*nm];
            y = h[i+1+i*nm];
            q = (wr[i] - p) * (wr[i] - p) + wi[i] * wi[i];
            t = (x * s - zz * r) / q;
            h[i+en*nm] = t;
            if ( fabs(x) <= fabs(zz) ) goto L650;
            h[i+1+en*nm] = (-r - w * t) / x;
            goto L680;
L650: ;
			h[i+1+en*nm] = (-s - y * t) / zz;
L680: ;
			t = fabs(h[i+en*nm]);
            if ( t == 0.0 ) goto L700;
            tst1 = t;
            tst2 = tst1 + 1.0/tst1;
            if ( tst2 > tst1 ) goto L700;
            for ( j = i; j <= en; j++ ) h[j+en*nm] /= t;			
L700: ;
		}
		goto L800;
L710: ;
		m = na;
		if ( fabs(h[en+na*nm]) <= fabs(h[na+en*nm]) ) goto L720;
		h[na+na*nm] = q / h[en+na*nm];
		h[na+en*nm] = -(h[en+en*nm] - p) / h[en+na*nm];
		goto L730;
L720: ;
		_cdiv(0.0, -h[na+en*nm], h[na+na*nm]-p, q, h[na+na*nm], h[na+en*nm] );
L730: ;
		h[en+na*nm] = 0.0;
		h[en+en*nm] = 1.0;
		enm2 = na;
		if ( enm2 == 0 ) goto L800;
		for ( ii = 1; ii <= enm2; ii++ )
		{
			i = na - ii;
			w = h[i+i*nm] - p;
			ra = 0.0;
			sa = 0.0;
			for ( j = m; j <= en; j++ )
			{
				ra += h[i+j*nm] * h[j+na*nm];
				sa += h[i+j*nm] * h[j+en*nm];
			}
            if ( wi[i] >= 0.0 ) goto L770;
            zz = w;
            r = ra;
            s = sa;
            goto L795;
L770: ;
			m = i;
            if ( wi[i] != 0.0 ) goto L780;
			_cdiv( -ra, -sa, w, q, h[i+na*nm], h[i+en*nm]);
            goto L790;
L780: ;
			x = h[i+(i+1)*nm];
            y = h[i+1+i*nm];
            vr = (wr[i] - p) * (wr[i] - p) + wi[i] * wi[i] - q * q;
            vi = (wr[i] - p) * 2.0 * q;
            if ( vr != 0.0 || vi != 0.0 ) goto L784;
			tst1 = norm * (fabs(w) + fabs(q) + fabs(x) + fabs(y) + fabs(zz));
			vr = tst1;
L783: ;
			vr = 0.01 * vr;
			tst2 = tst1 + vr;
			if ( tst2 > tst1 ) goto L783;
L784: ;
			_cdiv(x*r-zz*ra+q*sa, x*s-zz*sa-q*ra, vr, vi, h[i+na*nm], h[i+en*nm]);
			if ( fabs(x) <= fabs(zz) + fabs(q) ) goto L785;
			h[i+1+na*nm] = (-ra - w * h[i+na*nm] + q * h[i+en*nm]) / x;
			h[i+1+en*nm] = (-sa - w * h[i+en*nm] - q * h[i+na*nm]) / x;
			goto L790;
L785: ;
			_cdiv(-r-y*h[i+na*nm], -s-y*h[i+en*nm], zz, q, h[i+1+na*nm], h[i+1+en*nm]);
L790: ;
			t = max(fabs(h[i+na*nm]), fabs(h[i+en*nm]));
			if ( t == 0.0 ) goto L795;
			tst1 = t;
			tst2 = tst1 + 1.0/tst1;
			if ( tst2 > tst1 ) goto L795;
			for ( j = i; j <= en; j++ )
			{
				h[j+na*nm] /= t;
				h[j+en*nm] /= t;
			}
L795: ;
		}
L800: ;
	}
	for ( i = 0; i < n; i++ )
	{
		if ( i >= low && i <= igh ) goto L840;
		for ( j = i; j < n; j++ ) z[i+j*nm] = h[i+j*nm];
L840: ;
	}
	for ( jj = low+1; jj <= n; jj++ )
	{
		j = n + low - jj;
		m = min(j,igh);
		for ( i = low; i <= igh; i++ )
		{
			zz = 0.0;
			for ( k = low; k <= m; k++ ) zz += z[i+k*nm] * h[k+j*nm];
			
			z[i+j*nm] = zz;
		}
	}
	goto L1001;
L1000: ;
	ierr = en+1;
L1001: ;
	return;
}

void _rg(int nm, int n, double* a, double* wr, double* wi, int matz, double* z, int* iv1, double* fv1, int& ierr)
{
	int is1, is2;
	if ( n > nm )
	{
		ierr = 10 * n;
		return;
	}
	_balanc(nm,n,a,is1,is2,fv1);
	_elmhes(nm,n,is1,is2,a,iv1);
	if ( matz == 0 )
	{
		_hqr(nm,n,is1,is2,a,wr,wi,ierr);
		return;
	}
	_eltran(nm, n, is1, is2, a, iv1, z);
	_hqr2(nm, n, is1, is2, a, wr, wi, z, ierr);
	if ( ierr != 0 ) return;
	_balbak(nm,n,is1,is2,fv1,n,z);
	return;
}

//////////////////////////////////////////////////////////////////////////////
//
// End of EISPACK
//
//////////////////////////////////////////////////////////////////////////////

RMatrix Eig(RMatrix A)
{
	assert(A.row == A.col && "RMatrix Eig : not sqare");
	
	int i, ierr, matz = 0, n = A.row;
	static IMatrix _iv1_Eig;
	static RMatrix _fv1_Eig;

	if ( _iv1_Eig.row < n ) _iv1_Eig.ReNew(n,1);
	if ( _fv1_Eig.row < n ) _fv1_Eig.ReNew(n,1);
	RMatrix re(n,2);
	
	_rg(n, n, A.element, re.element, (re.element+n), matz, 0, _iv1_Eig.element, _fv1_Eig.element, ierr);
	
	double sum = 0.0;
	for ( i = 0; i < n; i++ ) sum += re.element[i+n] * re.element[i+n];
	if ( sum < SVD_EPS ) re.col = 1;
	
	return re;
}

void Eig(RMatrix &re, RMatrix &m)
{
	assert(m.row == m.col && "RMatrix Eig : not sqare");
	
	int ierr, matz = 0, n = m.row;
	static IMatrix _iv1_Eig;
	static RMatrix _fv1_Eig;

	if ( _iv1_Eig.row < n ) _iv1_Eig.ReNew(n,1);
	if ( _fv1_Eig.row < n ) _fv1_Eig.ReNew(n,1);
	re.ReNew(n,2);
	
	_rg(n, n, m.element, re.element, (re.element+n), matz, 0, _iv1_Eig.element, _fv1_Eig.element, ierr);
}

void Eig(RMatrix m, RMatrix &v, RMatrix &d)
{
	int i, n = m.row;
	static IMatrix _iv1_Eig;
	static RMatrix _fv1_Eig;

	assert(n == m.col && "RMatrix Eig : not sqare");
	
	if ( _iv1_Eig.row < n ) _iv1_Eig.ReNew(n,1);
	if ( _fv1_Eig.row < n ) _fv1_Eig.ReNew(n,1);
	
	int ierr, matz = 1;
	
	if ( v.row*v.col != n*n )
	{
		delete [] v.element;
		v.element = new double [n*n];
	}
	v.row = v.col = n;

	if ( d.row*d.col != 2*n )
	{
		delete [] d.element;
		d.element = new double [2*n];
	}
	d.row = n;
	d.col = 2;

	_rg(n, n, m.element, d.element, d.element+n, matz, v.element, _iv1_Eig.element, _fv1_Eig.element, ierr);
	
	double sum = 0.0;
	for ( i = 0; i < n; i++ ) sum += d.element[i+n] * d.element[i+n];
	if ( sum < SVD_EPS ) d.col = 1;
}


//----------------------------------------------------------------------
//
//		title		:	LINPACK C Version
//						
//		version		:	v1.5
//		author		:	Jinwook Kim (zinook@kist.re.kr)
//		last update	:	2001.7.23
//
//----------------------------------------------------------------------

void _dqrdc(double *x, int ldx, int n, int p, double *qraux, int *jpvt, double *work, int job)
/*	
	dqrdc uses householder transformations to compute the qr factorization of an n by p matrix x.  column pivoting
	based on the 2-norms of the reduced columns may be performed at the users option.

	on entry

	x		double precision(ldx,p), where ldx .ge. n. x contains the matrix whose decomposition is to be computed.
	ldx		int. ldx is the leading dimension of the array x.
	n		int. n is the number of rows of the matrix x.
	p		int. p is the number of columns of the matrix x.
	jpvt	int(p). jpvt contains ints that control the selection of the pivot columns.  the k-th column x(k) of x
			is placed in one of three classes according to the value of jpvt(k).
			if jpvt(k) .gt. 0, then x(k) is an initial column.
			if jpvt(k) .eq. 0, then x(k) is a free column.
			if jpvt(k) .lt. 0, then x(k) is a final column.
			before the decomposition is computed, initial columns are moved to the beginning of the array x and final
			columns to the end.  both initial and final columns are frozen in place during the computation and only
			free columns are moved.  at the k-th stage of the reduction, if x(k) is occupied by a free column
			it is interchanged with the free column of largest reduced norm.  jpvt is not referenced if
			job .eq. 0.
	work	double precision(p). work is a work array.  work is not referenced if job .eq. 0.
	job		int. job is an int that initiates column pivoting.
			if job .eq. 0, no pivoting is done.
			if job .ne. 0, pivoting is done.

	on return

	x		x contains in its upper triangle the upper triangular matrix r of the qr factorization.
			below its diagonal x contains information from which the orthogonal part of the decomposition
			can be recovered.  note that if pivoting has been requested, the decomposition is not that
			of the original matrix x but that of x with its columns permuted as described by jpvt.
	qraux	double precision(p). qraux contains further information required to recover the orthogonal part of the decomposition.
	jpvt	jpvt(k) contains the index of the column of the original matrix that has been interchanged into the k-th column, 
			if pivoting was requested.
*/
{
	int j, l, pl, pu, maxj, jp, lup, k;
	double maxnrm, tt, nrmxl, t, tmp;
	pl = 0;
	pu = -1;
	if ( job != 0 )
	{
		// pivoting has been requested.  rearrange the columns according to jpvt.
		for ( j = 0; j < p; j++ )
		{
			jpvt[j] = j;
			if ( jpvt[j] < 0 ) jpvt[j] = -j;
			if ( jpvt[j] > 0 )
			{
				if ( j != pl ) for ( k = 0; k < n; k++ ) { tmp = x[k+pl*ldx]; x[k+pl*ldx] = x[k+j*ldx]; x[k+j*ldx] = tmp; }
				jpvt[j] = jpvt[pl];
				jpvt[pl++] = j;
			} 
		}
		pu = p-1;
		for ( j = p-1; j >= 0; j-- )
		{
			if ( jpvt[j] < 0 )
			{
				jpvt[j] = -jpvt[j];
				if ( j != pu )
				{
					for ( k = 0; k < n; k++ ) { tmp = x[k+pu*ldx]; x[pu+pl*ldx] = x[k+j*ldx]; x[k+j*ldx] = tmp; }
					jp = jpvt[pu];
					jpvt[pu] = jpvt[j];
					jpvt[j] = jp;
				}
				pu--;
			}
		}
	}
	// compute the norms of the free columns.
	if ( pu >= pl )
	{
		for ( j = pl; j <= pu; j++ )
		{
			tmp = 0.0; 
			for ( k = 0; k < n; k++ ) tmp += x[k+j*ldx]*x[k+j*ldx];
			qraux[j] = sqrt(tmp);
			work[j] = qraux[j];
		}
	}
	// perform the householder reduction of x.
	lup = ( n > p ? p : n );
	for ( l = 0; l < lup; l++ )
	{
		if ( l >= pl && l < pu )
		{
			// locate the column of largest norm and bring it into the pivot position.
			maxnrm = 0.0;
			maxj = l;
            for ( j = l; j <= pu; j++ )
			{
				if ( qraux[j] > maxnrm )
				{
					maxnrm = qraux[j];
					maxj = j;
				}
			}
            if ( maxj != l )
			{
				for ( k = 0; k < n; k++ ) { tmp = x[k+l*ldx]; x[k+l*ldx] = x[k+maxj*ldx]; x[k+maxj*ldx] = tmp; }
				qraux[maxj] = qraux[l];
				work[maxj] = work[l];
				jp = jpvt[maxj];
				jpvt[maxj] = jpvt[l];
				jpvt[l] = jp;
			}
		}
		qraux[l] = 0.0;
		if ( l != n-1 )
		{
			// compute the householder transformation for column (l+1).
			tmp = 0.0; 
			for ( k = 0; k < n-l; k++ ) tmp += x[l+k+l*ldx]*x[l+k+l*ldx];
			nrmxl = sqrt(tmp);
            if ( nrmxl > 1e-12 )
			{
				if ( fabs(x[l+l*ldx]) > 1e-12 ) nrmxl = dsign(nrmxl,x[l+l*ldx]);
				for ( k = 0; k < n-l; k++ ) x[l+k+l*ldx] *= 1.0 / nrmxl;
				x[l+l*ldx] += 1.0;
				// apply the transformation to the remaining columns,
				// updating the norms.
				if ( p >= l+2 )
				{
					for ( j = l+1; j < p; j++ )
					{
						tmp = 0.0; 
						for ( k = 0; k < n-l; k++ ) tmp += x[l+k+l*ldx]*x[l+k+j*ldx];
						t = -tmp/x[l+l*ldx];
						for ( k = 0; k < n-l; k++ ) x[l+k+j*ldx] += t*x[l+k+l*ldx];
						if ( j >= pl && j <= pu && fabs(qraux[j]) > 1e-12 )
						{
							tt = 1.0 - pow(fabs(x[l+j*ldx])/qraux[j], 2);
							tt = ( tt > 0.0 ? tt : 0.0 );
							t = tt;
							tt = 1.0 + 0.05*tt*pow(qraux[j]/work[j],2);
							if ( tt == 1.0 )
							{
								tmp = 0.0;
								for ( k = 0; k < n-l-1; k++ ) tmp += x[l+1+k+j*ldx]*x[l+1+k+j*ldx];
								qraux[j] = sqrt(tmp);
								work[j] = qraux[j];
							} else qraux[j] *= sqrt(t);
						}
					}
				}
				// save the transformation.
				qraux[l] = x[l+l*ldx];
				x[l+l*ldx] = -nrmxl;
			}
		}
	}
	return;
}

void _dqrsl(double *x, int ldx, int n, int k, double *qraux, const double *y, double *qy, double *qty, double *b, double *rsd, double *xb, int job, int &info)
/*
	dqrsl applies the output of dqrdc to compute coordinate transformations, projections, and least squares solutions.
	for k .le. min(n,p), let xk be the matrix 

		xk = (x(jpvt(1)),x(jpvt(2)), ... ,x(jpvt(k)))

	formed from columnns jpvt(1), ... ,jpvt(k) of the original n x p matrix x that was input to dqrdc (if no pivoting was
	done, xk consists of the first k columns of x in their original order).  dqrdc produces a factored orthogonal matrix q
	and an upper triangular matrix r such that

		xk = q * (r)
				 (0)

	this information is contained in coded form in the arrays x and qraux.

	on entry

	x		double precision(ldx,p). x contains the output of dqrdc.
	ldx		integer. ldx is the leading dimension of the array x.
	n		integer. n is the number of rows of the matrix xk.  it must have the same value as n in dqrdc.
	k		integer. k is the number of columns of the matrix xk.  k must nnot be greater than min(n,p), where p is the
			same as in the calling sequence to dqrdc.
	qraux	double precision(p). qraux contains the auxiliary output from dqrdc.
	y		double precision(n). y contains an n-vector that is to be manipulated by dqrsl.
	job		integer. job specifies what is to be computed.  job has the decimal expansion abcde, with the following meaning.
				if a.ne.0, compute qy.
				if b,c,d, or e .ne. 0, compute qty.
				if c.ne.0, compute b.
				if d.ne.0, compute rsd.
				if e.ne.0, compute xb.
			note that a request to compute b, rsd, or xb automatically triggers the computation of qty, for
			which an array must be provided in the calling sequence.

     on return

	qy		double precision(n). qy conntains q*y, if its computation has been requested.
	qty		double precision(n). qty contains trans(q)*y, if its computation has been requested.  here trans(q) is the
			transpose of the matrix q.
	b		double precision(k) b contains the solution of the least squares problem
			minimize norm2(y - xk*b),

	if its computation has been requested.  (note that if pivoting was requested in dqrdc, the j-th
	component of b will be associated with column jpvt(j) of the original matrix x that was input into dqrdc.)
	
	rsd    double precision(n). rsd contains the least squares residual y - xk*b, if its computation has been requested.  rsd is
	also the orthogonal projection of y onto the orthogonal complement of the column space of xk.
	
	xb     double precision(n). xb contains the least squares approximation xk*b, if its computation has been requested.  xb is also
	the orthogonal projection of y onto the column space of x.
	
	info   integer. info is zero unless the computation of b has been requested and r is exactly singular.  
	in this case, info is the index of the first zero diagonal element of r and b is left unaltered.

	the parameters qy, qty, b, rsd, and xb are not referenced if their computation is not requested and in this case
	can be replaced by dummy variables in the calling program. to save storage, the user may in some cases use the same
	array for different parameters in the calling sequence.  a frequently occuring example is when one wishes to compute
	any of b, rsd, or xb and does not need y or qty.  in this case one may identify y, qty, and one of b, rsd, or xb, while
	providing separate arrays for anything else that is to be computed.  thus the calling sequence

		call dqrsl(x,ldx,n,k,qraux,y,dum,y,b,y,dum,110,info)

	will result in the computation of b and rsd, with rsd overwriting y.  more generally, each item in the following
	list contains groups of permissible identifications for a single callinng sequence.
		1. (y,qty,b) (rsd) (xb) (qy)
		2. (y,qty,rsd) (b) (xb) (qy)
		3. (y,qty,xb) (b) (rsd) (qy)
		4. (y,qy) (qty,b) (rsd) (xb)
		5. (y,qy) (qty,rsd) (b) (xb)
		6. (y,qy) (qty,xb) (b) (rsd)
	in any group the value returned in the array allocated to the group corresponds to the last member of the group.
*/
{
	int i, j, ju, l;
	double t,temp;
	int cb, cqy, cqty, cr, cxb;
	info = 0;
	// determine what is to be computed.

	cqy = ( job / 10000 != 0 );
	cqty = ( ( job % 10000 ) != 0 );
	cb = ( ( job % 1000 ) / 100 != 0 );
	cr = ( ( job % 100 ) / 10 != 0 );
	cxb = ( ( job % 10 ) !=  0 );
	ju = ( k > n-1 ? n-1 : k );

	// special action when n=1.

	if ( ju == 0 )
	{
		if ( cqy ) qy[0] = y[0];
		if ( cqty ) qty[0] = y[0];
		if ( cxb ) xb[0] = y[0];
		if ( cb )
		{
			if ( fabs(x[0]) > 1e-12 ) 
			{
				b[0] = y[0] / x[0];
				for ( j = 1; j < k; j++) b[j] = 0.0;
			}
			else info = 1;
		}
		if ( cr ) rsd[0] = 0.0;
		return;
	}

	// set up to compute qy or qty.

	if ( cqy ) for ( l = 0; l < n; l++ ) qy[l] = y[l];
	if ( cqty ) for ( l = 0; l < n; l++ ) qty[l] = y[l];
	if ( cqy )
	{
		// compute qy.
		for ( j = ju-1; j >= 0; j-- )
		{
			if ( qraux[j] != 0.0 )
			{
				temp = x[j+j*ldx];
				x[j+j*ldx] = qraux[j];
				t = 0.0; 
				for ( l = 0; l < n-j; l++ ) t += x[j+l+j*ldx]*qy[j+l];
				t /= -x[j+j*ldx];
				for ( l = 0; l < n-j; l++ ) qy[j+l] += t*x[j+l+j*ldx];
				x[j+j*ldx] = temp;
			}
		}
	}
	if ( cqty )
	{
		// compute trans(q)*y
		for ( j = 0; j+1 <= ju; j++ )
		{
			if ( fabs(qraux[j]) > 1e-12 )
			{
				temp = x[j+j*ldx];
				x[j+j*ldx] = qraux[j];
				t = 0.0; 
				for ( l = 0; l < n-j; l++ ) t += x[j+l+j*ldx]*qty[j+l];
				t /= -x[j+j*ldx];
				for ( l = 0; l < n-j; l++ ) qty[j+l] += t*x[j+l+j*ldx];
				x[j+j*ldx] = temp;
			}
		}
	}
	// set up to compute b, rsd, or xb.
	if ( cb ) for ( l = 0; l < k; l++ ) b[l] = qty[l];
	if ( cxb ) for ( l = 0; l < k; l++ ) xb[l] = qty[l];
	if ( cr && k < n) for ( l = 0; l < n-k; l++ ) rsd[l] = qty[l];
	if ( cxb && k < n )
		for ( i = k; i < n; i++ ) xb[i] = 0.0;
	
	if ( cr )
		for ( i = 0; i < k; i++ ) rsd[i] = 0.0;

	if ( cb )
	{
		// compute b.
		for ( j = k-1; j >= 0; j-- )
		{
			if ( j < min(k,n) )
			{
				if ( fabs(x[j+j*ldx]) < SVD_EPS )
				{
					b[j] = 0.0;
					info = j+1;
					//break;
				} else b[j] /= x[j+j*ldx];
				
				if ( j != 0 )
				{
					t = -b[j];
					for ( l = 0; l < j; l++ ) b[l] += t*x[l+j*ldx];
				}
			} else b[j] = 0.0;
		}
	}
	if ( cr || cxb )
	{
		// compute rsd or xb as required.
		for ( j = ju-1; j >= 0; j-- )
		{
			if ( fabs(qraux[j]) > 1e-12 )
			{
				temp = x[j+j*ldx];
				x[j+j*ldx] = qraux[j];
				if ( cr )
				{
					t = 0.0; 
					for ( l = 0; l < n-j; l++ ) t += x[j+l+j*ldx]*rsd[j+l];
					t /= -x[j+j*ldx];
					for ( l = 0; l < n-j; l++ ) rsd[j+l] += t*x[j+l+j*ldx];
				}
				if ( cxb )
				{
					t = 0.0; 
					for ( l = 0; l < n-j; l++ ) t += x[j+l+j*ldx]*xb[j+l];
					t /= -x[j+j*ldx];
					for ( l = 0; l < n-j; l++ ) xb[j+l] += t*x[j+l+j*ldx];
				}
				x[j+j*ldx] = temp;
			}
		}
	}
	return;
}

// END OF LINPACK DQRDC

bool QRSolveAxEqualB(const RMatrix &A, RMatrix &x, const RMatrix &B)
{
	static IMatrix _jpvt_QR;
	static RMatrix _qraux_QR, _qty_QR, _qy_QR, _work_QR, _A_QR;
	_A_QR.ReNew(A.row, A.col);

	int i;
	for ( i = 0; i < A.row * A.col; i++ ) _A_QR.element[i] = A.element[i];

	int n = _A_QR.row, p = _A_QR.col, j, info;
	
	x.ReNew(p, B.col);

	if ( _qraux_QR.RowSize() < p ) _qraux_QR.ReNew(p);
	if ( _qty_QR.RowSize() < n ) _qty_QR.ReNew(n);
	if ( _work_QR.RowSize() < max(p,n) ) _work_QR.ReNew(max(p,n));
	if ( _jpvt_QR.RowSize() < p ) _jpvt_QR.ReNew(p);
	
	for ( i = 0; i < p; i++ ) _jpvt_QR.element[i] = 0;

	_dqrdc(_A_QR.element, n, n, p, _qraux_QR.element, _jpvt_QR.element, _work_QR.element, 1);
	
	for ( j = 0; j < B.col; j++ ) 
	{
		_dqrsl(_A_QR.element, n, n, p, _qraux_QR.element, B.element+j*B.row, NULL, _qty_QR.element, _work_QR.element, NULL, NULL, 100, info);
		for ( i = 0; i < p; i++ ) x.element[_jpvt_QR.element[i]+j*x.row] = _work_QR.element[i];
	}
	
	return (info == 0);
}

bool QRSolveAtxEqualB(const RMatrix &At, RMatrix &x, const RMatrix &B)
{
	int i, j;
	static IMatrix _jpvt_QR;
	static RMatrix _qraux_QR, _qty_QR, _qy_QR, _work_QR, _A_QR;

	_A_QR.ReNew(At.col, At.row);
	double *a = _A_QR.element, *at;
	for ( i = 0; i < _A_QR.col; i++ )
	{
		at = At.element + i;
		for ( j = 0; j < _A_QR.row; j++, at += At.row )	*(a++) = *at;
	}
	
	int n = _A_QR.row, p = _A_QR.col, info;
	x.ReNew(p, B.col);

	if ( _qraux_QR.RowSize() < p ) _qraux_QR.ReNew(p);
	if ( _qty_QR.RowSize() < n ) _qty_QR.ReNew(n);
	if ( _work_QR.RowSize() < max(p,n) ) _work_QR.ReNew(max(p,n));
	if ( _jpvt_QR.RowSize() < p ) _jpvt_QR.ReNew(p);
	
	for ( i = 0; i < p; i++ ) _jpvt_QR.element[i] = 0;

	_dqrdc(_A_QR.element, n, n, p, _qraux_QR.element, _jpvt_QR.element, _work_QR.element, 1);
	
	for ( j = 0; j < B.col; j++ ) 
	{
		_dqrsl(_A_QR.element, n, n, p, _qraux_QR.element, B.element+j*B.row, NULL, _qty_QR.element, _work_QR.element, NULL, NULL, 100, info);
		for ( i = 0; i < p; i++ ) x.element[_jpvt_QR.element[i]+j*x.row] = _work_QR.element[i];
	}
	
	return (info == 0);
}
/*
bool SVDSolveAxEqualB(const RMatrix &A, RMatrix &x, const RMatrix &B)
{
	assert(A.row == A.col && "SVDSolveAxEqualB -> not square matrix");

	static RMatrix U, S, V, UtB, _rv_SVD, _U_SVD;
	bool re = true;

	U = A;

	_rv_SVD.ReNew(A.col,1);
	S.ReNew(A.col,1);
	V.ReNew(A.col, A.col); 
	
	if( !_svdcmp(A.row, A.col, U.element, S.element, V.element, 1, 1, _rv_SVD.element) ) return false;

	AtMultB(UtB, U, B);

	for ( int i = 0; i < UtB.row; i++ )
	{
		if ( fabs(S.element[i]) < SVD_EPS ) 
			S.element[i] = 0.0;
		else
			S.element[i] = UtB.element[i] / S.element[i];
	}

	AMultB(x, V, S);

	return re;
}
*/