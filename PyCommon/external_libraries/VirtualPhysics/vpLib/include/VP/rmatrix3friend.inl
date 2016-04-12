//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	rmatrix3friend.inl
//						
//		version		:	v2.895
//		author		:	Jinwook Kim (jwkim@imrc.kist.re.kr)
//		last update	:	2003.9.2
//
//		Note		:
//
//////////////////////////////////////////////////////////////////////////////////

/*!
	\fn drand(double range)	
	get a random number between [range, range].
*/
template <class TYPE>
TYPE drand(TYPE range)
{
	return (TYPE)2.0 * range * (TYPE)rand() / (TYPE)RAND_MAX - range;
}

/*!
	\fn drand(double lb, double ub)	
	get a random number between [lb, ub].
*/
template <class TYPE>
TYPE drand(TYPE _min, TYPE _max)
{
	return min(_min, _max) + abs(_max - _min) * (TYPE)rand() / (TYPE)RAND_MAX;
}


template <class TYPE>
ostream &operator << (ostream &os, const _rmatrix<TYPE> &m)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[" << endl;
	for ( int i = 0; i < m.row; i++ )
	{
		for ( int j = 0; j < m.col; j++ )
		{
			if ( m(i,j) >= 0 ) os << " " << setw(6) << m(i,j) << " ";
			else os << setw(7) << m(i,j) << " ";
		}
		os << ";" << endl;
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

template <class TYPE>
TYPE FNorm(const _rmatrix<TYPE> &A)
{
	return sqrt(SquareSum(A));
}

template <class TYPE>
TYPE SquareSum(const _rmatrix<TYPE> &A)
{
	TYPE sum = (TYPE)0.0, *_m = A.element;
	int n = A.row * A.col;
	while ( n-- ) sum += *_m * *(_m++);
	return sum;
}

template <class TYPE>
TYPE AbsSum(const _rmatrix<TYPE> &A)
{
	TYPE sum = (TYPE)0.0, *_m = A.element;
	int n = A.row * A.col;
	while ( n-- ) sum += abs(*_m++);
	return sum;
}

template <class TYPE>
TYPE MaxVec(const _rmatrix<TYPE> &x, int *idx)
{
	TYPE mx = x.element[0];
	if ( idx != NULL ) *idx = 0;
	for ( int i = 1; i < _size(x); i++ )
	{
		if ( x.element[i] > mx )
		{
			mx = x.element[i];
			if ( idx != NULL ) *idx = i;
		}
	}
	return mx;
}

template <class TYPE>
TYPE MinVec(const _rmatrix<TYPE> &x, int *idx)
{
	TYPE mn = x.element[0];
	if ( idx != NULL ) *idx = 0;
	for ( int i = 1; i < _size(x); i++ )
	{
		if ( x.element[i] < mn )
		{
			mn = x.element[i];
			if ( idx != NULL ) *idx = i;
		}
	}
	return mn;
}

template <class TYPE>
int t_imax(int n, TYPE *dx)
{
	TYPE dmax;
	int i, idamax = 0;

	assert(n >= 0 && "t_imax -> wrong size");
	if ( n == 1 ) return 1;

	dmax = abs(dx[0]);
	for ( i = 1; i < n; i++ )
	{
		if ( abs(dx[i]) > dmax )
		{
			idamax = i;
			dmax = abs(dx[i]);
		}
	}
	return idamax;
}

template <class TYPE>
void t_dgefa(TYPE *x, int lda, int n, int *jpvt, int &info)
{
	TYPE t, *xk = x, *xj;
	int i, j, k, l;
	// gaussian elimination with partial pivoting
	info = -1;
	
	if ( n > 1 )
	{
		for ( k = 0; k < n - 1; k++, xk += lda )
		{
			// find l = pivot index
			l = t_imax(n-k, xk+k) + k;
			jpvt[k] = l;
			// zero pivot implies this column already triangularized
			if ( abs(xk[l]) < DGEFA_EPS ) info = k;
			else
			{
				// interchange if necessary
				if ( l != k )
				{
					t = xk[l];
					xk[l] = xk[k];
					xk[k] = t;
				}
				// compute multipliers
				t = -1 / xk[k];
				for ( j = 1 + k; j < n; j++ ) xk[j] *= t;
				// row elimination with column indexing
				for ( j = k+1, xj = xk+lda; j < n; j++, xj += lda )
				{
					t = xj[l];
					if ( l != k )
					{
						xj[l] = xj[k];
						xj[k] = t;
					}
					for ( i = 1 + k; i < n; i++ ) xj[i] += t * xk[i];
				}
			}
		}
	} else k = 0;

	jpvt[k] = k;
	if ( abs(xk[k]) < DGEFA_EPS ) info = k;
	return;
}

template <class TYPE>
TYPE Det(_rmatrix<TYPE> A)
{
	int info, i;
	static _rmatrix<int> _ipvt_Det;

	if ( _ipvt_Det.RowSize() < A.row ) _ipvt_Det.ReNew(A.row, 1);
	TYPE re = 1;

	t_dgefa(A.element, A.row, A.col, &_ipvt_Det[0], info);

	for ( i = 0; i < A.row; i++ )
	{
		if ( i != _ipvt_Det[i] ) re = -re;
		re *= A.element[i+i*A.row];
	}
	return re;
}

template <class TYPE>
_rmatrix<TYPE> Eye(int r, int c)
{
	_rmatrix<TYPE> re(r, c);
	int n = r * c;
	TYPE *_r = re.element;
	while ( n-- ) *(_r++) = 0;
	
	if ( c > r ) c = r;
	r++;

	_r = re.element;
	while ( c-- )
	{
		*_r = 1;
		_r += r;
	}		
	return re;
}

template <class TYPE>
_rmatrix<TYPE> Eye(int r)
{
	return Eye<TYPE>(r, r);
}

template <class TYPE>
_rmatrix<TYPE> Zeros(int r, int c)
{
	_rmatrix<TYPE> re(r, c);
	int n = r * c;
	TYPE *_r = re.element;
	while ( n-- ) *(_r++) = 0;
	return re;
}

template <class TYPE>
_rmatrix<TYPE> Rand(int r, int c)
{
	//srand( (unsigned)time( NULL ) );
	_rmatrix<TYPE> re(r, c);
	int n = r * c;
	TYPE *_r = re.element;
	while ( n-- ) *(_r++) = drand<TYPE>((TYPE)1.0);
	return re;		
}

template <class TYPE>
TYPE Inner(const _rmatrix<TYPE> &x, const _rmatrix<TYPE> &y)
{
	assert(x.row * x.col == y.row * y.col && "Inner(const _rmatrix &, const _rmatrix &) -> size is not compatible");

	TYPE sum = 0, *_tmpx = x.element, *_tmpy = y.element;
	int n = x.row * x.col;
	while ( n-- ) sum += *(_tmpx++) * *(_tmpy++);
	return sum;
}
template <class TYPE>
TYPE Quadratic(const _rmatrix<TYPE> &x, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &y)
{
	assert((x.row * x.col == A.row || y.row * y.col == A.col) && "Quadratic(const _rmatrix &, const _rmatrix &. const _rmatrix &) -> size is not compatible");
	
	int r, c = A.col;
	TYPE sum = 0, xa, *tmpa = A.element, *tmpx, *tmpy = y.element;
	while ( c-- )
	{
		xa = 0;
		tmpx = x.element;
		r = A.row;
		while ( r-- ) xa += *(tmpx++) * *(tmpa++);			
		sum += xa * *(tmpy++);
	}
	return sum;		
}

template <class TYPE>
TYPE Trace(const _rmatrix<TYPE> &A)
{
	assert(A.row == A.col && "Trace(_rmatrix &) -> not square");

	TYPE tr = 0, *_tmp = A.element;
	int n = A.row, np1 = n + 1;
	while ( n-- )
	{
		tr += *_tmp;
		_tmp += np1;
	}
	return tr;
}

template <class TYPE>
_rmatrix<TYPE> Diag(_rmatrix<TYPE> &m)
{
	int n, i;
	_rmatrix<TYPE> re;
	if ( m.row == 1 || m.col == 1 )
	{
		n = m.row * m.col;
		re = Zeros<TYPE>(n,n);
		for ( i = 0; i < n; i++ ) re.element[i*n+i] = m.element[i];
	} else 
	{
		n = min(m.row, m.col);
		re.ReNew(n,1);
		for ( i = 0; i < n; i++ ) re.element[i] = m.element[i*m.row+i];
	}
	return re;
}

template <class TYPE>
void AMultB(_rmatrix<TYPE> &C, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &B)
{
	assert(A.col == B.row && "AMultB(_rmatrix &, const _rmatrix &, const _rmtrix &) -> size is not compatible");

	C.ReNew(A.row, B.col);
	int i, bc = B.col, k, ar;
	TYPE sum, *tmpa, *tmpb = B.element, *rij = C.element;
	while ( bc-- )
	{
		ar = A.row;
		i = 0;
		while ( ar-- )
		{
			tmpa = A.element + (i++);
			sum = 0;
			k = A.col;
			while ( k-- )
			{
				sum += *tmpa * *tmpb;
				tmpa += A.row;
				tmpb++;
			}
			tmpb -= B.row;
			*(rij++) = sum;
		}
		tmpb += B.row;
	}
}

template <class TYPE>
void AMultBt(_rmatrix<TYPE> &C, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &B)
{
	assert(A.col == B.col && "AMultBt(_rmatrix &, const _rmatrix &, const _rmtrix &) -> size is not compatible");

	int i, j = 0, br = B.row, ar, ac;
	C.ReNew(A.row, br);
	TYPE sum, *tmpa, *tmpb, *rij = C.element;
	
	while ( br-- )
	{
		ar = A.row;
		i = 0;
		while ( ar-- )
		{
			tmpa = A.element + (i++);
			tmpb = B.element + j;
			sum = 0;
			ac = A.col;
			while ( ac-- )
			{
				sum += *tmpa * *tmpb;
				tmpa += A.row;
				tmpb += B.row;
			}
			*(rij++) = sum;
		}
		j++;
	}
}

template <class TYPE>
void AtMultB(_rmatrix<TYPE> &C, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &B)
{
	assert(A.row == B.row && "AtMultB(_rmatrix &, const _rmatrix &, const _rmtrix &) -> size is not compatible");

	C.ReNew(A.col, B.col);
	int ac, bc = B.col, ar;
	TYPE sum, *tmpa, *tmpb = B.element, *rij = C.element;
	while ( bc-- )
	{
		tmpa = A.element;
		ac = A.col;
		while ( ac-- )
		{
			sum = 0;
			ar = A.row;
			while ( ar-- ) sum += *(tmpa++) * *(tmpb++);
				
			*(rij++) = sum;
			tmpb -= B.row;
		}
		tmpb += B.row;
	}
}

template <class TYPE>
void t_dgesl(TYPE *x, int lda, int n, int *jpvt, TYPE *b, int job)
{
	TYPE t, *xk = x;
	int k, l;

	if ( job == 0 ) 
	{
		// job = 0 , solve  a * x = b
		// first solve  l*y = b
		if ( n >= 2 )
		{
			for ( k = 0; k < n-1; k++ )
			{
				l = jpvt[k];
				t = b[l];
				if ( l != k )
				{
					b[l] = b[k];
					b[k] = t;
				}
				for ( l = k+1; l < n; l++ ) b[l] += t * xk[l];
				xk += lda;				
			}
		}
		// now solve  u*x = y
		for ( k = n-1; k >= 0; k-- )
		{
			b[k] /= xk[k];
			t = -b[k];
			for ( l = 0; l < k; l++ ) b[l] += t * xk[l];
			xk -= lda;			
		}
		return;
	}

	// job = nonzero, solve  trans(a) * x = b
	// first solve  trans(u)*y = b
	for ( k = 0; k < n; k++ )
	{
		t = 0;
		for ( l = 0; l < k; l++ ) t += xk[l] * b[l];
		b[k] = (b[k] - t) / xk[k];
		xk += lda;
	}
	// now solve trans(l)*x = y
	if ( n >= 2 )
	{
		xk--;
		for ( k = n-1; k >= 0; k-- )
		{
			t = 0;			
			for ( l = 1; l < n-k; l++ ) t += xk[l] * b[k+l];
			b[k] += t;

			l = jpvt[k];
			if ( l != k )
			{
				t = b[l];
				b[l] = b[k];
				b[k] = t;
			}
			xk -= lda + 1;
		}
	}
	return;
}

template <class TYPE>
bool SolveAxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &B)
{
	if ( A.row * A.col == 1 ) 
	{
		if ( abs(A.element[0]) < DGEFA_EPS ) return false;
		x = B / A.element[0];
		return true;
	}
	if ( A.row != B.row ) return false;
	if ( A.row != A.col )
	{
		if ( A.row > A.col ) return SolveAxEqualB(A ^ A, x, A ^ B);
		bool flag = SolveAxEqualB(A | A, x, B);
		x = A ^ x;
		return flag;
		//return QRSolveAxEqualB(A, x, B);
	}
	int info, i;
	static _rmatrix<int> _ipvt_SlvAxB;
	static _rmatrix<TYPE> _A_SlvAxB;

	_A_SlvAxB = A;
	x = B;

	if ( _ipvt_SlvAxB.RowSize() < _A_SlvAxB.RowSize() ) _ipvt_SlvAxB.ReNew(_A_SlvAxB.RowSize(), 1);

	t_dgefa(_A_SlvAxB.element, _A_SlvAxB.row, _A_SlvAxB.col, &_ipvt_SlvAxB[0], info);
	if ( info != -1 ) return false;
	for ( i = 0; i < x.col; i++ ) t_dgesl(_A_SlvAxB.element, _A_SlvAxB.row, _A_SlvAxB.col, &_ipvt_SlvAxB[0], x.element+x.row*i, 0);

	return true;
}

template <class TYPE>
bool SolveAxEqualB_(_rmatrix<TYPE> &A, _rmatrix<TYPE> &x)
{
	/*if ( A.row * A.col == 1 ) 
	{
		if ( abs(A.element[0]) < DGEFA_EPS ) return false;
		x /= A.element[0];
		return true;
	}
	if ( A.row != x.row ) return false;
	if ( A.row != A.col )
	{
		if ( A.row > A.col ) return SolveAxEqualB(A ^ A, x, A ^ x);
		bool flag = SolveAxEqualB(A | A, x, x);
		x = A ^ x;
		return flag;
		//return QRSolveAxEqualB(A, x, B);
	}
	*/int info;
	static _rmatrix<int> _ipvt_SlvAxB;

	_ipvt_SlvAxB.ReNew(A.row, 1);

	t_dgefa(A.element, A.row, A.col, &_ipvt_SlvAxB[0], info);
	if ( info != -1 ) return false;
	for ( int i = 0; i < x.col; i++ ) t_dgesl(A.element, A.row, A.col, &_ipvt_SlvAxB[0], x.element+x.row*i, 0);

	return true;
}

template <class TYPE>
bool SolveAtxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &B)
{
	if ( A.row * A.col == 1 )
	{
		if ( A[0] == 0 ) return false;
		x = B / A[0];
		return true;
	}
	if ( A.col != B.row ) return false;
	if ( A.row != A.col )
		return SolveAxEqualB(~A, x, B);
	// return QRSolveAtxEqualB(A, x, B);

	int info, i;
	static _rmatrix<int> _ipvt_SlvAxB;
	static _rmatrix<TYPE> _A_SlvAxB;

	_A_SlvAxB = A;
	x = B;

	if ( _ipvt_SlvAxB.RowSize() < _A_SlvAxB.row ) _ipvt_SlvAxB.ReNew(_A_SlvAxB.row, 1);

	t_dgefa(_A_SlvAxB.element, _A_SlvAxB.row, _A_SlvAxB.col, &_ipvt_SlvAxB[0], info);
	if ( info != -1 ) return false;
	for ( i = 0; i < x.col; i++ ) t_dgesl(_A_SlvAxB.element, _A_SlvAxB.row, _A_SlvAxB.col, &_ipvt_SlvAxB[0], x.element+x.row*i, 1);

	return true;
}

template <class TYPE>
int t_dpofa(TYPE *a, int n)
{
	TYPE s, t;
	int i, j, k, info;
	// begin block with ...exits to 40
	for ( j = 0; j < n; j++ )
	{
		info = j;
		s = (TYPE)0.0;
		for ( k = 0; k < j; k++ )
		{
			t = (TYPE)0.0;
			for ( i = 0; i < k; i++ ) t += a[i+k*n] * a[i+j*n];
			t = a[k+j*n] - t;
			t /= a[k+k*n];
			a[k+j*n] = t;
			s += t * t;
		}
		s = a[j+j*n] - s;
		// exit
		if ( s <= (TYPE)0.0 ) return info;
		a[j+j*n] = sqrt(s);
	}	
	return -1;
}

template <class TYPE>
void t_dposl(TYPE *a, int n, TYPE *b)
{
	TYPE t;
	int i, k;

	// solve trans(r)*y = b
	for ( k = 0; k < n; k++ )
	{
		t = (TYPE)0.0;
		for ( i = 0; i < k; i++ ) t += a[i+k*n] * b[i];
		b[k] = (b[k] - t) / a[k+k*n];
	}

	// solve r*x = y
	for ( k = n - 1; k >= 0; k-- )
	{
		b[k] /= a[k+k*n];
		t = -b[k];
		for ( i = 0; i < k; i++ ) b[i] += t * a[i+k*n];		
	}
	return;
}

template <class TYPE>
bool SolvePosDefAxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &B)
{
	static _rmatrix<TYPE> _A;

	_A = A;

	if ( A.row != A.col ) return false;
	x = B;
	if ( t_dpofa(_A.element, A.row) != -1 ) return false;

	TYPE *x_i = x.element;

	for ( int i = 0; i < B.col; i++ )
	{
		t_dposl(_A.element, A.row, x_i);
		x_i += x.row;
	}
	return true;
}

// pivot x and b
// r, c : size of x
// k : starting pivot
// ipvt, jpvt : pivot index
template <class TYPE>
void t_fullpivoting(TYPE *x, int r, int c, int k, int *ipvt, int *jpvt)
{
	int i, j, imax, jmax, itmp;
	TYPE _max, dtmp, *xk = x + k + k * r, *xj;

	_max = abs(*xk);
	imax = jmax = k;

	for ( j = k; j < c; j++ )
	{
		for ( i = k; i < r; i++ )
		{
			dtmp = abs(*(xk++));
			if ( dtmp > _max ) { _max = dtmp; imax = i; jmax = j; }
		}
		xk += k;
	}

	// row swapping
	if ( imax != k )
	{
		itmp = ipvt[imax];	ipvt[imax] = ipvt[k]; 	ipvt[k] = itmp;
		for ( i = k, xk = x + k * r; i < c; i++, xk += r ) 
		{ 
			dtmp = xk[imax];	
			xk[imax] = xk[k];
			xk[k] = dtmp;			
		}
	}

	// column swapping
	if ( jmax != k ) 
	{
		itmp = jpvt[jmax];	jpvt[jmax] = jpvt[k];	jpvt[k] = itmp;
		for ( i = 0, xk = x + k * r, xj = x + jmax * r; i < r; i++, xk++, xj++ ) 
		{	
			dtmp = *xj;
			*xj = *xk;
			*xk = dtmp;			
		}
	}
}

// x a = b
// elimination process on x and b
// full pivoting on x and row pivoting on b
// if u want to ignore b, set bc = 0
// return value : rank of x
//
// x [ r X c ]
// b [ r X bc ]
// ipvt [ r ]
// jpvt [ c ]
// zero_tolerance : criterion for determining zero, 1e-8 will be good for usual case

template <class TYPE>
int t_gauss_elimination(TYPE *x, int r, int c, int *ipvt, int *jpvt, TYPE zero_tolerance)
{
	int i, j, k;
	TYPE t, *xi = x, *xk;

	for ( i = 0; i < r; i++ ) ipvt[i] = i;
	for ( j = 0; j < c; j++ ) jpvt[j] = j;

	for ( i = 0; i < min(r,c); i++, xi += r )
	{
		t_fullpivoting(x, r, c, i, ipvt, jpvt);
		if ( abs(xi[i]) < zero_tolerance ) return i;
		else
		{
			for ( j = i+1; j < r; j++ )
			{
				t =  - xi[j] / xi[i];
				xi[j] = 0;
				for ( k = i+1, xk = xi + r; k < c; k++, xk += r ) xk[j] += t * xk[i];
			}
		}
	}
	return min(r,c);
}

template <class TYPE>
int GaussElimination(_rmatrix<TYPE> &A, _rmatrix<int> &row_pivot, _rmatrix<int> &column_pivot, TYPE eps)
{
	row_pivot.ReNew(A.row);
	column_pivot.ReNew(A.col);
	return t_gauss_elimination(A.element, A.row, A.col, &row_pivot[0], &column_pivot[0], eps);
}
template <class TYPE>
int GaussElimination(_rmatrix<TYPE> &A, _rmatrix<int> &row_pivot, _rmatrix<int> &column_pivot)
{
	return GaussElimination(A, row_pivot, column_pivot, 1e-6);
}

template <class TYPE>
bool t_drive_to_zero(int d, const _rmatrix<TYPE> &A, _rmatrix<TYPE> &a, _rmatrix<TYPE> &f, _rmatrix<int> &C, _rmatrix<int> &NC)
{
	int i, j, n, idx, jdx;
	TYPE s, sp;
	static _rmatrix<TYPE> delf, dela, Acc, Acd, x;
	
	delf.ReNew(a.RowSize(), 1);
	dela.ReNew(a.RowSize(), 1);

	while ( true )
	{
//		_fdirection(d, delf, A, C);		
		delf.SetZero();
		delf[d] = 1;
		
		for ( i = n = 0; i < C.RowSize(); i++ ) if ( C[i] ) n++;

		Acc.ReNew(n, n);
		Acd.ReNew(n, 1);

		for ( i = idx = 0; i < A.RowSize(); i++ )
		{
			if ( C[i] )
			{
				for ( j = jdx = 0; j < A.RowSize(); j++ ) if ( C[j] ) Acc(idx, jdx++) = A(i, j);
				Acd[idx++] = A(i, d);
			}
		}
		
		if ( !SolvePosDefAxEqualB (Acc, x, Acd) ) SVDSolveAxEqualB (Acc, x, Acd);
		
		for ( i = idx = 0; i < A.RowSize(); i++ ) if ( C[i] ) delf[i] = -x[idx++];
//		_fdirection(d, delf, A, C);
		
		AMultB(dela, A, delf);
		
//		_max_step(s, j, f, a, delf, dela, d, C, NC);
		s = DBL_MAX;
		j = -1;

		if ( dela[d] > 0 )
		{
			j = d;
			s = -a[d] / dela[d];
		}

		for ( i = 0; i < C.RowSize(); i++ )
		{
			if ( C[i] && delf[i] < 0 )
			{
				sp = -f[i] / delf[i];
				if ( sp < s )
				{
					s = sp;
					j = i;
				}
			}
		}

		for ( i = 0; i < NC.RowSize(); i++ )
		{
			if ( NC[i] && dela[i] < 0 )
			{
				sp = -a[i] / dela[i];
				if ( sp < s )
				{
					s = sp;
					j = i;
				}
			}
		}
//		_max_step(s, j, f, a, delf, dela, d, C, NC);

		if ( j == -1 || s <= 0 )
		{
// 			fout << "LCP::_max_step -> fail " << s << endl;
			return false;
		}
		
		delf *= s;
		dela *= s;
		f += delf;
		a += dela;

		if ( C[j] )
		{
			C[j] = 0;
			NC[j] = 1;
		} else if ( NC[j] )
		{
			NC[j] = 0;
			C[j] = 1;
		} else
		{
			C[j] = 1;
			return true;
		}
	}
}

template <class TYPE>
bool t_check_lcp(const _rmatrix<TYPE> &a, const _rmatrix<TYPE> &f)
{
	for ( int i = 0; i < a.RowSize(); i++ )
	{
		if ( a[i] < -1E-6 ) return false;
		if ( f[i] < -1E-6 ) return false;
		if ( abs(a[i] * f[i]) > 1E-9 ) return false;
	}
	return true;
}

// find f such that Af + b >= 0 ,f >= 0 and <f, Af + b> = 0
template <class TYPE>
bool DantzigSolveLCP(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &f, const _rmatrix<TYPE> &b)
{
	int i;
	static _rmatrix<TYPE> a;
	static _rmatrix<int> C, NC;
	
	f.SetZero(b.row, 1);
	a = b;
	
	C.ReNew(b.row);
	NC.ReNew(b.row);

	for ( i = 0; i < b.row; i++ ) C[i] = NC[i] = 0;
	
	for ( i = 0; i < b.row; i++ )
	{
		if ( !C[i] && !NC[i] && a.element[i] < 0 )
		{
			if ( !t_drive_to_zero<TYPE>(i, A, a, f, C, NC) ) return t_check_lcp<TYPE>(a, f);
		}
	}
	
	return true;
}

template <class TYPE>
void lcp_lexicolemke(int nn, const TYPE *vec, const TYPE *q, TYPE *zlem, TYPE *wlem, int &info, int itermax, int &iter_count)
{ 
	int nobasis, drive, block, ic, jc, dim = nn, dim2 = 2 * (nn + 1);
	TYPE qs, z0, zb, dblock, pivot, tovip, tmp;

	int *basis = new int [dim];
	TYPE *A = new TYPE [dim * dim2];

	for ( ic = 0; ic < dim; ++ic )
		for ( jc = 0; jc < dim2; ++jc ) A[ic+jc*dim] = (TYPE)0.0;

	for ( ic = 0; ic < dim; ++ic )
	{
		for ( jc = 0; jc < dim; ++jc ) A[ic+(jc+dim+2)*dim] = -vec[dim*jc+ic];

		A[ic] = q[ic];
		A[ic+(ic+1)*dim] = (TYPE)1.0;
		A[ic+(dim+1)*dim] = -(TYPE)1.0;
	}
	
	qs = q[0];

	for ( ic = 1; ic < dim; ++ic )
	{
		if ( q[ic] < qs ) qs = q[ic];
	}

	info = 1;

	if ( qs >= -SVD_EPS )
	{
		for ( ic = 0; ic < dim; ++ic )
		{
			zlem[ic] = (TYPE)0.0;
			wlem[ic] = q[ic];
			z0 = (TYPE)0.0;
		}

		info = 0;
	} else
	{
		for ( ic = 0; ic < dim; ++ic ) basis[ic] = ic + 1;

		drive = dim + 1;
		block = 0;
		z0 = A[block];
		iter_count = 0;

		for ( ic = 1; ic < dim; ++ic )
		{
			zb = A[ic];
			if ( zb < z0 )
			{
				z0 = zb;
				block = ic;
			} else if ( zb == z0 )
			{
				for ( jc = 0; jc < dim; ++jc )
				{
					dblock = A[block+(1+jc)*dim] - A[ic+(1+jc)*dim];
					if ( dblock < 0 ) break;
					else if ( dblock > 0 )
					{
						block = ic;
						break;
					}
				}
			}
		}

		pivot = A[block+drive*dim];
		tovip = (TYPE)1.0 / pivot;

		A[block+drive*dim] = (TYPE)1.0;
		for ( ic = 0; ic < drive; ++ic ) A[block+ic*dim] = A[block+ic*dim] * tovip;
		for ( ic = drive + 1; ic < dim2; ++ic ) A[block+ic*dim] = A[block+ic*dim] * tovip;

		for ( ic = 0; ic < block; ++ic )
		{
			tmp = A[ic+drive*dim];
			for ( jc = 0; jc < dim2; ++jc ) A[ic+jc*dim] -= tmp * A[block+jc*dim];
		}

		for ( ic = block + 1; ic < dim; ++ic )
		{
			tmp = A[ic+drive*dim];
			for ( jc = 0; jc < dim2; ++jc ) A[ic+jc*dim] -= tmp * A[block+jc*dim];
		} 

		nobasis = basis[block];
		basis[block] = drive;

		while ( iter_count < itermax && info )
		{
			++iter_count;    

			if ( nobasis < dim + 1 ) drive = nobasis + (dim + 1);
			else if ( nobasis > dim + 1 ) drive = nobasis - (dim + 1);

			pivot = (TYPE)1e20;
			block = -1;

			for ( ic = 0; ic < dim; ++ic )
			{
				zb = A[ic+drive*dim];
				if ( zb > (TYPE)0.0 )
				{
					z0 = A[ic] / zb;
					if ( z0 > pivot ) continue;
					
					if ( z0 < pivot )
					{
						pivot = z0;
						block = ic;
					} else
					{
						for ( jc = 1; jc < dim+1; ++jc )
						{
							dblock = A[block+jc*dim] / pivot - A[ic+jc*dim] / zb;
							if ( dblock < 0 ) break;
							else if ( dblock > 0 )
							{
								block = ic;
								break;
							}
						}
					}
				}
			}

			if ( block == -1 ) break;

			if ( basis[block] == dim+1 ) info = 0;

			pivot = A[block+drive*dim];
			tovip = (TYPE)1.0 / pivot;
			A[block+drive*dim] = 1;

			for ( ic = 0; ic < drive; ++ic ) A[block+ic*dim] = A[block+ic*dim] * tovip;
			for ( ic = drive + 1; ic < dim2; ++ic ) A[block+ic*dim] = A[block+ic*dim] * tovip;

			for ( ic = 0; ic < block; ++ic )
			{
				tmp = A[ic+drive*dim];
				for ( jc = 0; jc < dim2; ++jc ) A[ic+jc*dim] -= tmp * A[block+jc*dim];
			}
			
			for ( ic = block + 1; ic < dim; ++ic )
			{
				tmp = A[ic+drive*dim];
				for ( jc = 0; jc < dim2; ++jc ) A[ic+jc*dim] -= tmp * A[block+jc*dim];
			} 

			nobasis = basis[block];
			basis[block] = drive;
		}

		for ( ic = 0; ic < dim; ++ic )
		{
			drive = basis[ic];
			if ( drive < dim + 1 )
			{ 
				zlem[drive-1] = (TYPE)0.0;
				wlem[drive-1] = A[ic];
			} else if ( drive > dim + 1 )
			{
				zlem[drive-dim-2] = A[ic];
				wlem[drive-dim-2] = (TYPE)0.0;
			}
		}
	}

	delete [] basis;
	delete [] A;	
}

template <class TYPE>
bool LemkeSolveLCP(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &f, const _rmatrix<TYPE> &b)
{
	f.SetZero(A.row, 1);
	_rmatrix<TYPE> w(A.row, 1), zr;
	int info;
	int iter_count;

	lcp_lexicolemke(A.row, &A[0], &b[0], &f[0], &w[0], info, 5 * A.row, iter_count);

	zr = A * f + b - w;
	if ( FNorm(zr) > SVD_EPS ) return false;

	return (info == 0);
}

template <class TYPE>
TYPE t_dsign(TYPE a, TYPE b)
{
	return (b >= 0 ? abs(a) : -abs(a));
}

template <class TYPE>
TYPE t_pythag(TYPE a, TYPE b)
{
	TYPE absa, absb;
	absa = abs(a);
	absb = abs(b);
	if ( absa > absb ) return absa * sqrt(1 + (absb / absa) * (absb / absa));
	else return (absb == 0 ? 0 : absb * sqrt(1 + (absa / absb) * (absa / absb)));
}

// singular value decomposition in 'numerical recipe'
// modified with EISPACK 'dsvd.f'
template <class TYPE>
bool t_svdcmp(int m, int n, TYPE *a, TYPE *w, TYPE *v, bool matu, bool matv, TYPE *rv1)
{
	int flag, i, its, j, jj, k, l, nm;
	TYPE anorm, c, f, g, h, s, scale, x, y, z;

	g = scale = anorm = 0; // Householder reduction to bidiagonal form
	for ( i = 0; i < n; i++ ) 
	{
		l = i+1;
		rv1[i] = scale * g;
		g = s = scale = 0;
		if ( i < m ) 
		{
			for ( k = i; k < m; k++ ) scale += abs(a[k+m*i]);
			if ( scale ) 
			{
				for ( k = i; k < m; k++ ) 
				{
					a[k+m*i] /= scale;
					s += a[k+m*i] * a[k+m*i];
				}
				f = a[i+m*i];
				g = -t_dsign(sqrt(s), f);
				h = f * g - s;
				a[i+m*i] = f - g;
				for ( j = l; j < n; j++ ) 
				{
					for ( s = 0, k = i; k < m; k++ ) s += a[k+m*i] * a[k+m*j];
					f = s / h;
					for ( k = i; k < m; k++ ) a[k+m*j] += f * a[k+m*i];
				}
				for ( k = i; k < m; k++ ) a[k+m*i] *= scale;
			}
		}
		w[i] = scale * g;
		g = s = scale = 0;
		if ( i < m && i != n-1 )
		{
			for ( k = l; k < n; k++ ) scale += abs(a[i+m*k]);
			if ( scale ) 
			{
				for ( k = l; k < n; k++ ) 
				{
					a[i+m*k] /= scale;
					s += a[i+m*k] * a[i+m*k];
				}
				f = a[i+m*l];
				g = -t_dsign(sqrt(s), f);
				h = f * g - s;
				a[i+m*l] = f - g; 
				for ( k = l; k < n; k++ ) rv1[k] = a[i+m*k] / h;
				for ( j = l; j < m; j++ ) 
				{
					for ( s = 0, k = l; k < n; k++ ) s += a[j+m*k] * a[i+m*k];
					for ( k = l ; k < n; k++ ) a[j+m*k] += s * rv1[k];
				}
				for ( k = l ; k < n; k++ ) a[i+m*k] *= scale;
			}
		}
		anorm = max(anorm, (abs(w[i]) + abs(rv1[i])));
	}
	if ( matv ) 
	{
		for ( i = n-1 ; i >= 0; i-- ) // Accumulation of right-hand transformations
		{
			if ( i < n ) 
			{
				if ( g )
				{
					for ( j = l; j < n; j++ ) // double devision to avoid possible underflow
						v[j+n*i] = (a[i+m*j] / a[i+m*l]) / g;
					for ( j = l; j < n; j++ ) 
					{
						for ( s = 0, k = l; k < n; k++ ) s += a[i+m*k] * v[k+n*j];
						for ( k = l; k < n; k++ ) v[k+n*j] += s * v[k+n*i];
					}
				}
				for ( j = l; j < n; j++ ) v[i+n*j] = v[j+n*i] = 0;
			}
			v[i+n*i] = 1;
			g = rv1[i];
			l = i;
		}
	}
	if ( matu )
	{
		for ( i = min(m-1, n-1); i >= 0; i-- ) // Accumulation of left-hand transformations
		{
			l = i + 1;
			g = w[i];
			for ( j = l; j < n; j++ ) a[i+m*j] = 0;
			if ( g )
			{
				g = 1 / g;
				for ( j = l; j < n; j++ ) 
				{
					for ( s = 0, k = l; k < m; k++ ) s += a[k+m*i] * a[k+m*j];
					f = (s / a[i+m*i]) * g;
					for ( k = i; k < m; k++ ) a[k+m*j] += f * a[k+m*i];
				}
				for ( j = i; j < m; j++ ) a[j+m*i] *= g;
			} else for ( j = i; j < m; j++ ) a[j+m*i] = 0;
			++a[i+m*i];
		}
	}
	for ( k = n-1; k >= 0; k-- ) // Diagonalization of the bidiagonal form: Loop over
	{
		for ( its = 1; its <= 30; its++ ) // singular values, and over allowed iterations
		{
			flag = 1;
			for ( l = k; l >= 0; l-- ) // Test for splitting
			{
				nm = l-1; // Note that rv1(1) is always zero
				if ( (abs(rv1[l]) + anorm) == anorm )
				{
					flag = 0;
					break;
				}
				if ( (abs(w[nm]) + anorm) == anorm ) break;
			}
			if ( flag )
			{
				c = 0;
				s = 1;
				for ( i = l; i <= k; i++ )
				{
					f = s * rv1[i];
					rv1[i] = c * rv1[i];
					if ( (abs(f) + anorm) == anorm ) break;
					g = w[i];
					h = t_pythag(f, g);
					w[i] = h;
					h = 1 / h;
					c = g * h;
					s = -f * h;
					if ( matu ) 
					{
						for ( j = 0; j < m; j++ ) 
						{
							y = a[j+m*nm];
							z = a[j+m*i];
							a[j+m*nm] = y * c + z * s;
							a[j+m*i] = z * c - y * s;
						}
					}
				}
			}
			z = w[k];
			if ( l == k ) // Convergence
			{
				if ( z < 0 ) // Singular value is made nonnegative
				{
					w[k] = -z;
					if ( matv ) for ( j = 0; j < n; j++ ) v[j+n*k] = -v[j+n*k];
				}
				break;
			}
			if ( its == 30 ) return false;
//			assert(its != 30 && "svdcmp : no convergence in 30 iterations");
			x = w[l]; // Shift from bottom 2-by-2 minor
			nm = k-1;
			y = w[nm];
			g = rv1[nm];
			h = rv1[k];
			f = ((y-z)*(y+z) + (g-h)*(g+h)) / (2 * h * y);
			g = t_pythag(f, (TYPE)1.0);
			f = ((x-z)*(x+z) + h * ((y / (f + t_dsign(g, f))) - h)) / x;
			c = s = 1; // Next QR transformation
			for ( j = l; j <= nm; j++ )
			{
				i = j+1;
				g = rv1[i];
				y = w[i];
				h = s * g;
				g = c * g;
				z = t_pythag(f, h);
				rv1[j] = z;
				c = f / z;
				s = h / z;
				f = x * c + g * s;
				g = g * c - x * s;
				h = y * s;
				y *= c;
				if ( matv ) 
				{
					for ( jj = 0; jj < n; jj++ )
					{
						x = v[jj+n*j];
						z = v[jj+n*i];
						v[jj+n*j] = x * c + z * s;
						v[jj+n*i] = z * c - x * s;
					}
				}
				z = t_pythag(f, h);
				w[j] = z; // Rotation can be arbitary if z = 0
				if ( z ) 
				{
					z = 1 / z;
					c = f * z;
					s = h * z;
				}
				f = c * g + s * y;
				x = c * y - s * g;
				if ( matu ) 
				{
					for ( jj = 0; jj < m; jj++ ) 
					{
						y = a[jj+m*j];
						z = a[jj+m*i];
						a[jj+m*j] = y * c + z * s;
						a[jj+m*i] = z * c - y * s;
					}
				}
			}
			rv1[l] = 0;
			rv1[k] = f;
			w[k] = x;
		}
	}

	return true;
}

template <class TYPE>
_rmatrix<TYPE> SVD(const _rmatrix<TYPE> &m)
{
	static _rmatrix<TYPE> _rv_SVD, _U_SVD;
	
	if ( m.row < m.col )
	{
		_U_SVD.ReNew(m.col, m.row);
		int i = 0, r = m.row, c;
		TYPE *a = _U_SVD.element, *at;
		while ( r-- )
		{
			at = m.element + (i++);
			c = m.col;
			while ( c-- )
			{
				*(a++) = *at;
				at += m.row;
			}
		}
	} else _U_SVD = m;
	
	_rmatrix<TYPE> S(_U_SVD.col, 1);
	
	if ( _rv_SVD.row < _U_SVD.col ) _rv_SVD.ReNew(_U_SVD.col, 1);
	t_svdcmp<TYPE>(_U_SVD.row, _U_SVD.col, _U_SVD.element, S.element, NULL, false, false, _rv_SVD.element);

	return S;
}

template <class TYPE>
void SVD(const _rmatrix<TYPE> &M, _rmatrix<TYPE> &U, _rmatrix<TYPE> &S, _rmatrix<TYPE> &V)
{
	static _rmatrix<TYPE> _rv_SVD, _U_SVD;

	if ( M.row > M.col ) U = ~M;
	else U = M;
	if ( M.col > _rv_SVD.row ) _rv_SVD.ReNew(M.col,1);
	S.ReNew(M.col,1);
	V.ReNew(M.col, M.col); 
	t_svdcmp(M.row, M.col, U.element, S.element, V.element, true, true, _rv_SVD.element);
}

template <class TYPE>
int Rank(const _rmatrix<TYPE> &m, TYPE eps)
{
	int i, rank = 0;
	_rmatrix<TYPE> s = SingualrValue(m);
	TYPE imx = 1 / MaxVec(s, NULL);
	for ( i = 0; i < s.row; i++ ) if ( s.element[i] * imx > eps ) rank++;
	return rank;
}

template <class TYPE>
int Rank(const _rmatrix<TYPE> &m)
{
	return Rank(m, TYPE(1e-6));
}

template <class TYPE>
bool SVDSolveAxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &B)
{
	assert(A.row == A.col && "SVDSolveAxEqualB -> not square matrix");

	static _rmatrix<TYPE> U, S, V, UtB, _rv_SVD, _U_SVD;
	bool re = true;

	U = A;

	_rv_SVD.ReNew(A.col,1);
	S.ReNew(A.col,1);
	V.ReNew(A.col, A.col); 
	
	if ( !t_svdcmp(A.row, A.col, U.element, S.element, V.element, true, true, _rv_SVD.element) ) return false;

	AtMultB(UtB, U, B);

	for ( int i = 0; i < UtB.row; i++ )
	{
		if ( abs(S.element[i]) < SVD_EPS ) 
		{
			S.element[i] = (TYPE)0.0;
			if ( abs(UtB.element[i]) > 10.0 * SVD_EPS ) re = false;
		} else
			S.element[i] = UtB.element[i] / S.element[i];
	}

	AMultB(x, V, S);

	return re;
}

template <class TYPE>
_rmatrix<TYPE> Inv(const _rmatrix<TYPE> &A)
{		
	_rmatrix<TYPE> x;
	bool re = SolveAxEqualB(A, x, Eye<TYPE>(A.row, A.row));
	assert(re && "Inv(const _rmatrix &) -> no inverse");
	return x;
}

template <class TYPE>
bool FixedPointSolveLCP(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &f, const _rmatrix<TYPE> &b)
{
	int n, i;
	int iteration = 10 * (A.row + 1);

	_rmatrix<TYPE> IplusA, IminusA, B, r, x;

	IplusA = A;

	IminusA.ReNew(A.row, A.col + 1);
	
	TYPE *_IminusA = IminusA.element, *_A = A.element;

	n = A.row * A.col;
	while ( n-- ) *(_IminusA++) = -*(_A++);

	for ( i = 0; i < A.row; i++ )
	{
		IplusA[(A.row + 1) * i] += (TYPE)1.0;
		IminusA[(A.row + 1) * i] += (TYPE)1.0;

		IminusA[A.row * A.col + i] = -b[i];
	}

	SolvePosDefAxEqualB<TYPE>(IplusA, B, IminusA);

	r.ReNew(A.row, 1);
	for ( i = 0; i < A.row; i++ ) r[i] = B[A.row * A.col + i];
	B.ReNew(A.row, A.row);

	x = r;

	while ( iteration-- )
	{
		f = x;
		f.SetAbs();
		AMultB(x, B, f);
		x += r;
	}

	f = x;
	x.SetAbs();
	f += x;

	return true;
}

template <class TYPE>
_rmatrix<TYPE> Abs(const _rmatrix<TYPE> &M)
{
	_rmatrix<TYPE> re(M.row, M.col);
	TYPE *_m = re.element, *_t = M.element;
	int n = M.row * M.col;
	while ( n-- ) *(_m++) = abs(*(_t++));
	return re; 
}

#define dsign(a, b) ((b) >= 0.0 ? abs(a) : -abs(a))

template <class TYPE>
void t_dqrdc(TYPE *x, int ldx, int n, int p, TYPE *qraux, int *jpvt, TYPE *work, int job)
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
	TYPE maxnrm, tt, nrmxl, t, tmp;
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

template <class TYPE>
void t_dqrsl(TYPE *x, int ldx, int n, int k, TYPE *qraux, const TYPE *y, TYPE *qy, TYPE *qty, TYPE *b, TYPE *rsd, TYPE *xb, int job, int &info)
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
	TYPE t,temp;
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

template <class TYPE>
bool QRSolveAxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &B)
{
	static _rmatrix<int> _jpvt_QR;
	static _rmatrix<TYPE> _qraux_QR, _qty_QR, _qy_QR, _work_QR, _A_QR;
	_A_QR.ReNew(A.row, A.col);
	for ( int i = 0; i < A.row * A.col; i++ ) _A_QR.element[i] = A.element[i];

	int n = _A_QR.row, p = _A_QR.col, j, info;
	
	x.ReNew(p, B.col);

	if ( _qraux_QR.RowSize() < p ) _qraux_QR.ReNew(p);
	if ( _qty_QR.RowSize() < n ) _qty_QR.ReNew(n);
	if ( _work_QR.RowSize() < max(p,n) ) _work_QR.ReNew(max(p,n));
	if ( _jpvt_QR.RowSize() < p ) _jpvt_QR.ReNew(p);
	
	for ( i = 0; i < p; i++ ) _jpvt_QR[i] = 0;

	t_dqrdc< TYPE> (_A_QR.element, n, n, p, _qraux_QR.element, &_jpvt_QR[0], _work_QR.element, 1);
	
	for ( j = 0; j < B.col; j++ ) 
	{
		t_dqrsl<TYPE>(_A_QR.element, n, n, p, _qraux_QR.element, B.element+j*B.row, NULL, _qty_QR.element, _work_QR.element, NULL, NULL, 100, info);
		for ( i = 0; i < p; i++ ) x.element[_jpvt_QR[i]+j*x.row] = _work_QR.element[i];
	}
	
	return (info == 0);
}

template <class TYPE>
bool FixedPointSolveAxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b)
{
	int i;
	int iteration = 10 * (A.row + 1);

	static _rmatrix<TYPE> IplusA, InvIplusA, _eye, y;
	
	IplusA = A;

	for ( i = 0; i < A.row; i++ ) IplusA.element[i*(A.row+1)] += (TYPE)1.0;

	_eye.SetEye(A.row, A.row);

	SolvePosDefAxEqualB(IplusA, InvIplusA, _eye);

	x.SetZero(A.row, 1);

	while ( iteration-- )
	{
		x += b;
		AMultB(y, InvIplusA, x);

		x -= y;
		x -= b;

		TYPE err = AbsSum(x);

		if ( err  < 1e-6 )
		{
			x = y;
			return true;
		}

		x = y;
	}
	
	return false;
}

template <class TYPE>
bool SORSolveAxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, const TYPE &w, int iter)
{
	x = b;
	TYPE sum;
	int i, j, n = A.row;

	while ( iter-- )
	{
		for ( i = 0; i < n; i++ )
		{
			sum = b[i];
			for ( j = 0; j < i; j++ ) sum -= A(i,j) * x[j];
			for ( j = i+1; j < n; j++ ) sum -= A(i,j) * x[j];
			x[i] = (1.0 - w) * x[i] + w / A[i*(n+1)] * sum;
		}
	}

	return true;
}

template <class TYPE>
bool SORSolveLCP(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, const TYPE &w, int iter)
{
	x = b;
	TYPE sum;
	int i, j, n = A.row;

	while ( iter-- )
	{
		for ( i = 0; i < n; i++ )
		{
			sum = b[i];
			for ( j = 0; j < i; j++ ) sum -= A(i,j) * x[j];
			for ( j = i+1; j < n; j++ ) sum -= A(i,j) * x[j];
			x[i] = (1.0 - w) * x[i] + w / A[i*(n+1)] * sum;
			if ( x[i] < 0.0 ) x[i] = 0.0;
		}
	}

	return true;
}
