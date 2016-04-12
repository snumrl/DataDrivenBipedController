/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef SMATRIX
#define SMATRIX

#include <VP/rmatrix3.h>
#include <VP/_array.h>

template <class TYPE>
class _smatrix
{
public:
		_smatrix()
		{
			row = col = 0;
		}

		~_smatrix()
		{
		}

		void clear(int r, int c)
		{
			row = r;
			col = c;
			index.resize(r);
			value.resize(r);
			diag_recp.resize(r);
			for ( int i = 0; i < r; i++ )
			{
				index[i].clear();
				value[i].clear();
				diag_recp[i] = (TYPE)0.0;
			}
		}
		
		// assume that both setValue(r, c1, v) and setValue(r, c2, v) are never called at the same stage, if c1 == c2.
		void setValue(int r, int c, const TYPE &val)
		{
			if ( r == c ) diag_recp[r] = (TYPE)1.0 / val;
			else
			{
				index[r].push_back(c);
				value[r].push_back(val);

				index[c].push_back(r);
				value[c].push_back(val);
			}			
		}

		friend bool SORSolveAxEqualB<TYPE>(const _smatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, const TYPE &w, int iter)
		{
			x = b;
			TYPE sum, one_w = 1.0 - w;
			int i, j, n = A.row;

			while ( iter-- )
			{
				for ( i = 0; i < n; i++ )
				{
					sum = b[i];
					for ( j = 0; j < A.index[i].size(); j++ ) sum -= A.value[i][j] * x[A.index[i][j]];
					x[i] = one_w * x[i] + w * A.diag_recp[i] * sum;
				}
			}

			return true;
		}

		friend _rmatrix<TYPE> convert(const _smatrix<TYPE> &A)
		{
			_rmatrix<TYPE> re = Zeros<TYPE>(A.row, A.col);
			for ( int i = 0; i < A.index.size(); i++ )
			{
				re(i,i) = 1.0 / A.diag_recp[i];
				for ( int j = 0; j < A.index[i].size(); j++ )
					re(i, A.index[i][j]) = A.value[i][j];
			}
			return re;
		}

		friend bool SORSolveLCP<TYPE>(const _smatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, const TYPE &w, int iter)
		{
			x = b;
			TYPE sum, one_w = 1.0 - w;
			int i, j, n = A.row;

			while ( iter-- )
			{
				for ( i = 0; i < n; i++ )
				{
					sum = b[i];
					for ( j = 0; j < A.index[i].size(); j++ ) sum -= A.value[i][j] * x[A.index[i][j]];
					x[i] = one_w * x[i] + w * A.diag_recp[i] * sum;
					if ( x[i] < 0.0 ) x[i] = 0.0;
				}
			}

			return true;
		}

protected:
	int row, col;
	_array<TYPE>				diag_recp;
	_array<_array<int> >		index;
	_array<_array<TYPE> >		value;
};

#endif
