//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	rmatrix3.h
//						
//		version		:	v2.895
//		author		:	Jinwook Kim (jwkim@imrc.kist.re.kr)
//		last update	:	2003.9.2
//
//		Note		:
//
//////////////////////////////////////////////////////////////////////////////////

#ifndef _RMatrix3_
#define _RMatrix3_

#include <math.h>
#include <iostream>
#include <iomanip>
#include <assert.h>
#include <float.h>

#define DGEFA_EPS	1.0E-6
#define SVD_EPS		1.0E-6

using namespace std;

template <class TYPE> class _rmatrix;

/*!
	\class _rmatrix
	\brief General dense matrix and linear algebraic calculation
	
	_rmatrix is a template class to manipulate general dense matrices and calculate linear algebraic equations.
	EISPACK is used for funciton to get eigenvalue and LINPACK is used for solving linear equations.
*/

#include <VP/rmatrix3friend.inl>

template <class TYPE>
class _rmatrix
{
public:
	_rmatrix();
	
	/** @name Construction and manipulation
	*/
	//@{
	/*!
		constructor
		make a r-by-c matrix.
	*/
	_rmatrix(int r, int c);
	
	/*!
		copy constructor
	*/
	_rmatrix(const _rmatrix<TYPE> &m);
	
	/*!
		constructor
		make a r-by-c matrix and its elements will be copied from an array d.
	*/
	_rmatrix(int r, int c, const TYPE d[]);

	/*!
		resize itself to be a r-by-c matrix. Elements of the matrix will not be initialized.
	*/
	void ReNew(int r, int c);
	void ReNew(int r);

	/*!
		set all the elements to be zero.
	*/
	void SetZero(void);

	/*!
		resize itself to be a r-by-c zero matrix.
	*/
	void SetZero(int r, int c);

	/*!
		make itself to be absolute.
	*/
	void SetAbs(void);

	/*!
		resize itself to be the identity matrix.
		Resized matrix is a r-by-c matrix with 1's on the diagonal and zeros elsewhere.
	*/
	void SetEye(int r, int c);
	
	/*!
		invert itself.
		\return true if the inversion exists.
	*/
	bool SetInv(void)
	{
		static _rmatrix<TYPE> _eye, _copy;

		_copy = *this;
		_eye.SetEye(row, row);

		bool re = SolveAxEqualB(_copy, *this, _eye);
		return re;
	}

	/*!
		normalize itself, where the matrix is assumed to be a vector.
		\return a length of the vector
	*/
	TYPE Normalize(void)
	{
		TYPE norm = FNorm(*this), inorm = SCALAR_1 / norm;
		*this *= inorm;
		return norm;
	}
	//@}
	
	~_rmatrix();

	/** @name Operators
	*/
	//@{
	/*!
		access the i th element, where the matrix is assumed to be a column order vector.
	*/
	TYPE &operator [] (int i);

	const TYPE &operator [] (int i) const;

	/*!
		access the i th row and the j th column element.
	*/
	TYPE &operator () (int i, int j );

	const TYPE &operator () (int i, int j ) const;

	/*!
		unary plus operator
	*/
	const _rmatrix<TYPE> &operator + (void) const;
	
	/*!
		unary minus operator
	*/
	_rmatrix<TYPE> operator - (void) const;
	
	/*!
		transpose operator
	*/
	_rmatrix<TYPE> operator ~ (void) const;
	
	/*!
		substitution operator
	*/
	const _rmatrix<TYPE> &operator = (const _rmatrix<TYPE> &m);

	/*!
		+= operator
	*/
	const _rmatrix<TYPE> &operator += (const _rmatrix<TYPE> &m);
	
	/*!
		-= operator
	*/
	const _rmatrix<TYPE> &operator -= (const _rmatrix<TYPE> &m);
	
	/*!
	&nbsp;*= operator with scalar
	*/
	const _rmatrix<TYPE> &operator *= (TYPE c);

	/*!
		/= operator with scalar
	*/
	const _rmatrix<TYPE> &operator /= (TYPE c);

	/*!
		addition operator
	*/
	_rmatrix<TYPE> operator + (const _rmatrix<TYPE> &m) const;

	/*!
		subtraction operator
	*/
	_rmatrix<TYPE> operator - (const _rmatrix<TYPE> &m) const;

	/*!
		matrix multiplication operator
	*/
	_rmatrix<TYPE> operator * (const _rmatrix<TYPE> &m) const;

	/*!
		matrix multiplication operator
		A | B = A * ~B
	*/
	_rmatrix<TYPE> operator | (const _rmatrix<TYPE> &m) const;

	/*!
		matrix multiplication operator
		A ^ B = ~A * B
	*/
	_rmatrix<TYPE> operator ^ (const _rmatrix<TYPE> &m) const;

	/*!
		scalar multiplication operator
	*/
	_rmatrix<TYPE> operator * (TYPE c) const;

	/*!
		scalar division operator
	*/
	_rmatrix<TYPE> operator / (TYPE c) const;

	/*!
		matrix inversion operator
		A \% B = Inv(A) * B
	*/
	_rmatrix<TYPE> operator % (const _rmatrix<TYPE> &m) const;

	/*!
		matrix inversion operator
		A & B = Inv(~A) * B
	*/
	_rmatrix<TYPE> operator & (const _rmatrix<TYPE> &m) const;

	/*!
		scalar multiplication operator
	*/
	friend _rmatrix<TYPE> operator * (TYPE c, _rmatrix<TYPE> m)
	{		
		int n = m.row * m.col;
		TYPE *_m = m.element;
		while ( n-- ) *(_m++) *= c;
		return m;
	}

	/*!
		standard output operator
	*/
	friend ostream &operator <<<TYPE>(ostream &os, const _rmatrix<TYPE> &m);

	//@}

	/** @name Attributes
	*/
	//@{
 		/*!
		get a number of rows.
	*/
	int RowSize(void) const;
	
	/*!
		get a number of columns.
	*/
	int ColSize(void) const;
	
	//@}
	
	/*!
		get the maximum element in x.
		\param idx if not NULL, an index of the maximum element
	*/
	friend TYPE MaxVec<TYPE>(const _rmatrix<TYPE> &x, int *idx);

	/*!
		get the minimum element in x.
		\param idx if not NULL, an index of the minimum element
	*/
	friend TYPE MinVec<TYPE>(const _rmatrix<TYPE> &x, int *idx);
	
	/*!
		get a r-by-c zero matrix.
	*/
	friend _rmatrix<TYPE> Zeros<TYPE>(int r, int c);

	/*!
		get a r-by-c random matrix. call srand() to set a seed for random-number generation.
	*/
	friend _rmatrix<TYPE> Rand<TYPE>(int r, int c);

	/*!
		get the r-by-c identity matrix.
	*/
	friend _rmatrix<TYPE> Eye<TYPE>(int r, int c);

	/*!
		get the n-by-n identity matrix.
	*/
	friend _rmatrix<TYPE> Eye<TYPE>(int n);
	
	/*!
		get a squared sum of A.
	*/
	friend TYPE SquareSum<TYPE>(const _rmatrix<TYPE> &A);

	/*!
		get an absolute sum of A.
	*/
	friend TYPE AbsSum<TYPE>(const _rmatrix<TYPE> &A);

	/*!
		get an inner product of x and y, where x and y are assumed to be vectors.
	*/
	friend TYPE Inner<TYPE>(const _rmatrix<TYPE> &x, const _rmatrix<TYPE> &y);

	/*!
		get a quadratic product.
		Quadratic(x,A,y) = \f$\sum x_i A_{ij} y_j\f$.
		x and y will be assumed as vectors.
	*/
	friend TYPE Quadratic<TYPE>(const _rmatrix<TYPE> &x, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &y);

	/*!
		get the Frobenius norm of A.
		FNorm(A) = \f$\sqrt{\sum A_{ij}^2}\f$.
	*/
	
	friend TYPE FNorm<TYPE>(const _rmatrix<TYPE> &A);
	
	/*!
		get a determinant of \a m.
	*/
	friend TYPE Det<TYPE>(_rmatrix<TYPE> A);

	/*!
		get the sum of the diagonal elements of A.
	*/
	friend TYPE Trace<TYPE>(const _rmatrix<TYPE> &A);
	
	/*!
		diagonal matrix and diagonals of a matrix
		If m is a n-vector, Diag(m) is a sqaure matrix with the elements of m on the diagonal.
		If m is a square matrix, Diag(m) is the main diagonal of \a m.
	*/
	friend _rmatrix<TYPE> Diag<TYPE>(_rmatrix<TYPE> &m);
	
	/*!
		the inverse of the square matrix A
	*/
	friend _rmatrix<TYPE> Inv<TYPE>(const _rmatrix<TYPE> &A);
	
	/*!
		matrix multiplication
		C = A * B
	*/
	friend void AMultB<TYPE>(_rmatrix<TYPE> &C, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &B);

	/*!
		matrix multiplication
		C = A * ~B
	*/
	friend void AMultBt<TYPE>(_rmatrix<TYPE> &C, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &B);

	/*!
		matrix multiplication
		C = ~A * B
	*/
	friend void AtMultB<TYPE>(_rmatrix<TYPE> &C, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &B);

	/*!
		get abs(m)
	*/
	friend _rmatrix<TYPE> Abs<TYPE>(const _rmatrix<TYPE> &M);

	/** @name Solving linear equations
	*/
	//@{ 	
	/*!
		solve linear equation, A x = b.
		\param[out] x the solution
		\return true if the solution exists.
	*/
	friend bool SolveAxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b);
	friend bool SolveAxEqualB_<TYPE>(_rmatrix<TYPE> &A, _rmatrix<TYPE> &B);

	/*!
		solve linear equation, ~A x = b.
		\param[out] x the solution
		\return true if the solution exists.
	*/
	friend bool SolveAtxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b);

	/*!
		solve linear equation, A x = b, where \a A should be positive definite.
		\param[out] x the solution
		\return true if the solution exists.
		\note elements of A change after return.
	*/
	friend bool SolvePosDefAxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b);

	/*!
		solve linear equation, A x = b using singular value decomposition.
		\param[out] x the solution
		\return true if the solution exists.
	*/
	friend bool SVDSolveAxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b);

	/*!
		solve linear equation, A x = b using QR decomposition.
		\param[out] x the solution
		\return true if the solution exists.
	*/
	friend bool QRSolveAxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b);

	/*!
		solve linear equation, A x = b using fixed point problem.
		\param[out] x the solution
		\return true if the solution exists.
	*/
	friend bool FixedPointSolveAxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b);
	//@}

	friend int GaussElimination<TYPE>(_rmatrix<TYPE> &, _rmatrix<int> &, _rmatrix<int> &);
	friend int GaussElimination<TYPE>(_rmatrix<TYPE> &, _rmatrix<int> &, _rmatrix<int> &, TYPE);

	/** @name Singualr value decomposition
	*/
	//@{
 	/*!
		get singular values of A.
	*/
	friend _rmatrix<TYPE> SVD<TYPE>(const _rmatrix<TYPE> &A);

	/*!
		singular value decomposition
		A = U * Diag(S) * ~V
	*/
	friend void SVD<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &U, _rmatrix<TYPE> &S, _rmatrix<TYPE> &V);

	friend int Rank<TYPE>(const _rmatrix<TYPE> &, TYPE);
	
	/*!
		get an number of linearly independent rows or columns of A.
	*/
	friend int Rank<TYPE>(const _rmatrix<TYPE> &A);
	//@}

	/*!
		solve linear complementarity problem using Lemke's method
		\note Find \f$f\f$ such that \f$0 < f \perp Af + b > 0 \f$.
	*/
	friend bool LemkeSolveLCP<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &f, const _rmatrix<TYPE> &b);
	
	/*!
		solve linear complementarity problem using Dantzig's method
		\note Find \f$f\f$ such that \f$0 < f \perp Af + b > 0 \f$.
	*/
	friend bool DantzigSolveLCP<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &f, const _rmatrix<TYPE> &b);
	
	/*!
		solve linear complementarity problem using the fixed point problem
		\note Find \f$f\f$ such that \f$0 < f \perp Af + b > 0 \f$.
	*/
	friend bool FixedPointSolveLCP<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &f, const _rmatrix<TYPE> &b);

	friend bool SORSolveLCP<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, const TYPE &w, int iter);

	/*!
		solve Ax=b using Successive over-relaxation method.
		\note w = SOR factor, iter = number of iteration
	*/
	friend bool SORSolveAxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, const TYPE &w, int iter);

private:
	int		 row;
	int		 col;
	int		 mem_size;
	TYPE	*element;
};

#include <VP/rmatrix3.inl>

#endif
