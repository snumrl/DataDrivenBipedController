#ifndef __array_
#define __array_

/*
	_array Template Class

	2003.Sep.2.
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr

	template <class TYPE> class _array
	method : 
		_array();
		_array(int sz);
		_array(const _array<TYPE>&ary);
		~_array();
		int size(void) const;
		void resize(int n);
		const TYPE &operator [] (int i) const;
		TYPE &operator [] (int i);
		_array &operator = (const _array<TYPE>&ary);
		int find(const TYPE &value) const;
		void check_push_back(const TYPE &value);
		void push_back(const TYPE &value);
		void push_back(const _array<TYPE>&ary);
		void remove(int idx);
*/

#include <assert.h>

#define RESERVE 1

#ifndef NULL
#define NULL 0
#endif

/*!
	\class _array
	\brief a simple port of STL vector template
	
	_array is a template class to facilitate managing data in an array.
*/
template <class TYPE> class _array
{
public :
	_array()
	{
		_sz = _mem_sz = 0;
		_first = NULL;
	}

	/*!
		constructor
		\param sz a number of item slots in the array
	*/
	_array(int sz)
	{
		assert(sz >= 0 && "_array(int) -> invalid _sz");
		
		_mem_sz = (_sz = sz) << RESERVE;
		if ( _mem_sz > 0 ) _first = new TYPE [_mem_sz];
		else _first = NULL;
	}
	
	/*!
		copy constructor
	*/
	_array(const _array<TYPE> &ary)
	{
		_first = new TYPE [_mem_sz = (_sz = ary._sz) << RESERVE];
		int n = _sz;
		TYPE *base_i = _first, *ary_base_i = ary._first;
		while ( n-- ) *base_i++ = *ary_base_i++;
	}
	
	~_array()
	{
		delete [] _first;
	}
	
	/*!
		\return a number of items in the array
	*/
	int size(void) const
	{
		return _sz;
	}
	
	/*!
		adjust a _sz of the array.
		\param n a number of item slot in the array
		\param keep_value In order to keep values in the array after adjusting a _sz, set 'keep_value' to be true.
		If 'keep_value' is false, values in the array after adjunsting a _sz may change.
	*/
	void resize(int n, bool keep_value = false)
	{
		assert(n >= 0 && "_array::resize(int, bool) -> invalid _sz");
		
		if ( n > _mem_sz )
		{
			TYPE *tmp = new TYPE [_mem_sz = n << RESERVE];
			if ( keep_value )
			{
				int n = _sz;
				TYPE *tmp_i = tmp, *base_i = _first;
				while ( n-- ) *tmp_i++ = *base_i++;
			}
			delete [] _first;
			_first = tmp;		
		}
		_sz = n;
	}

	void clear(void)
	{
		resize(0);
	}
	
	bool empty(void) const
	{
		return (_sz == 0);
	}

	const TYPE &operator [] (int i) const
	{
		assert(i >= 0 && i < _sz && "_array::operator[int] -> invalid index");
		return _first[i];
	}
	
	/*!
		access to the i-th item in the array.
	*/
	TYPE &operator [] (int i)
	{
		assert(i >= 0 && i < _sz && "_array::operator[int] -> invalid index");
		return _first[i];
	}
	
	/*!
		equal operator
	*/
	_array<TYPE> &operator = (const _array<TYPE> &ary)
	{
		if ( ary._sz > _mem_sz )
		{
			delete [] _first;
			_first = new TYPE [_mem_sz = ary._sz << RESERVE];
		} 
		_sz = ary._sz;
		
		int n = _sz;
		TYPE *base_i = _first, *ary_base_i = ary._first;
		while ( n-- ) *base_i++ = *ary_base_i++;

		for ( int i = 0; i < _sz; i++ ) _first[i] = ary._first[i];
		return *this;
	}

	/*!
		find value in the array.
		\return index count where value is found.
		If there is no item equal to value, return -1.
	*/
	int find(const TYPE &value) const
	{
		int i = 0, n = _sz;
		TYPE *base_i = _first;
		while ( n-- ) 
		{
			if ( *base_i++ == value ) return i;
			i++;
		}
		return -1;
	}
	
	/*!
		add an item at tail.
	*/
	void push_back(const TYPE &value)
	{
		int n = _sz + 1;
		if ( n > _mem_sz )
		{
			TYPE *tmp = new TYPE [_mem_sz = n << RESERVE], *tmp_i = tmp, *base_i = _first;
			while ( --n ) *tmp_i++ = *base_i++;
			delete [] _first;
			_first = tmp;
		}
		_first[_sz++] = value;
	}

	/*!
		if 'value' is in the array, else add it at tail.
	*/
	void check_push_back(const TYPE &value)
	{
		int n = _sz;
		TYPE *base_i = _first;
		while ( n-- ) if ( *base_i++ == value ) return;
		
		n = _sz + 1;
		if ( n > _mem_sz )
		{
			TYPE *tmp = new TYPE [_mem_sz = n << RESERVE], *tmp_i = tmp, *base_i = _first;
			while ( --n ) *tmp_i++ = *base_i++;
			delete [] _first;
			_first = tmp;
		}
		_first[_sz++] = value;
	}

	/*!
		add another array at tail.
	*/
	void push_back(const _array<TYPE> &ary)
	{
		if ( _sz + ary._sz > _mem_sz )
		{
			int n = _sz;
			TYPE *tmp = new TYPE [_mem_sz = (_sz + ary._sz) << RESERVE], *tmp_i = tmp, *base_i = _first;
			while ( n-- ) *tmp_i++ = *base_i++;
			delete [] _first;
			_first = tmp;
		}

		int n = ary._sz;
		TYPE *base_i = _first + _sz, *ary_base_i = ary._first;
		_sz += n;
		while ( n-- ) *base_i++ = *ary_base_i++;
	}

	/*!
		pop out the idx-th item.
	*/
	void remove(int idx)
	{
		if ( idx < 0 || idx >= _sz ) return;
		
		int n = _sz - idx;
		TYPE *base_i = _first + idx, *base_ip1 = base_i + 1;
		while ( --n ) *base_i++ = *base_ip1++;
		_sz--;
	}

private :

	int		 _sz;
	int		 _mem_sz;
	TYPE	*_first;
};

#endif
