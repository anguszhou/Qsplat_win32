#ifndef MEMPOOL_H
#define MEMPOOL_H
/*
Szymon Rusinkiewicz

mempool.h
Replacement memory management for a class using a memory pool.

Sample usage:
	class MyClass {
	private:
		static PoolAlloc memPool;
	public:
		void *operator new(size_t n) { return memPool.alloc(n); }
		void operator delete(void *p, size_t n) { memPool.free(p,n); }
		// ...
	};

	PoolAlloc MyClass::memPool(sizeof(MyClass));

Does *no* error checking.  Make sure itemsize is larger than sizeof(void *).

Based on the description of the Pool class in _Effective C++_ by Scott Meyers
*/

#include <vector>
#include <algorithm>

#define POOL_MEMBLOCK 4088


class PoolAlloc {
private:
	//size_t is a kind has none of mechine's business and have size of local unsigned int or int64
	size_t itemsize;
	void *freelist;//null kind pointer which can contain other kind pointer
	void grow_freelist()
	{
		int n = POOL_MEMBLOCK / itemsize;
		
		//new operator:call operator new() to distribute size demanded and call constructor.
		
		//::operator new()[==new ClassA()]:overload the new function and only distribute size demanded but not call constructor
		//when overloaded,its first para must be size_t means distribute size you wanted
		// and can carry other parameters and return is void* 

		freelist = ::operator new(n * itemsize);
		for (int i=0; i < n-1; i++)
			* (void **)((char *)freelist+itemsize*i) = (char *)freelist + itemsize*(i+1);
		* (void **)((char *)freelist+itemsize*(n-1)) = 0;
	}

public:
	PoolAlloc(size_t size) : itemsize(size), freelist(0) {}
	void *alloc(size_t n)
	{
		if (n != itemsize)
			return ::operator new(n);
		if (!freelist)
			grow_freelist();
		void *next = freelist;
		freelist = * (void **)next;
		return next;
	}
	void free(void *p, size_t n)
	{
		if (!p)
			return;
		else if (n != itemsize)
			::operator delete(p);
		else {
			* (void **)p = freelist;
			freelist = p;
		}
	}
	void sort_freelist()
	{
		if (!freelist)
			return;
		std::vector<void *> v;
		void *p;
		for (p = freelist; * (void **)p; p = * (void **)p)
			v.push_back(p);
		std::sort(v.begin(), v.end());
		p = freelist = v[0];
		for (int i=1; i < v.size(); i++) {
			* (void **)p = v[i];
			p = * (void **)p;
		}
		* (void **)p = NULL;
	}
};

#endif
