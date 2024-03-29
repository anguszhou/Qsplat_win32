#ifndef QSPLAT_MAKE_QTREE_V11_H
#define QSPLAT_MAKE_QTREE_V11_H
/*
Szymon Rusinkiewicz

qsplat_make_qtree_v11.h
Data structure for the tree.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include "mempool.h"
#include "qsplat_util.h"
#include <string>

//Size of memory of an instance of struct is totle of all the member variables which have their own memory
//Size of memory of an instance of union is the most variable'size and all member variables shall it and 
//every time of write will flush previous value.  
struct QTree_Node {
	point pos; // pos is an instance of float[3]
	float r;
	vec norm; //norm is an instance of float[3]
	float normcone;
	union {
		QTree_Node * child[4];
		struct {
			int refcount;
			int remap;
			short col_tmp[3];
		} m;	// What @$@#! on the C++ committee
		// put in anon unions but not structs?
	};
	color col; //col is an instance of unsigned char[3]


	static PoolAlloc memPool;//PoolAlloc is a class defined in mempool.h
	void *operator new(size_t n) { return memPool.alloc(n); }
	void operator delete(void *p, size_t n) { memPool.free(p,n); }
};


class QTree {
private:
	QTree_Node *root, *leaves;
	int numleaves;
	QTree_Node **leafptr;
	bool havecolor;
	void InitLeaves();
	QTree_Node *BuildLeaves(int begin, int end);
	QTree_Node *CombineNodes(QTree_Node *n1, QTree_Node *n2);
	QTree_Node *CombineNodes(QTree_Node *n1, QTree_Node *n2, QTree_Node *n3);
	QTree_Node *CombineNodes(QTree_Node *n1, QTree_Node *n2, QTree_Node *n3, QTree_Node *n4);
	QTree_Node *CombineLeaves(int begin, int end);
	int Partition(int begin, int end);
	int Num_Children(QTree_Node *n);
	bool Has_Grandchildren(QTree_Node *n);
	int Nodesize(QTree_Node *n);
	int Treesize(QTree_Node *n = NULL);

public:
	QTree(int _numleaves, QTree_Node *_leaves, bool _havecolor) :
	  numleaves(_numleaves), leaves(_leaves), havecolor(_havecolor)
	  {
		  InitLeaves();
	  }
	  void BuildTree();
	  void Write(const char *qsfile, const std::string &comments);
};

#endif
