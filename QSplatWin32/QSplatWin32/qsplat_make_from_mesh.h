#ifndef QSPLAT_MAKE_FROM_MESH_H
#define QSPLAT_MAKE_FROM_MESH_H
/*
Szymon Rusinkiewicz

qsplat_make_from_mesh.h
Code for reading in a triangle mesh from a ply file and initializing the
QSplat data structure.

Copyright (c) 1999-2000 The Board of Trustees of the
Leland Stanford Junior University.  All Rights Reserved.
*/

#include "qsplat_make_qtree_v11.h"
#include "ann/ANN/ANN.h"

//extern : variables or functions' declaration or definition exists in other place.
extern bool read_ply(const char *plyfile,
					 int &numleaves, QTree_Node * &leaves,
					 int &numfaces, face * &faces,
					 bool &havecolor,
					 std::string &comments);
extern void find_normals(int numleaves, QTree_Node *leaves,
						 int numfaces, const face *faces);
extern void merge_nodes(int &numleaves, QTree_Node *leaves,
						int &numfaces, face *faces,
						bool havecolor, float thresh);
extern void find_splat_sizes(int numleaves, QTree_Node *leaves,
							 int numfaces, const face *faces);
extern void compute_splat_sizes(int numleaves, QTree_Node *leaves);
extern void swap_point(int nPts, ANNkd_tree *kdTree, int numleaves, QTree_Node *leaves, bool side);

#endif
