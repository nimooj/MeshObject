#ifdef MESHOBJECT_EXPORTS
#define MESHOBJECT_API __declspec(dllexport)
#else
#define MESHOBJECT_API __declspec(dllimport)
#endif

#pragma once

#include "mjType.h"

#include <string>
#include <iostream>
#include <fstream>


// This class is exported from the dll
class MESHOBJECT_API MeshObject {
public:
	MeshObject(void);
	MeshObject(const MeshObject& m);
	MeshObject(
		std::vector<mjVec3 *> *v,
		std::vector<mjPos2 *> *t,
		std::vector<mjVec3 *> *n,
		// std::vector<mjEdge *> *e,
		// std::vector<mjFace *> *f,
		std::vector<float> vb,
		std::vector<int> ib);
	~MeshObject();

	// member variables
	const char *fame;

	std::vector<mjVec3 *> *m_Vertices;
	std::vector<mjPos2 *> *m_Texels;
	std::vector<mjVec3 *> *m_Normals;

	// std::vector<mjEdge *> *m_Edges;
	// std::vector<mjFace *> *m_Faces;

	// buffer for rendering
	std::vector<float> m_VertBuf;
	std::vector<int> m_IndexBuf;

	// member functions
	void AddVertex(mjVec3 *v);
	void AddTexel(mjPos2 *t);
	void AddNormal(mjVec3 *n);

	// void AddEdge(mjVec3 *v0, mjVertex *v1);
	// void AddEdge(mjEdge *e0);

	// void AddFace(mjFace *f);

	void UpdateVertBuff();
	void UpdateIndexBuff();


	bool LoadObj(const char* fname);
	bool WriteObj(const char* fname);
};


extern MESHOBJECT_API int nMeshObject;

MESHOBJECT_API int fnMeshObject(void);
