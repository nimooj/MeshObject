// MeshObject.cpp : Defines the exported functions for the DLL.
//

#include "pch.h"
#include "framework.h"
#include "MeshObject.h"


// This is an example of an exported variable
MESHOBJECT_API int nMeshObject=0;

// This is an example of an exported function.
MESHOBJECT_API int fnMeshObject(void)
{
    return 0;
}

// This is the constructor of a class that has been exported.
MeshObject::MeshObject()
{
	m_Vertices = new std::vector<mjVec3*>();
	m_Texels = new std::vector<mjPos2*>();
	m_Normals = new std::vector<mjVec3*>();

	// m_Edges = new std::vector<mjEdge*>();
	// m_Faces = new std::vector<mjFace*>();

	m_VertBuf = std::vector<float>();
	m_IndexBuf = std::vector<int>();

    return;
}

MeshObject::MeshObject(const MeshObject& m) {
	m_Vertices = m.m_Vertices;
	m_Texels = m.m_Texels;
	m_Normals = m.m_Normals;
}

MeshObject::MeshObject(std::vector<mjVec3 *> *v, std::vector<mjPos2 *> *t, std::vector<mjVec3 *> *n, 
	// std::vector<mjEdge *> *e, std::vector<mjFace *> *f, 
	std::vector<float> vb, std::vector<int> ib) {
	m_Vertices = v;
	m_Texels = t;
	m_Normals = n;
	// m_Edges = e;
	// m_Faces = f;
	m_VertBuf = vb;
	m_IndexBuf = ib;
}


MeshObject::~MeshObject() {
	delete[] m_Vertices;
	delete[] m_Texels;
	delete[] m_Normals;

	// delete[] m_Edges;
	// delete[] m_Faces;
}

bool MeshObject::LoadObj(const char* fname) {
	FILE *fp;
	fopen_s(&fp, fname, "r");

	if (!fp) {
		printf("Loading %s failed...\n", fname);
		return false;
	}
	else {
		printf("Importing %s...\n", fname);
	}

	char tag[256];
	bool isTexture = false;
	bool isNormal = false;

	while (fscanf_s(fp, "%s", tag, 256) != EOF) {
		// mtlib-tag
		if (!strcmp(tag, "mtlib")) {
		}

		// g-tag
		if (!strcmp(tag, "g")) {
		}

		// usemtl-tag 
		if (!strcmp(tag, "usemtl")) {
		}


		// v-tag
		if (!strcmp(tag, "v")) {
			float x, y, z;
			fscanf_s(fp, "%f%f%f", &x, &y, &z);
			mjVec3 *nVert = new mjVec3(x, y, z);
			// nVert->m_Idx = m_Vertices->size();
			AddVertex(nVert);
		}

		// vt-tag
		if (!strcmp(tag, "vt")) {
			float u, v;
			isTexture = true;
			fscanf_s(fp, "%f%f", &u, &v);
			mjPos2 *nTex = new mjPos2(u, v);
			AddTexel(nTex);
		}

		// vn-tag
		if (!strcmp(tag, "vn")) {
			float x, y, z;
			isNormal = true;
			fscanf_s(fp, "%f%f%f", &x, &y, &z);
			mjVec3 *nNorm = new mjVec3(x, y, z);
			AddNormal(nNorm);
		}

		// f-tag
		if (!strcmp(tag, "f")) {
			std::vector<int> vIndices, tIndices, nIndices;

			// Get a whole line
			char line[256];
			fgets(line, 256, fp);
			char *data = line;

			// Reading line...
			while (true) {
				// Move past spaces and newlines
				while(*data == ' ' || *data == '\n')
					data++;

				// When a line ends, break
				if (!strcmp(data, ""))
					break;


				// Read index info to a buffer
				char buffer[256];
				sscanf_s(data, "%s", buffer, 256);
				data += strlen(buffer);


				char separator;
				int vidx, tidx, nidx;

				// when both texture and normal
				if (isTexture && isNormal) {
					sscanf_s(buffer, "%d%c%d%c%d", &vidx, &separator, 1, &tidx, &separator, 1, &nidx);
					vidx = (vidx < 0) ? -vidx : vidx;
					tidx = (tidx < 0) ? -tidx : tidx;
					nidx = (nidx < 0) ? -nidx : nidx;

					vIndices.push_back(vidx - 1);
					tIndices.push_back(tidx - 1);
					nIndices.push_back(nidx - 1);
					continue;
				}
				// when only texture
				else if (isTexture && !isNormal) {
					sscanf_s(buffer, "%d%c%d", &vidx, &separator, 1, &tidx);
					vidx = (vidx < 0) ? -vidx : vidx;
					tidx = (tidx < 0) ? -tidx : tidx;
					vIndices.push_back(vidx - 1);
					tIndices.push_back(tidx - 1);
					continue;
				}
				// when only normal 
				else if (!isTexture && isNormal) {
					sscanf_s(buffer, "%d%c%d", &vidx, &separator, 1, &nidx);
					vidx = (vidx < 0) ? -vidx : vidx;
					nidx = (nidx < 0) ? -nidx : nidx;
					vIndices.push_back(vidx - 1);
					nIndices.push_back(nidx - 1);
					continue;
				}
				// when only vIndex
				else if (!isTexture && !isNormal) {
					sscanf_s(buffer, "%d", &vidx);
					vidx = (vidx < 0) ? -vidx : vidx;
					vIndices.push_back(vidx - 1);
					continue;
				}
					
			}
			// End of reading line...

			/////// Add Face Info to MeshObject
			mjVec3 *v0 = (*m_Vertices)[vIndices[0]],
				*v1 = (*m_Vertices)[vIndices[1]],
				*v2 = (*m_Vertices)[vIndices[2]];

			mjPos2 *t0 = 0, *t1 = 0, *t2 = 0; 
			if (isTexture) {
				t0 = (*m_Texels)[tIndices[0]];
				t1 = (*m_Texels)[tIndices[1]];
				t2 = (*m_Texels)[tIndices[2]];
			}

			mjVec3 *n0 = 0, *n1 = 0, *n2 = 0;
			if (isNormal) {
				n0 = (*m_Normals)[nIndices[0]];
				n1 = (*m_Normals)[nIndices[1]];
				n2 = (*m_Normals)[nIndices[2]];
			}
			else {
				// Generate normal if not provided
				// Assign triangle's normal
				// mjVec3 *n = new mjVec3((v1->m_Coord - v0->m_Coord) ^ (v2->m_Coord - v0->m_Coord));

				//normalize
				/*
				n->x /= n->length();
				n->y /= n->length();
				n->z /= n->length();

				n0 = n1 = n2 = n;
				*/
			}

			/*
			mjFace *face = new mjFace(v0, v1, v2, t0, t1, t2, n0, n1, n2);
			face->m_Idx = m_Faces->size();
			AddFace(face);
			*/


			/////// Add Edge
			/*
			mjEdge *e0 = new mjEdge(v0, v1), 
				*e1 = new mjEdge(v1, v2), 
				*e2 = new mjEdge(v2, v0);

			AddEdge(e0);
			AddEdge(e1);
			AddEdge(e2);
			*/
		}
	}

	UpdateVertBuff();
	UpdateIndexBuff();
}


bool MeshObject::WriteObj(const char* fname) {
	printf("Writing to %s...\n", fname);

	std::ofstream oFile(fname);

	/*
	for (mjVec3 *v : *m_Vertices) {
		oFile << "v " <<  v->m_Coord->x << " " << v->m_Coord->y << " " << v->m_Coord->z << std::endl;
	}
	*/

	/*
	for (mjVec2 *t : *m_texels) {
		oFile << "vt " << t->x << " " << t->y << std::endl;
	}

	for (mjVec3 *n : *m_normals) {
		oFile << "vn " << n->x << " " << n->y << " " << n->z << std::endl;
	}
	*/

	/*
	for (int i = 0; i < m_Faces->size(); i++) {
		int idx0 = (*m_Faces)[i]->GetVertIdx(0) + 1;
		int idx1 = (*m_Faces)[i]->GetVertIdx(1) + 1;
		int idx2 = (*m_Faces)[i]->GetVertIdx(2) + 1;

		oFile << "f " << idx0 << " " << idx1 << " " << idx2 << std::endl;
	}
	*/

	oFile.close();

	return true;
}


void MeshObject::AddVertex(mjVec3 *v) {
	m_Vertices->push_back(v);
}

void MeshObject::AddTexel(mjPos2 *t) {
	m_Texels->push_back(t);
}

void MeshObject::AddNormal(mjVec3 *n) {
	m_Normals->push_back(n);
}


/*
void MeshObject::AddEdge(mjVec3 *v0, mjVec3 *v1) {

}

void MeshObject::AddEdge(mjEdge *e0) {

}

void MeshObject::AddFace(mjFace *f) {
	m_Faces->push_back(f);
}
*/


void MeshObject::UpdateVertBuff() {
	/*
	m_VertBuf.clear();

	for (mjVec3 *v : *m_Vertices) {
		m_VertBuf.push_back(v->m_Coord->x);
		m_VertBuf.push_back(v->m_Coord->y);
		m_VertBuf.push_back(v->m_Coord->z);
	}
	*/
}

void MeshObject::UpdateIndexBuff() {
	/*
	m_IndexBuf.clear();

	for (mjFace *f : *m_Faces) {
		m_IndexBuf.push_back(f->GetVertIdx(0));
		m_IndexBuf.push_back(f->GetVertIdx(1));
		m_IndexBuf.push_back(f->GetVertIdx(2));
	}
	*/
}

