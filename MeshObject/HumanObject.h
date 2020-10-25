#ifdef HUMANOBJECT_EXPORTS
#define HUMANOBJECT_API __declspec(dllexport)
#else
#define HUMANOBJECT_API __declspec(dllimport)
#endif

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#define STB_IMAGE_IMPLEMENTATION   // use of stb functions once and for all
#include "stb_image.h"

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>


#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "JointList.h"
#include "BoneList.h"

#include "MeshObject.h"


#define Female 0
#define Male 1

#define MAX(x, y)	(x > y ? x : y)
#define MIN(x, y)	(x < y ? x : y)


class mjVertex;
class mjNormal;
class mjEdge;
class mjFace; // Tri face

class mjJoint;
class mjBone;
class mjSkeleton;

class mjLandmark;

class mjTexture;
class mjMaterial;

class mjBoundingBox;

class HumanObject;


class mjVertex {
public: 
	int m_Idx;

	mjPos3 *m_Coord;
	mjPos2 *m_Texel;
	mjNormal *m_Normal;

	int m_Segment;
public: 
	mjVertex(float x = 0.0f, float y = 0.0f, float z = 0.0f);
	mjVertex(const mjVertex& cpy);
	~mjVertex();
};


class mjNormal {
public:
	int m_Idx;

	mjVec3 *m_Dir;

public:
	mjNormal(float x = 0.0f, float y = 0.0f, float z = 0.0f);
	mjNormal(mjVec3 *n);
	mjNormal(const mjNormal& cpy);
	~mjNormal();
};


class mjEdge {
public:
	std::vector<mjVertex *> *m_Verts;

	mjEdge *m_Prev;
	mjEdge *m_Next;

	// 어느 face에 속해있는지
	mjFace *m_Face;

public:
	mjEdge(mjVertex *v0, mjVertex *v1);
	~mjEdge();

};


class mjFace {
public:
	//////////////////// Member Variables
	int m_Idx;

	mjMaterial *m_Material;
	std::string m_Group;

	// vertices
	std::vector<mjVertex *> *m_Verts;

	// textures
	std::vector<mjPos2 *> *m_Texels;

	// face normals
	std::vector<mjNormal *> *m_Normals;

	// edges
	std::vector<mjEdge *> *m_Edges;


public:
	//////////////////// Constructor & Deconstructor
	mjFace(
		mjVertex *v0, mjVertex *v1, mjVertex *v2,
		mjPos2 *t0, mjPos2 *t1, mjPos2 *t2,
		mjNormal *n0, mjNormal *n1, mjNormal * n2
	);
	mjFace(
		mjVertex *v0, mjVertex *v1, mjVertex *v2,
		mjPos2 *t0, mjPos2 *t1, mjPos2 *t2,
		mjNormal *n0, mjNormal *n1, mjNormal * n2,
		mjMaterial *mtl,
		std::string group
	);
	mjFace(const mjFace &cpy);
	~mjFace();



	//////////////////// Member Variables
	mjVertex* GetVert(int idx);
	mjPos3* GetVertPos(int idx);
	int GetVertIdx(int idx);
};


class mjJoint {
public:
	mjJoint(float _x = 0.0f, float _y = 0.0f, float _z = 0.0f);
	mjJoint(const mjJoint &cpy);
	~mjJoint();

public:
	int m_Idx;
	mjPos3 *m_Coord;

	HumanObject *m_Human;

	void SetHuman(HumanObject *h);
};

class mjBone {
public:
	mjBone();
	mjBone(mjJoint *upper, mjJoint *lower);
	mjBone(const mjBone &cpy);
	~mjBone();

public:
	HumanObject *m_Human;
	mjSkeleton *m_Skeleton;

	bool isLeaf;

	int m_Idx;
	int m_ChildNum;
	mjLine3 *m_Bone;
	float m_Length;

	mjJoint *m_UpperJoint = NULL, *m_LowerJoint = NULL;

	mjBone *m_Parent;
	std::vector<mjBone *> *m_Children = NULL;


	// Vertices effected by the bones
	// and its corresponding weights
	std::vector<mjVertex *> *m_VertList;
	std::vector<float>  *m_WeightList;

public:
	void SetParent(mjBone *p);
	void SetChild(mjBone *c);

	void SetHuman(HumanObject *h);
	void SetSkeleton(mjSkeleton *s);

	void AddVertexWeight(mjVertex *v, float w);
};


class mjSkeleton {
public:
	mjSkeleton();
	mjSkeleton(const mjSkeleton& cpy);
	~mjSkeleton();

public:
	HumanObject *m_Human;
	std::vector<mjBone *> *m_Bones;
	std::vector<mjJoint *> *m_Joints;

public:
	void SetHierarchy(int type = 0);
	void AddJoint(int idx, mjJoint *joint);
	void AddBone(int idx, mjBone *bone);
};

#define Length 0
#define Girth 1

class mjLandmark {
public:
	mjLandmark(const char* name = "", int type = 0, float lvl = 0.0, float val = 0.0f);
	mjLandmark(const mjLandmark& cpy);
	~mjLandmark();

public:
	HumanObject *m_Human;

	// tmp construction for now (20. 8. 22)
	int m_Idx;

	const char* m_Name;
	int m_Type;
	float m_Level; 
	float m_Value;

	std::vector<int> m_SegmentIdx;
	std::vector<int> m_VertIdx;


public:
	void SetName(char *lname);
	void SetLandmark(mjLandmark *l);
	void SetHuman(HumanObject *h);
	void SetSegment(int idx);

	int GetIndex();
	int GetIndex(char *lname);
	const char* GetName();
	const char* GetName(int idx);
	std::vector<int> GetSegments();
	float GetSize();

	bool HasSegment(int idx);
	float CalcSize();

	void Deform(float nval = 0.0, float upperBound = 0.0, float lowerBound = 0.0);
	void DeformLengthType(float nval = 0.0);
	void DeformGirthType(float nval = 0.0, float upperBound = 0.0, float lowerBound = 0.0);
};


class mjTexture {
public:
	mjTexture(int id, std::string fname);
	~mjTexture();

	void LoadTexture();

public:
	std::string m_Filename;

    unsigned int m_Idx;
    int m_Width;
    int m_Height;
    int m_Channels;

	unsigned char *m_TextureData;
};


class mjMaterial {
public:
	/*! \brief 재질의 인덱스 */
	int m_Idx;

	/*! \brief 재질의 이름 */
	std::string m_Name;

	/*! \brief 재질의 주변광 반사 계수 */
	float m_Ambient[4];

	/*! \brief 재질의 난반사 계수 */
	float m_Diffuse[4];

	/*! \brief 재질의 전반사 계수 */
	float m_Specular[4];

	/*! \brief 재질의 전반사 지수 */
	float m_Shiness;

	/*! \brief 재질이 사용하는 텍스처에 대한 포인터(텍스처는 여러 재질이 공유 할 수 있음) */
	mjTexture *m_Texture;

public:
	// 생성자 및 소멸자
	mjMaterial(std::string name);
	mjMaterial(const mjMaterial &cpy);
	~mjMaterial();


	// 재질 설정 함수
	void SetAmbient(float r, float g, float b, float a = 1.0f);
	void SetDiffuse(float r, float g, float b, float a = 1.0f);
	void SetSpecular(float r, float g, float b, float a = 1.0f);
	void SetShiness(float s);
	void SetTexture(mjTexture *pTexture);
	void Enable();
	void Disable();
};

class mjBoundingBox {
public:
	float m_MinX, m_MaxX;
	float m_MinY, m_MaxY;
	float m_MinZ, m_MaxZ;

public:
	mjBoundingBox();
	mjBoundingBox(const mjBoundingBox &cpy);
	~mjBoundingBox();

public:
	void SetBounds(float minX = 0.0, float maxX = 100.0f, float minY = 0.0, float maxY = 100.0f, float minZ = 0.0, float maxZ = 100.0f);
};

#define RENDER_SHADE 0
#define RENDER_TEXTURE 1

class HUMANOBJECT_API HumanObject : public MeshObject {
public:
	/////// Constructor & Deconstructor
	HumanObject();
	HumanObject(const HumanObject &cpy);
	~HumanObject();


public:
	/////// Member variables
	const char *fname;

	int m_Gender;
	int m_RenderType;
	
	float m_MinX, m_MaxX, m_MinY, m_MaxY, m_MinZ, m_MaxZ;

	mjBoundingBox *m_BoundingBox;

	std::vector<mjVertex *> *m_Vertices;
	std::vector<mjPos2 *> *m_Texels;
	std::vector<mjNormal *> *m_Normals;

	std::vector<mjEdge *> *m_Edges;
	std::vector<mjFace *> *m_Faces;

	std::vector<mjTexture *> *m_Textures;
	std::vector<mjMaterial *> *m_Materials;

	// buffer for rendering
	// std::vector<int> m_IndexBuf;
	std::map<std::string, std::vector<int>> m_IndexBuf;

	// std::vector<float> m_VertBuf;
	std::map<std::string, std::vector<float>> m_VertBuf;
	std::map<std::string, std::vector<float>> m_TexBuf;
	std::map<std::string, std::vector<float>> m_NormBuf;
	// std::vector<float> m_TexBuf;
	// std::vector<float> m_NormBuf;
	


	mjSkeleton *m_Skeleton;

	std::vector<mjLandmark *> *m_Landmarks;

	// tmp for sizing functions (20. 8. 22)
	// 해당 Bone segment에 속하는 vertex들을 갖는다
	std::vector<mjVertex *> m_Segment[Bone_Num];

public:
	/////// Member functions
	bool LoadObj(const char* fname);
	bool LoadObjMtl(const char* fname);
	bool LoadHuman(const char* fname);
	bool LoadJoints(const char* fname);
	bool LoadLandmarks(const char* fname);


	void AddVertex(mjVertex *v);
	void AddTexel(mjPos2 *t);
	void AddNormal(mjNormal *n);

	void AddEdge(mjVertex *v0, mjVertex *v1);
	void AddEdge(mjEdge *e0);

	void AddFace(mjFace *f);

	void AddTexture(mjTexture *pTexture);
	void AddMaterial(mjMaterial *pMaterial);

	void AddLandmark(mjLandmark *pLandmark);


	void UpdateJoints();
	void UpdateLandmarks();


	void UpdateIndexBuff();
	void UpdateVertBuff();
	void UpdateTexBuff();
	void UpdateNormBuff();


	bool SetSkeleton();
	bool AssignWeight();
	bool SetSegment();


	bool WriteObj(const char* fname);
	void WriteHuman(const char* fname);

	// void initHuman();

	void SetMale();
	void SetFemale();


	mjMaterial* GetMaterial(std::string fname);
	mjTexture* GetTexture(std::string fname);




	/////// Measure 
	// 측정항목 개수
	int GetLandmarkNum();
	// i번째 측정항목 이름
	void GetLandmarkName(int i, char* buffer);
	// i번째 측정항목 값
	float GetLandmarkValue(int i);
	// lname 측정항목 값
	float GetLandmarkValue(char* lname);
	// sizes 순서대로 치수 변형
	void SetSizes(float *sizes);
	// i번째 측정항목 치수 변형
	void SetSize(int i, float value);
	// lname 측정항목 치수 변형
	void SetSize(char* lname, float value);

	// i번째 측정항목 측정경로 길이(점 개수)
	// int GetSizePathLength(int i);
	// i번째 측정항목의 n번째 점 좌표를 coord에 저장
	// void getSizePathPos(int l, int n, float* coord);
	// i번째 측정항목 좌표 coord에 한 번 받기
	// void getSizePath(int l, float *coord);


	/////// Geometry
	int GetVertNum();
	int GetFaceNum();
	// i번째 정점의 좌표 반환
	void GetVert(int i, float *coord);
	// 전체 정점 좌표 반환
	void GetVerts(float *coord);
	// i번째 face의 번호를 반환
	void GetIndex(int i, int *node);
	// 모든 face 번호를 반환
	void GetIndices(int *node);

	/////// Bounding / Collision

	/////// Pose
	void SetTPose(int s);

	/////// Visual
	void Render();
	void RenderTexture();
};

// extern HUMANOBJECT_API int mHumanObject;

// HUMANOBJECT_API int fnHumanObject(void);
