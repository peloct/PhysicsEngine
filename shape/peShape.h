#pragma once

#include"../common/peMath.h"
#include"../collision/peCollision.h"

struct HalfEdge;
struct Edge;

struct MassData
{
	float32 mass;
	Matrix3x3 inertiaTensor;
	Vector3 centerOfMass;
};

struct Face
{
	Face() : index(-1), normal(), halfEdge(nullptr) {}
	Face(int32 index, const Vector3& normal, HalfEdge* edges) : index(index), normal(normal), halfEdge(edges) {}

	int32 index;
	Vector3 normal;
	HalfEdge* halfEdge;
};

struct HalfEdge
{
	HalfEdge() : edge(nullptr), face(nullptr), pair(nullptr), next(nullptr) {}
	HalfEdge(Edge* edge) : edge(nullptr), face(nullptr), pair(nullptr), next(nullptr) {}

	Edge* edge;
	Face* face;
	HalfEdge* pair;
	HalfEdge* next;

	int32 getFrom() const;
	int32 getTo() const;
	Vector3 getDirection() const;
};

struct Edge
{
	Edge() : index(-1), halfEdgeAtoB(), halfEdgeBtoA(), vertexA(-1), vertexB(-1), dirAtoB() {}
	Edge(int32 index, int32 vertexA, int32 vertexB, const Vector3& dirAtoB) :
		index(index), halfEdgeAtoB(), halfEdgeBtoA(), vertexA(vertexA), vertexB(vertexB), dirAtoB(dirAtoB) {}

	int32 index;
	HalfEdge halfEdgeAtoB;
	HalfEdge halfEdgeBtoA;
	int32 vertexA;
	int32 vertexB;
	Vector3 dirAtoB;
};

class Shape
{
public:
	enum Type
	{
		box,
		shapeCount
	};

	virtual Type getType() const = 0;
	virtual void computeMassData(MassData* massData, float32 density) const = 0;
	virtual void computeAABB(AABB* aabb, const Vector3& position, const Matrix3x3& orientationMatrix) const = 0;
};