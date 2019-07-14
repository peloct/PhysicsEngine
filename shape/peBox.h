#pragma once

#include"peShape.h"
#include"../common/peSettings.h"
#include"../common/peMath.h"

class Renderer;

class Box : public Shape
{
public:
	Box(float32 x, float32 y, float32 z) : Shape(), x(x), y(y), z(z)
	{
		assert(x > 0.0f && y > 0.0f && z > 0.0f);

		Vector3 halfExt(x * 0.5f, y * 0.5f, z * 0.5f);

		vertices[0] = Vector3(-halfExt.x, -halfExt.y, -halfExt.z);
		vertices[1] = Vector3( halfExt.x, -halfExt.y, -halfExt.z);
		vertices[2] = Vector3( halfExt.x,  halfExt.y, -halfExt.z);
		vertices[3] = Vector3(-halfExt.x,  halfExt.y, -halfExt.z);
		vertices[4] = Vector3(-halfExt.x, -halfExt.y,  halfExt.z);
		vertices[5] = Vector3( halfExt.x, -halfExt.y,  halfExt.z);
		vertices[6] = Vector3( halfExt.x,  halfExt.y,  halfExt.z);
		vertices[7] = Vector3(-halfExt.x,  halfExt.y,  halfExt.z);

		edges[0] = Edge(0, 0, 1, Vector3(1, 0, 0));
		edges[1] = Edge(1, 1, 2, Vector3(0, 1, 0));
		edges[2] = Edge(2, 2, 3, Vector3(-1, 0, 0));
		edges[3] = Edge(3, 3, 0, Vector3(0, -1, 0));

		edges[4] = Edge(4, 4, 5, Vector3(1, 0, 0));
		edges[5] = Edge(5, 5, 6, Vector3(0, 1, 0));
		edges[6] = Edge(6, 6, 7, Vector3(-1, 0, 0));
		edges[7] = Edge(7, 7, 4, Vector3(0, -1, 0));

		edges[8] = Edge(8, 0, 4, Vector3(0, 0, 1));
		edges[9] = Edge(9, 1, 5, Vector3(0, 0, 1));
		edges[10] = Edge(10, 2, 6, Vector3(0, 0, 1));
		edges[11] = Edge(11, 3, 7, Vector3(0, 0, 1));

		for (int i = 0; i < 12; ++i)
		{
			edges[i].halfEdgeAtoB.edge = &edges[i];
			edges[i].halfEdgeAtoB.pair = &edges[i].halfEdgeBtoA;
			edges[i].halfEdgeBtoA.edge = &edges[i];
			edges[i].halfEdgeBtoA.pair = &edges[i].halfEdgeAtoB;
		}

		// -z
		faces[0] = Face(0, Vector3(0, 0, -1), &edges[3].halfEdgeBtoA);
		edges[3].halfEdgeBtoA.next = &edges[2].halfEdgeBtoA;
		edges[2].halfEdgeBtoA.next = &edges[1].halfEdgeBtoA;
		edges[1].halfEdgeBtoA.next = &edges[0].halfEdgeBtoA;
		edges[0].halfEdgeBtoA.next = &edges[3].halfEdgeBtoA;

		// z
		faces[1] = Face(1, Vector3(0, 0, 1), &edges[4].halfEdgeAtoB);
		edges[4].halfEdgeAtoB.next = &edges[5].halfEdgeAtoB;
		edges[5].halfEdgeAtoB.next = &edges[6].halfEdgeAtoB;
		edges[6].halfEdgeAtoB.next = &edges[7].halfEdgeAtoB;
		edges[7].halfEdgeAtoB.next = &edges[4].halfEdgeAtoB;

		// -x
		faces[2] = Face(2, Vector3(-1, 0, 0), &edges[8].halfEdgeAtoB);
		edges[8].halfEdgeAtoB.next = &edges[7].halfEdgeBtoA;
		edges[7].halfEdgeBtoA.next = &edges[11].halfEdgeBtoA;
		edges[11].halfEdgeBtoA.next = &edges[3].halfEdgeAtoB;
		edges[3].halfEdgeAtoB.next = &edges[8].halfEdgeAtoB;

		// x
		faces[3] = Face(3, Vector3(1, 0, 0), &edges[1].halfEdgeAtoB);
		edges[1].halfEdgeAtoB.next = &edges[10].halfEdgeAtoB;
		edges[10].halfEdgeAtoB.next = &edges[5].halfEdgeBtoA;
		edges[5].halfEdgeBtoA.next = &edges[9].halfEdgeBtoA;
		edges[9].halfEdgeBtoA.next = &edges[1].halfEdgeAtoB;

		// -y
		faces[4] = Face(4, Vector3(0, -1, 0), &edges[0].halfEdgeAtoB);
		edges[0].halfEdgeAtoB.next = &edges[9].halfEdgeAtoB;
		edges[9].halfEdgeAtoB.next = &edges[4].halfEdgeBtoA;
		edges[4].halfEdgeBtoA.next = &edges[8].halfEdgeBtoA;
		edges[8].halfEdgeBtoA.next = &edges[0].halfEdgeAtoB;

		// y
		faces[5] = Face(5, Vector3(0, 1, 0), &edges[2].halfEdgeAtoB);
		edges[2].halfEdgeAtoB.next = &edges[11].halfEdgeAtoB;
		edges[11].halfEdgeAtoB.next = &edges[6].halfEdgeBtoA;
		edges[6].halfEdgeBtoA.next = &edges[10].halfEdgeBtoA;
		edges[10].halfEdgeBtoA.next = &edges[2].halfEdgeAtoB;

		for (int i = 0; i < 6; ++i)
		{
			HalfEdge* it = faces[i].halfEdge;
			for (int j= 0; j < 4; ++j)
			{
				it->face = &faces[i];
				it = it->next;
			}
		}
	}

	Vector3 getExtent() const
	{
		return Vector3(x, y, z);
	}

	Type getType() const override;
	void computeMassData(MassData* massData, float32 density) const override;
	void computeAABB(AABB* aabb, const Vector3& position, const Matrix3x3& orientationMatrix) const override;

	const Face* getFaces() const { return faces; }
	const Edge* getEdges() const { return edges; }
	const Vector3* getVertices() const { return vertices; }

private:
	friend class Mesh;

	float32 x, y, z;
	Face faces[6];
	Edge edges[12];
	Vector3 vertices[8];
};