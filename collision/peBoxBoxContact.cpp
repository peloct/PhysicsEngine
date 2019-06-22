#include"peBoxBoxContact.h"
#include"../memory/peBlockAllocator.h"
#include"../peFixture.h"
#include"../shape/peBox.h"

#include<new>

#define PARELLEL_TOL 0.0001f

#define CLIPPING_TOL 0.01f
#define CLIPPING_MIN_DIST_SQR 0.1f * 0.1f

#define FACEVERTEX_COL_TOL 0.01f
#define EDGEEDGE_COL_TOL 0.01f

// nearestFace 에는 fixture1의 면
// nearestHalfEdge 는 가장 가까운 점으로 향하는 fixture2의 임의 edge
float32 getMaximalSeparationFaceVertex(const Face** nearestFace, const HalfEdge** nearestHalfEdge, Fixture* fixture1, Fixture* fixture2, const Matrix4x4& WTofixture1, const Matrix4x4& fixture1ToFixture2)
{
	Box* box1 = (Box*)fixture1->getShape();
	Box* box2 = (Box*)fixture2->getShape();

	const Face* faces1 = box1->getFaces();
	const Vector3* vertices1 = box1->getVertices();
	const Vector3* vertices2 = box2->getVertices();

	Vector3 relPos = WTofixture1.transformDirection(fixture2->getBodyPosition() - fixture1->getBodyPosition());
	float32 maxSep = -MaxFloat32;

	for (int i = 0; i < 6; ++i)
	{
		if (Vector3::dot(relPos, faces1[i].normal) <= 0)
			continue;

		Vector3 facePos = fixture1ToFixture2.transformPoint(vertices1[faces1[i].halfEdge->getFrom()]);
		Vector3 faceDir = fixture1ToFixture2.transformDirection(faces1[i].normal);
		faceDir.normalize();

		float32 origin = Vector3::dot(faceDir, facePos);
		const HalfEdge* halfEdge = &box1->getEdges()[0].halfEdgeAtoB;

		// minDot 에 갱신이 없다면, halfEdge 의 from 이 minDot 을 만드는 것이므로 pair 로 바꿔준다.
		const HalfEdge* edgeToMin = halfEdge->pair;

		float32 dirDot = Vector3::dot(faceDir, halfEdge->getDirection());

		if (dirDot > 0)
			halfEdge = halfEdge->pair;

		int32 minVertex = halfEdge->getTo();
		edgeToMin = halfEdge;

		halfEdge = halfEdge->next;
		dirDot = Vector3::dot(faceDir, halfEdge->getDirection());

		if (dirDot < 0)
		{
			minVertex = halfEdge->getTo();
			edgeToMin = halfEdge;
			halfEdge = halfEdge->next->pair->next;
		}
		else
		{
			halfEdge = halfEdge->pair->next;
		}

		dirDot = Vector3::dot(faceDir, halfEdge->getDirection());
		if (dirDot < 0)
		{
			minVertex = halfEdge->getTo();
			edgeToMin = halfEdge;
		}

		float32 minDot = Vector3::dot(faceDir, vertices2[minVertex]);
		float32 separation = minDot - origin;
		if (separation > maxSep)
		{
			(*nearestFace) = &faces1[i];
			(*nearestHalfEdge) = edgeToMin;
			maxSep = separation;

			if (separation > 0)
				return MaxFloat32;
		}
	}

	return maxSep;
}

float32 getMaximalSeparationEdgeEdge
(const Edge** nearestEdge1, const Edge** nearestEdge2, Fixture* fixture1, Fixture* fixture2, const Matrix4x4& fixture1ToFixture2)
{
	Box* box1 = (Box*)fixture1->getShape();
	Box* box2 = (Box*)fixture2->getShape();
	const Vector3* vertices1 = box1->getVertices();
	const Vector3* vertices2 = box2->getVertices();
	const Edge* edges = box1->getEdges();
	const Face* faces = box1->getFaces();

	Vector3 relPos = fixture1ToFixture2.transformPoint(Vector3(0, 0, 0));
	relPos * -1;

	const HalfEdge* dirEdge[3];

	dirEdge[0] = &edges[2].halfEdgeBtoA;
	dirEdge[1] = &edges[3].halfEdgeBtoA;
	dirEdge[2] = &edges[11].halfEdgeAtoB;

	Vector3 dirs1[3];
	dirs1[0] = Vector3(fixture1ToFixture2.m00, fixture1ToFixture2.m10, fixture1ToFixture2.m20);
	dirs1[1] = Vector3(fixture1ToFixture2.m01, fixture1ToFixture2.m11, fixture1ToFixture2.m21);
	dirs1[2] = Vector3(fixture1ToFixture2.m02, fixture1ToFixture2.m12, fixture1ToFixture2.m22);

	Vector3 dirs2[3];
	dirs2[0] = Vector3(1, 0, 0);
	dirs2[1] = Vector3(0, 1, 0);
	dirs2[2] = Vector3(0, 0, 1);

	float32 maxSep = -MaxFloat32;

	for (int i = 0; i < 3; ++i)
	{
		Vector3 edgesPoses[4];

		HalfEdge* edgeOfFace = dirEdge[i]->next->pair;
		for (int j = 0; j < 4; ++j)
		{
			edgesPoses[j] = fixture1ToFixture2.transformPoint(vertices1[edgeOfFace->getFrom()]);
			edgeOfFace = edgeOfFace->next;
		}

		for (int j = 0; j < 3; ++j)
		{
			Vector3 normal = Vector3::cross(dirs1[i], dirs2[j]);

			if (normal.sqrMagnitude() < PARELLEL_TOL * PARELLEL_TOL)
				continue;

			normal.normalize();

			if (Vector3::dot(normal, relPos) > 0)
				normal *= -1;

			float32 maxDot = -MaxFloat32;
			Edge* selectedEdge1 = nullptr;

			edgeOfFace = dirEdge[i]->next->pair;
			for (int k = 0; k < 4; ++k)
			{
				float32 dot = Vector3::dot(edgesPoses[k], normal);

				if (maxDot < dot)
				{
					maxDot = dot;
					selectedEdge1 = edgeOfFace->pair->next->edge;
				}

				edgeOfFace = edgeOfFace->next;
			}

			float32 minDot = MaxFloat32;
			Edge* selectedEdge2 = nullptr;

			edgeOfFace = dirEdge[j]->next->pair;
			for (int k = 0; k < 4; ++k)
			{
				float32 dot = Vector3::dot(vertices2[edgeOfFace->getFrom()], normal);

				if (minDot > dot)
				{
					minDot = dot;
					selectedEdge2 = edgeOfFace->pair->next->edge;
				}

				edgeOfFace = edgeOfFace->next;
			}

			float32 separation = minDot - maxDot;
			if (maxSep < separation)
			{
				maxSep = separation;
				(*nearestEdge1) = selectedEdge1;
				(*nearestEdge2) = selectedEdge2;

				if (separation > 0)
					return MaxFloat32;
			}
		}
	}

	return maxSep;
}

void clipPoints(int32* pointCount, Vector3* points, const Vector3& clipPlaneNormal, const Vector3& clipPlanePoint)
{
	int bufferCount = 0;
	Vector3 buffer[8];

	int count = *pointCount;
	Vector3 prevPoint = points[0];
	float32 prevDot = Vector3::dot(clipPlaneNormal, (points[0] - clipPlanePoint));
	bool isPrevOut = prevDot > 0;

	for (int i = 1; i <= count; ++i)
	{
		Vector3 point = points[i % count];
		float32 dot = Vector3::dot(clipPlaneNormal, (point - clipPlanePoint));
		bool createClipPoint = false;
		bool isOut = dot > 0;

		if ((isOut && !isPrevOut) || (!isOut && isPrevOut))
		{
			Vector3 pointToInsert;

			if (peAbsf(dot - prevDot) >= CLIPPING_TOL)
			{
				Vector3 dp = point - prevPoint;
				float32 a = Vector3::dot(clipPlanePoint - prevPoint, clipPlaneNormal);
				float32 b = Vector3::dot(dp, clipPlaneNormal);
				pointToInsert = prevPoint + dp * (a / b);
			}
			else
			{
				pointToInsert = point;
			}

			if (bufferCount == 0 || (buffer[bufferCount - 1] - pointToInsert).sqrMagnitude() > CLIPPING_MIN_DIST_SQR)
				buffer[bufferCount++] = pointToInsert;
		}

		if (!isOut)
		{
			if (bufferCount == 0 || (buffer[bufferCount - 1] - point).sqrMagnitude() > CLIPPING_MIN_DIST_SQR)
				buffer[bufferCount++] = point;
		}

		prevPoint = point;
		isPrevOut = dot > 0;
		prevDot = dot;
	}

	for (int i = 0; i < bufferCount; ++i)
		points[i] = buffer[i];
	(*pointCount) = bufferCount;
}

void BoxBoxContact::evaluate()
{
	contactPointCount = 0;
	contactCacheKey.contactCount = 0;

	Matrix4x4 fixtureA2W = fixtureA->getBodyLocalToWorld();
	Matrix4x4 fixtureB2W = fixtureB->getBodyLocalToWorld();
	Matrix4x4 w2FixtureA = fixtureA2W.getInverseTransform();
	Matrix4x4 w2FixtureB = fixtureB2W.getInverseTransform();

	Matrix4x4 fixtureB2A = w2FixtureA * fixtureB2W;
	Matrix4x4 fixtureA2B = w2FixtureB * fixtureA2W;

	const Vector3* verticesA = ((Box*)fixtureA->getShape())->getVertices();
	const Vector3* verticesB = ((Box*)fixtureB->getShape())->getVertices();

	if ((fixtureA->getBodyPosition() - fixtureB->getBodyPosition()).sqrMagnitude() < 0.00001f)
		return;

	const Face* faceA = nullptr;
	const HalfEdge* halfEdgeB = nullptr;
	float32 sep1 = getMaximalSeparationFaceVertex(&faceA, &halfEdgeB, fixtureA, fixtureB, w2FixtureA, fixtureA2B);

	if (sep1 > 0)
		return;

	const Face* faceB = nullptr;
	const HalfEdge* halfEdgeA = nullptr;
	float32 sep2 = getMaximalSeparationFaceVertex(&faceB, &halfEdgeA, fixtureB, fixtureA, w2FixtureB, fixtureB2A);

	if (sep2 > 0)
		return;

	const Edge* edgeA = nullptr;
	const Edge* edgeB = nullptr;
	float32 sep3 = getMaximalSeparationEdgeEdge(&edgeA, &edgeB, fixtureA, fixtureB, fixtureA2B);

	if (sep3 > 0)
		return;

	// edge-edge contact 먼저 체크한다.
	// sep 이 가장 크더라도 사용하기에 적합하지 않은 케이스가 있기 때문.
	if (peMaxf(sep1, sep2) + EDGEEDGE_COL_TOL < sep3)
	{
		Vector3 aDir = fixtureA2W.transformDirection(edgeA->dirAtoB);
		Vector3 bDir = fixtureB2W.transformDirection(edgeB->dirAtoB);

		Vector3 a0 = fixtureA2W.transformPoint(verticesA[edgeA->vertexA]);
		Vector3 b0 = fixtureB2W.transformPoint(verticesB[edgeB->vertexA]);
		Vector3 a1 = fixtureA2W.transformPoint(verticesA[edgeA->vertexB]);
		Vector3 b1 = fixtureB2W.transformPoint(verticesB[edgeB->vertexB]);

		Vector3 normal = Vector3::cross(aDir, bDir);
		normal.normalize();

		Vector3 aTanDir = Vector3::cross(aDir, normal);
		Vector3 bTanDir = Vector3::cross(bDir, normal);

		if (Vector3::dot(b0 - a0, aTanDir) * Vector3::dot(b1 - a0, aTanDir) <= 0
			&& Vector3::dot(a0 - b0, bTanDir) * Vector3::dot(a1 - b0, bTanDir) <= 0)
		{
			Vector3 dp = b0 - a0;
			Vector3 cToD = normal * Vector3::dot(normal, dp);

			Vector3 c = dp - bDir * Vector3::dot(bDir, dp) - cToD;
			float32 cMag = c.magnitude();

			c = aDir * (cMag / Vector3::dot(aDir, c / cMag)) + a0;
			Vector3 d = c + cToD;

			assert(normal.sqrMagnitude() >= PARELLEL_TOL * PARELLEL_TOL);

			contactPointCount = 1;
			contactFaceOwner = ContactFaceOwner::fixtureA;
			localPlanePoint = w2FixtureA.transformPoint(c);
			localNormal = w2FixtureA.transformDirection(normal);
			localNormal.normalize();
			localTangent1 = edgeA->dirAtoB;
			localTangent2 = Vector3::cross(localNormal, localTangent1);

			contactPoints[0].localPoint = w2FixtureB.transformPoint(d);
			contactPoints[0].normalImpulse = 0;
			contactPoints[0].tangentImpulse.setZero();

			contactCacheKey.faceOwner = ContactFaceOwner::fixtureA;
			contactCacheKey.contactCount = 1;
			contactCacheKey.a = -edgeA->index;
			contactCacheKey.b = -edgeB->index;
			return;
		}
	}

	const Face* referenceFace;
	const HalfEdge* incidentEdge;
	Matrix4x4* refernceL2W = nullptr;
	Matrix4x4* incidentL2W = nullptr;
	Matrix4x4* incidentW2L = nullptr;
	const Vector3* refFaceVertices;
	const Vector3* incFaceVertices;
	ContactFaceOwner faceOnwer;

	if (sep2 <= sep1 + FACEVERTEX_COL_TOL)
	{
		faceOnwer = ContactFaceOwner::fixtureA;
		referenceFace = faceA;
		incidentEdge = halfEdgeB;
		refernceL2W = &fixtureA2W;
		incidentL2W = &fixtureB2W;
		incidentW2L = &w2FixtureB;
		refFaceVertices = verticesA;
		incFaceVertices = verticesB;
	}
	else
	{
		faceOnwer = ContactFaceOwner::fixtureB;
		referenceFace = faceB;
		incidentEdge = halfEdgeA;
		refernceL2W = &fixtureB2W;
		incidentL2W = &fixtureA2W;
		incidentW2L = &w2FixtureA;
		refFaceVertices = verticesB;
		incFaceVertices = verticesA;
	}

	Vector3 faceNormal = refernceL2W->transformDirection(referenceFace->normal);
	const HalfEdge* curEdge = incidentEdge;
	Face* incidentFace = nullptr;
	float32 minDot = MaxFloat32;

	for (int i = 0; i < 3; ++i)
	{
		float32 dot = Vector3::dot(faceNormal, incidentL2W->transformDirection(curEdge->face->normal));

		if (dot < minDot)
		{
			minDot = dot;
			incidentFace = curEdge->face;
		}

		curEdge = curEdge->next->pair;
	}

	int32 clippedPointCount = 4;
	Vector3 clippedPoints[8];
	Vector3 facePoint;

	curEdge = incidentFace->halfEdge;
	for (int i = 0; i < 4; ++i)
	{
		clippedPoints[i] = incidentL2W->transformPoint(incFaceVertices[curEdge->getFrom()]);;
		curEdge = curEdge->next;
	}

	curEdge = referenceFace->halfEdge;
	for (int i = 0; i < 4; ++i)
	{
		Vector3 clipPlaneNormal = refernceL2W->transformDirection(Vector3::cross(curEdge->getDirection(), referenceFace->normal));
		Vector3 clipPlanePoint = refernceL2W->transformPoint(refFaceVertices[curEdge->getFrom()]);
		clipPoints(&clippedPointCount, clippedPoints, clipPlaneNormal, clipPlanePoint);
		curEdge = curEdge->next;
		facePoint = clipPlanePoint;
	}

	int nextIdx = 0;
	for (int i = 0; i < clippedPointCount; ++i)
		if (Vector3::dot(clippedPoints[i] - facePoint, faceNormal) < CLIPPING_TOL)
			clippedPoints[nextIdx++] = clippedPoints[i];

	contactPointCount = nextIdx;
	contactFaceOwner = faceOnwer;
	localPlanePoint = refFaceVertices[referenceFace->halfEdge->getFrom()];
	localNormal = referenceFace->normal;
	localTangent1 = referenceFace->halfEdge->getDirection();
	localTangent2 = Vector3::cross(localNormal, localTangent1);

	for (int i = 0; i < contactPointCount; ++i)
	{
		contactPoints[i].localPoint = incidentW2L->transformPoint(clippedPoints[i]);
		contactPoints[i].normalImpulse = 0;
		contactPoints[i].tangentImpulse.setZero();
	}

	contactCacheKey.faceOwner = faceOnwer;
	contactCacheKey.contactCount = contactPointCount;
	contactCacheKey.a = referenceFace->index;
	contactCacheKey.b = incidentFace->index;
}

Contact* BoxBoxContact::createFunc(BlockAllocator* blockAllocator, Fixture* fixtureA, Fixture* fixtureB)
{
	void* mem = blockAllocator->allocate(sizeof(BoxBoxContact));
	return new (mem) BoxBoxContact(fixtureA, fixtureB);
}

void BoxBoxContact::deleteFunc(BlockAllocator* blockAllocator, Contact* contact)
{
	blockAllocator->free(contact, sizeof(BoxBoxContact));
}