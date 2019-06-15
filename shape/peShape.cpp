#include"peShape.h"

int32 HalfEdge::getFrom() const
{
	if (this == &edge->halfEdgeAtoB)
		return edge->vertexA;
	else
		return edge->vertexB;
}

int32 HalfEdge::getTo() const
{
	if (this == &edge->halfEdgeAtoB)
		return edge->vertexB;
	else
		return edge->vertexA;
}

Vector3 HalfEdge::getDirection() const
{
	if (this == &edge->halfEdgeAtoB)
		return edge->dirAtoB;
	else
		return -edge->dirAtoB;
}