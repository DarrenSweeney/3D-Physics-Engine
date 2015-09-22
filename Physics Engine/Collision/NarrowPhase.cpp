#include "NarrowPhase.h"
#include <cstdlib>
#include <assert.h>

using namespace Physics_Engine;

void CollisionPrimitive::calculateInternals()
{
	transform = body->getTransform() * offset;
}

static inline real transformToAxis(const CollisionBox &box, const Vector3 &axis)
{
	/*
		Compute the projection interval radius.
	*/
	return box.halfSize.x * real_abs(axis * box.getAxis(0)) +
		box.halfSize.y * real_abs(axis * box.getAxis(1)) +
		box.halfSize.z * real_abs(axis * box.getAxis(2));
}

/*
	This function checks if the two boxes overlap
	along the given axis. The final parameter toCenter
	is used to pass in the vector between the boxes center
	points, to avoid having to recalculate it each time.
*/
static inline bool overLapOnAxis(const CollisionBox &one, const CollisionBox &two,
	const Vector3 &axis, const Vector3 &toCenter)
{
	// Project the half-size of one onto axis.
	real oneProject = transformToAxis(one, axis);
	real twoProject = transformToAxis(two, axis);

	// Project this onto the axis.
	real distance = real_abs(toCenter * axis);

	// Check for overlap
	return (distance < oneProject + twoProject);
}
/*
	This preprocessor definition is only used as a convenience
	in the boxAndBox intersection method.
*/
#define TEST_OVERLAP(axis) overLapOnAxis(one, two, (axis), toCenter)

bool IntersectionTests::boxAndBox(const CollisionBox & one, const CollisionBox & two)
{
	// Find the vector between the two centers.
	Vector3 toCenter = two.getAxis(3) - one.getAxis(3);

	return(
		// Check on box one's axis first.
		TEST_OVERLAP(one.getAxis(0)) && 
		TEST_OVERLAP(one.getAxis(1)) &&
		TEST_OVERLAP(one.getAxis(2)) &&

		// And on two's
		TEST_OVERLAP(two.getAxis(0)) &&
		TEST_OVERLAP(two.getAxis(1)) &&
		TEST_OVERLAP(two.getAxis(2)) &&

		// Now on the cross products. Eedge-Edge axis test.
		TEST_OVERLAP(one.getAxis(0) % two.getAxis(0)) &&
		TEST_OVERLAP(one.getAxis(0) % two.getAxis(1)) &&
		TEST_OVERLAP(one.getAxis(0) % two.getAxis(2)) &&
		TEST_OVERLAP(one.getAxis(1) % two.getAxis(0)) &&
		TEST_OVERLAP(one.getAxis(1) % two.getAxis(1)) &&
		TEST_OVERLAP(one.getAxis(1) % two.getAxis(2)) &&
		TEST_OVERLAP(one.getAxis(2) % two.getAxis(0)) &&
		TEST_OVERLAP(one.getAxis(2) % two.getAxis(1)) &&
		TEST_OVERLAP(one.getAxis(2) % two.getAxis(2))
		);
}
#undef TEST_OVERLAP

bool IntersectionTests::boxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane)
{
	// Working out the projection radius of the box onto the plane direction.
	real projectedRadius = transformToAxis(box, plane.normal);

	// Compute distance of box center from the plane.
	real distance = plane.normal * box.getAxis(3) - projectedRadius;

	/*
		Intersection occurs when distance falls with [-r, r] interval.
	*/
	return real_abs(distance) <= projectedRadius;
}

/*
	This function checks if two boxes overlap along the given axis,
	returing the amount of overlap.
	The final parameter toCenter is used to pass in the vector
	between the boxes center points, to avoid recalculate it each time.
*/
static inline real penetrationOnAxis(const CollisionBox &one, const CollisionBox &two,
	const Vector3 &axis, const Vector3 &toCenter)
{
	/*
		Calculates the half-length of each box along the axis.
	*/
	real oneProjection = transformToAxis(one, axis);
	real twoProjection = transformToAxis(two, axis);

	// toCenter is the vector from the center of the 
	// first box to the center of the second
	// Calculates the distance between each half lenght box along that axis.
	real distance = real_abs(toCenter * axis);

	/*
		Return the overlap (i.e, positive indicates
		overlap, negative indicates seperation). 
		The ammount of interpenetration is returned.
	*/
	return oneProjection + twoProjection - distance;
}

static inline bool tryAxis(const CollisionBox &one, const CollisionBox &two,
	Vector3 axis, const Vector3 &toCenter, unsigned index,

	// These values may be updated
	real &smallestPenetration,
	unsigned &smallestCase)
{
	// Make sure we have a normalized axis, and don't check almost parallel axes
	if (axis.squareMagnitude() < 0.0001)
		return true;

	axis.normalise();

	real penetration = penetrationOnAxis(one, two, axis, toCenter);

	if (penetration < 0)
		return false;

	if (penetration < smallestPenetration)
	{
		smallestPenetration = penetration;
		smallestCase = index;
	}
	return true;
}

static inline Vector3 contactPoint(const Vector3 &pOne, const Vector3 &dOne,
	real oneSize, const Vector3 &pTwo, const Vector3 &dTwo, real twoSize,
	
	/*
		If this is true and the contact point is outside
		the edge (in this case of an edge-face contact) then
		we use one's midpoint, otherwise we use two's.
	*/
	bool useOne)
{
	Vector3 toSt, cOne, cTwo;
	real dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
	real denom, mua, mub;

	smOne = dOne.squareMagnitude();
	smTwo = dTwo.squareMagnitude();
	dpOneTwo = dTwo * dOne;

	toSt = pOne - pTwo;
	dpStaOne = dOne * toSt;
	dpStaTwo = dTwo * toSt;

	denom = smOne * smTwo - dpOneTwo * dpOneTwo;

	// Zero denominator indicates parrallel lines
	if (real_abs(denom) < 0.0001f) {
		return useOne ? pOne : pTwo;
	}

	mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
	mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

	/*
		 If either of the edges has the nearest point out
		 of bounds, then the edges aren't crossed, we have
		 an edge-face contact. Our point is on the edge, which
		 we know from the useOne parameter.
	*/
	if (mua > oneSize ||
		mua < -oneSize ||
		mub > twoSize ||
		mub < -twoSize)
	{
		return useOne ? pOne : pTwo;
	}
	else
	{
		cOne = pOne + dOne * mua;
		cTwo = pTwo + dTwo * mub;

		return cOne * 0.5 + cTwo * 0.5;
	}
}

/*
	Determines which face is involved and then looks at 
	the orientation of the second box to see which vertex
	would lie closet to the first box.
*/
void fillPointFaceBoxBox(const CollisionBox &one, const CollisionBox &two,
	const Vector3 &toCenter, CollisionData *data, unsigned best, real pen)
{
	/*
		This method is called when we know that a vertex from
		box two is in contact with box one.
	*/

	Contact *contact = data->contacts;

	/*
		We know which axis the collision is on (i.e best),
		but we need to work out which of the two faces on
		this axis.
	*/
	Vector3 normal = one.getAxis(best);
	if (one.getAxis(best) * toCenter > 0)
	{
		normal = normal * -1.0f;
	}

	/*
		Work out which vertex of box two we are colliding with.
		Using toCenter doesn't work!
	*/
	Vector3 vertex = two.halfSize;
	if (two.getAxis(0) * normal < 0)
		vertex.x = -vertex.x;
	if (two.getAxis(1) * normal < 0)
		vertex.y = -vertex.y;
	if (two.getAxis(2) * normal < 0)
		vertex.z = -vertex.z;

	// Create the contact data
	contact->contactNormal = normal;
	contact->penetration = pen;
	contact->contactPoint = two.getTransform() * vertex;
	contact->setBodyData(one.body, two.body, data->friction, data->restitution);
}

unsigned CollisionDectector::sphereAndSphere(const CollisionSphere &one,
	const CollisionSphere &two,
	CollisionData *data)
{
	// Make sure we have contacts.
	if (data->contactsLeft <= 0)
		return 0;

	// Cache the sphere positions.
	Vector3 positionOne = one.getAxis(3);
	Vector3 positionTwo = two.getAxis(3);

	// Find the vector between the objects.
	Vector3 midline = positionOne - positionTwo;
	real size = midline.magnitude();

	// See if it is large enough.
	if (size <= 0.0f || size >= one.radius + two.radius)
	{
		return 0;
	}

	// We manually create the normal, because we have the
	// size in hand.
	Vector3 normal = midline * (((real)1.0) / size);

	Contact *contact = data->contacts;
	contact->contactNormal = normal;
	contact->contactPoint = positionOne + midline * (real)0.5;
	contact->penetration = (one.radius + two.radius - size);
	contact->setBodyData(one.body, two.body, data->friction, data->restitution);

	data->addContacts(1);

	return 1;
}

unsigned CollisionDectector::sphereAndHalfSpace(const CollisionSphere &sphere,
	const CollisionPlane &plane,
	CollisionData *data)
{
	// Make sure we have enough contacts.
	if (data->contactCount <= 0)
		return 0;

	// Cache the sphere position.
	Vector3 spherePos = sphere.getAxis(3);

	// Find the distance from the plane.
	real ballDistance = plane.normal * spherePos - sphere.radius - plane.offset;

	if (ballDistance >= 0)
		return 0;

	// Create the contact. It has a normal in the plane direction.
	Contact* contact = data->contacts;
	contact->contactNormal = plane.normal;
	contact->penetration = -ballDistance;
	contact->contactPoint = spherePos - plane.normal * (ballDistance + sphere.radius);
	contact->setBodyData(sphere.body, NULL, data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}

unsigned CollisionDectector::boxAndHalfSpace(const CollisionBox &box,
	const CollisionPlane &plane,
	CollisionData *data)
{
	// Make sure we have enough contacts.
	if (data->contactsLeft <= 0)
		return 0;

	// Check for intersetion.
	if (!IntersectionTests::boxAndHalfSpace(box, plane))
	{
		return 0;
	}

	/*
		We have an intersection, so find the intersection points.
		We can make do with only checking vertices.
		If the box is resting on a plane or on an edge,
		it will be reported as four or two contact points.
	*/

	// Go through each combination of + and - for each half-size.
	static real mults[8][3] = { { 1, 1, 1 }, { -1, 1, 1 }, { 1, -1, 1 }, { -1, -1, 1 },
								{ 1, 1, -1 }, { -1, 1, -1 }, { 1, -1, -1 }, { -1, -1, -1 } };

	Contact *contact = data->contacts;
	unsigned contactUsed = 0;
	for (unsigned i = 0; i < 8; i++)
	{
		// Calculate the position of each vertex.
		Vector3 vertexPos(mults[i][0], mults[i][1], mults[i][2]);
		vertexPos.componentProductUpdate(box.halfSize);
		vertexPos = box.transform.transform(vertexPos);

		// Calculate the distance from the plane.
		real vertexDistance = vertexPos * plane.normal;

		// Compare this to the plane's distance.
		if (vertexDistance <= plane.offset)
		{
			/* Create the contact data */

			/*
				The contact point is halfway between the vertex and the plane.
				We multiply the direction by half the seperation distance and
				add the vertex location.
			*/
			contact->contactPoint = plane.normal;
			contact->contactPoint *= (vertexDistance - plane.offset);
			contact->contactPoint += vertexPos;
			contact->contactNormal = plane.normal;
			contact->penetration = plane.offset - vertexDistance;

			// Write the appropriate data.
			contact->setBodyData(box.body, NULL, data->friction, data->restitution);

			// Move on to the next contact.
			contact++;
			contactUsed++;

			if (contactUsed == (unsigned)data->contactsLeft)
				return contactUsed;
		}
	}

	data->addContacts(contactUsed);

	return contactUsed;
}

unsigned CollisionDectector::boxAndSphere(const CollisionBox &box,
	const CollisionSphere &sphere,
	CollisionData *data)
{
	/*
		Transform the center of the sphere into box coordinates.
		The box can be oriented in any direction, so the following
		calculations will be simpler if we can remove that orientation
		at the very beginning by doing calculations in the box's 
		coordinate space.
	*/
	Vector3 center = sphere.getAxis(3);
	Vector3 relativeCenter = box.transform.transformInverse(center);

	// Early out check to see if we can exclude the contact.
	if (real_abs(relativeCenter.x) - sphere.radius > box.halfSize.x ||
		real_abs(relativeCenter.y) - sphere.radius > box.halfSize.y ||
		real_abs(relativeCenter.z) - sphere.radius > box.halfSize.z)
	{
		return 0;
	}

	Vector3 closestPoint(0, 0, 0);
	real distance;

	// If distance is farther than the box extents, clamp to the box.
	distance = relativeCenter.x;
	if (distance > box.halfSize.x)
		distance = box.halfSize.x;
	if (distance < -box.halfSize.x)
		distance = -box.halfSize.x;
	closestPoint.x = distance;

	distance = relativeCenter.y;
	if (distance > box.halfSize.y)
		distance = box.halfSize.y;
	if (distance < -box.halfSize.y)
		distance = -box.halfSize.y;
	closestPoint.y = distance;

	distance = relativeCenter.z;
	if (distance > box.halfSize.z)
		distance = box.halfSize.z;
	if (distance < -box.halfSize.z)
		distance = -box.halfSize.z;
	closestPoint.z = distance;

	// Check we are in contact.
	distance = (closestPoint - relativeCenter).squareMagnitude();
	if (distance > sphere.radius * sphere.radius)
		return 0;

	// The cotact properties need to be given in world coordinates.
	Vector3 closestPointWorld = box.transform.transform(closestPoint);

	Contact *contact = data->contacts;
	contact->contactNormal = (closestPointWorld - center);
	contact->contactNormal.normalise();
	contact->contactPoint = closestPointWorld;
	contact->penetration = sphere.radius - real_sqrt(distance);
	contact->setBodyData(box.body, sphere.body, data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}

/*
	This preprocessor definition is only used as a convenince
	in the boxAndBox contact generation method.
*/
#define CHECK_OVERLAP(axis, index) \
if (!tryAxis(one, two, (axis), toCenter, (index), pen, best)) return 0;


unsigned CollisionDectector::boxAndBox(const CollisionBox &one, const CollisionBox &two, CollisionData *data)
{
	if (!IntersectionTests::boxAndBox(one, two))
		return 0;

	// Find the vector between the two centers.
	Vector3 toCenter = two.getAxis(3) - one.getAxis(3);

	// We start assuming there is no contact.
	real pen = REAL_MAX;
	unsigned best = 0xffffff;	// Max

	/*
		Now we check each axis, returning if it gives us
		a seperating axis, and keeping track of the axis with
		the smallest penetration otherwise.
	*/

	// Face axis for object one.
	CHECK_OVERLAP(one.getAxis(0), 0);
	CHECK_OVERLAP(one.getAxis(1), 1);
	CHECK_OVERLAP(one.getAxis(2), 2);

	// Face axis for object two.
	CHECK_OVERLAP(one.getAxis(0), 3);
	CHECK_OVERLAP(one.getAxis(1), 4);
	CHECK_OVERLAP(one.getAxis(2), 5);

	/*
		Store the best axis-major, in case we run into almost
		parallel edge collisions later.
	*/
	unsigned bestSingleAxis = best;

	// Edge-edge axis.
	CHECK_OVERLAP(one.getAxis(0).vectorProduct(two.getAxis(0)), 6);
	CHECK_OVERLAP(one.getAxis(0).vectorProduct(two.getAxis(1)), 7);
	CHECK_OVERLAP(one.getAxis(0).vectorProduct(two.getAxis(2)), 8);
	CHECK_OVERLAP(one.getAxis(1).vectorProduct(two.getAxis(0)), 9);
	CHECK_OVERLAP(one.getAxis(1).vectorProduct(two.getAxis(1)), 10);
	CHECK_OVERLAP(one.getAxis(1).vectorProduct(two.getAxis(2)), 11);
	CHECK_OVERLAP(one.getAxis(2).vectorProduct(two.getAxis(0)), 12);
	CHECK_OVERLAP(one.getAxis(2).vectorProduct(two.getAxis(1)), 13);
	CHECK_OVERLAP(one.getAxis(2).vectorProduct(two.getAxis(2)), 14);

	// Make sure we have got a result.
	assert(best != 0xffffff);

	/*
		We now know there is a collision and we know which
		of the axis gave the smallest penetration. 
		We can now deal with it in different ways depending
		on the case.
	*/
	if (best < 3)
	{
		// We have got a vertex of box two on a face of box one.
		fillPointFaceBoxBox(one, two, toCenter, data, best, pen);
		data->addContacts(1);
		return 1;
	}
	else if (best < 6)
	{
		/*
			We have got a vertex pf box one on a face of box two.
			We use thse same algorithm as above, but swap around
			one and two (and therefore also the vector between
			their centers).
		*/
		fillPointFaceBoxBox(two, one, toCenter * -1.0f, data, best - 3, pen);
		data->addContacts(1);
		return 1;
	}
	else
	{
		// We have an edge-edge contact. Find out which axis.
		best -= 6;
		unsigned oneAxisIndex = best / 3;
		unsigned twoAxisIndex = best % 3;
		Vector3 oneAxis = one.getAxis(oneAxisIndex);
		Vector3 twoAxis = two.getAxis(twoAxisIndex);

		Vector3 axis = oneAxis.vectorProduct(twoAxis);
		axis.normalise();

		// The axis should point from box one to box two.
		if (axis * toCenter > 0)
			axis = axis * -1.0f;

		/*
			We have the axes, but not the edges: each axis has 4 edges parallel
			to it, we need to find which of the 4 for each object. We do that by
			finding the point in the center of the edge. We know that its component
			in the direction of the box's collision axis is zero (it's a midpoint)
			and we determine which of the extremems in each of the other axes is closest.
		*/
		Vector3 ptOnOneEdge = one.halfSize;
		Vector3 ptOnTwoEdge = two.halfSize;
		for (unsigned i = 0; i < 3; i++)
		{
			if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
			else if (one.getAxis(i) * axis > 0) ptOnOneEdge[i] = -ptOnOneEdge[i];

			if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
			else if (two.getAxis(i) * axis < 0) ptOnTwoEdge[i] = -ptOnTwoEdge[i];
		}

		/*
			Move them into world coordinates (they are already oriented
			correctly, since they have been derived from the axis).
		*/
		ptOnOneEdge = one.transform * ptOnOneEdge;
		ptOnTwoEdge = two.transform * ptOnTwoEdge;

		/*
			So we have a point and a direction for the colliding edges.
			We need to find out the point of closest approach of the 
			two line-segments.
		*/
		Vector3 vertex = contactPoint(ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex],
			ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex],
			bestSingleAxis > 2);

		// We can fill the contact.
		Contact *contact = data->contacts;

		contact->penetration = pen;
		contact->contactNormal = axis;
		contact->contactPoint = vertex;
		contact->setBodyData(one.body, two.body,
			data->friction, data->restitution);
		data->addContacts(1);

		return 1;
	}
	
	return 0;
}
#undef CHECK_OVERLAP