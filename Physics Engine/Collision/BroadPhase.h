#ifndef BROADPHASE_H
#define BROADPHASE_H

#include "../Dynamics/body.h"

namespace Physics_Engine
{
	struct BoundingSphere
	{
		Vector3 center;
		real radius;

	public:
		// Creates a new bounding sphere at the given center and radius.
		BoundingSphere(const Vector3 &center, real radius);

		// Creates a bounding sphere to enclose the two given bounding spheres.
		BoundingSphere(const BoundingSphere &one, const BoundingSphere &two);

		bool overlaps(const BoundingSphere *other) const;

		/*
			Reports how much this bounding sphere would have to grow
			by to incorporate this given bounding sphere. Not that this
			calculation returns a value not int any particular units (i.e.
			it's not a volume growth). In fact the best implementation
			takes into account the growth in surface area (after the
			Goldsmith-Salmon algorithm for tree construction).
		*/
		real getGrowth(const BoundingSphere &other) const;

		/*
			Returns the volume of this bounding volume. This is used
			to calculate how to recurse into the bounding volume tree.
			For a bounding sphere it is a simple calculation.
		*/
		real getSize() const
		{
			return ((real)1.333333) * R_PI * radius * radius * radius;
		}
	};

	struct PotentialContact
	{
		RigidBody* bodies[2];
	};

	/*
		A template class for nodes in a bouding volume hierarchy.
		This classe uses a binary tree to store the bounding volumes.
	*/
	template<class BoudingVolumeClass>
	class BVH_Node
	{
	public:
		BVH_Node *children[2];

		/*
			Holds a single bouding volume encompassing all the
			descedants of this node.
		*/
		BoudingVolumeClass volume;

		/*
			Holds the rigid body at this node of the hierarchy.
			Only leaf nodes can have a rigid body defines (see isLeaf).
		*/
		RigidBody *body;

		// Holds the node immediately above in the tree.
		BVH_Node *parent;

		/*
			Creates a new node in the hierarchy with the given parameters.
		*/
		BVH_Node(BVH_Node *parent, const BoudingVolumeClass &volume,
			RigidBody *body = NULL);

		/*
			Checks whether this node is at the bottom of the hierarchy.
		*/
		bool isLeaf() const
		{
			return (body != NULL);
		}

		/*
			Checks the potential contacts from this node downward in
			the hierarchy, writing them to the given array (up to the given limit).
			Returns the number of potential contacts it found.
		*/
		unsigned getPotentialContacts(PotentialContact* contacts, unsigned limit) const;

		/*
			Inserts the given rigid body, with the given bounding volume,
			into the hierarchy. This may involve the creation of further
			bounding volume nodes.
		*/
		void insert(RigidBody *body, const BoudingVolumeClass &volume);

		/*
			Deletes this node, removing it first from the hierarchy,
			along with its associated rigid body and child nodes. This
			method deletes the node and all its children (but obviously
			not the rigid bodies). This also has the effect of deleting
			the sibling of this node, and changing the parent node so
			that it contains the data currently in the system. Finally,
			it forces the hierarchy above the current node to reconsider
			its bounding volume.
		*/
		~BVH_Node();

	protected:
		/*
			Checks for overlapping between nodes in the hierarchy.
			Note that nay bouning volume should have an overlaps method
			implemented that checks for overlapping with another object
			of it's own type.
		*/
		bool overlaps(const BVH_Node<BoudingVolumeClass> *other) const;

		/*
			Checks the potential contacts between this node and the given
			other node, writing them to the given array (up to the given limit).
			Returns the number of potential contacts it found.
		*/
		unsigned getPotentialContactsWith(const BVH_Node<BoudingVolumeClass> *other,
			PotentialContact *contacts,
			unsigned limit) const;
	};

	template<class BoundingVolumeClass>
	BVH_Node<BoundingVolumeClass>::BVH_Node(BVH_Node *parent, const BoundingVolumeClass &volume,
		RigidBody *body = NULL)
		: parent(parent), volume(volume), body(body)
	{
		children[0] = children[1] = NULL;
	}

	template<class BoundingVolumeClass> 
	BVH_Node<BoundingVolumeClass>::~BVH_Node()
	{
		/*
			if we don't have a parent, then we ignore the sibling processing
		*/
		if (parent)
		{
			// Find our sibling.
			BVH_Node<BoundingVolumeClass> *sibling;
			if (parent->children[0] == this)
				sibling = parent->children[1];
			else
				sibling = parent->children[0];

			// Write its data to our parent.
			parent->volume = sibling->volume;
			parent->body = sibling->body;
			parent->children[0] = sibling->children[0];
			parent->children[1] = sibling->children[1];

			/*
				Deleting this sibling (we blank it's parent and 
				children to avoid processing/deleting them).
			*/
			sibling->parent = NULL;
			sibling->body = NULL;
			sibling->children[0] = NULL;
			sibling->children[1] = NULL;
			delete sibling;

			// Recalculate the parent's bounding volume.
			parent->recalculateBoundingVolume();
		}

		/*
			Delete our children (again, we remove their 
			parent data so we don't try to process their
			siblings as they are deleted).
		*/
		if (children[0])
		{
			children[0]->parent = NULL;
			delete children[0];
		}

		if (children[1])
		{
			children[1]->parent = NULL;
			delete children[0];	// ----------- why children[0]
		}
	}

	/*
		Note that becuase we are dealing with a template here, we need
		to have the implementations accessible to anything that imports
		this header.
	*/
	template<class BoundingVolumeClass>
	bool BVH_Node<BoundingVolumeClass>::overlaps(const BVH_Node<BoundingVolumeClass> *other) const
	{
		return volume->overlaps(other->volume);
	}

	template<class BoundingVolumeClass>
	void BVH_Node<BoundingVolumeClass>::insert(RigidBody *body, const BoundingVolumeClass &volume)
	{
		/*
			If we are a leaf, then the only opition is to spawn two
			new children and place the new body in one.
		*/
		if (isleaf())
		{
			// Child one is a copy of us.
			children[0] = new BVH_Node<BoundingVolumeClass>(this, volume, body);

			// Child two holds the new body.
			children[1] new BVH_Node<BoundingVolumeClass>(this, newVolume, newBody);

			// And we now lose the body (we are no longer a leaf).
			this->body = NULL;

			// We need to recalculate our bounding volume.
			recalculateBoundingVolume();
		}

		/*
			Otherwise, we need to work out which child gets to keep
			the inserted body. We give it to whoever would grow the
			least to incorporate it.
		*/
		else
		{
			if (children[0]->volume.getGrowth(newVolume) <
				children[1]->volume.getGrowth(newVolume))
			{
				children[0]->insert(newBody, newVolume);
			}
			else
			{
				children[1]->insert(newBody, newVolume);
			}
		}
	}

	template<class BoundingVolumeClass>
	unsigned BVH_Node<BoundingVolumeClass>::getPotentialContacts(PotentialContact* contacts, unsigned limit) const
	{
		/*
			Early out if we don't have the room for contacts,
			or if we are a leaf node.
		*/
		if (isLeaf() || limit == 0)
		{
			return 0;
		}

		/*
			Get the potential contacts of one of our children with the other.
		*/
		return children[0]->getPotentialContactsWith(children[1], contacts, limit);
	}

	template<class BoundingVolumeClass>
	unsigned BVH_Node<BoundingVolumeClass>::getPotentialContactsWith(
		const BVH_Node<BoundingVolumeClass> *other,
		PotentialContact *contacts,
		unsigned limit) const
	{
		if (!overlaps(other) || limit == 0)
			return 0;

		// If we are both at leaf nodes, then we have a potential contact.
		if (isLeaf() && other->isLeaf())
		{
			contacts->body[0] = body;
			contacts->body[1] = other->body;

			return 1;
		}

		/*
			Determin which node to decend into. if either is a leaf,
			then we descend the other. If both are branches,
			then we use the one with the largest size.
		*/
		if (other->isLeaf() || (!isLeaf() && volume->getSize() >= other->volume->getSize()))
		{
			// Recurse into self.
			unsigned count = children[0]->getPotentialContactsWith(other, contacts, limit);

			// Check that we have enough slots to do the other side too.
			if (limit > count)
			{
				return count + children[1]->getPotentialContactsWith(other, contacts + count, limit - count);
			}
			else
			{
				return count;
			}
		}
		else
		{
			// Recure into the other node.
			unsigned count = getPotentialContactsWith(other->children[0], contacts, limit);

			// Check that we have enought slots to do the other side too.
			if (limit > count)
			{
				return count + getPotentialContactsWith(other->children[1], contacts + count, limt - count);
			}
			else
			{
				return count;
			}
		}
	}
}
#endif BROADPHASE.H