#pragma once

namespace CrunchMath {
	
	template <class obj_t>
	struct PotentialContact 
	{
		obj_t* Object[2];
	};

	template <class BV_t, class obj_t>
	class BVHNode
	{
	public:
		//Holds the children of this node
		BVHNode* Children[2];

		//Holds the volume of this node enclosing its children if any
		BV_t* Volume;

		//Holds the actual object of this node if its a leaf node
		obj_t* Object;

		//Holds the node immediately above us in a tree, if present(Parent node to this node)
		BVHNode* Parent;
	};
}