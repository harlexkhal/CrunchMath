#include "World.h"

namespace CrunchMath {

	World::World(Vec3 gravity)
		:Gravity(gravity), Index(0)
	{
		Parent = true;
		memset(FreeStack, true, MaxNumberOfBodies);
		CData.ContactArray = Contacts;
		Resolver.SetIterations(6, 3);
		m_pNext = nullptr;
	}

	World::World(Vec3 gravity, bool parent)
		:Gravity(gravity), Index(0)
	{
		this->Parent = parent;
		memset(FreeStack, true, MaxNumberOfBodies);
		CData.ContactArray = Contacts;
		Resolver.SetIterations(6, 3);
		m_pNext = nullptr;
    }

	World::~World()
	{
		//Only Parent World Nodes/Blocks can delete its children
		if (Parent)
		{
			World* ptr = this->m_pNext;
			while (ptr != nullptr)
			{
				World* Hold = ptr;
				ptr = ptr->m_pNext;
                delete Hold;
			}
		}
    }

	Body* World::CreateBody(Shape* primitive)
	{
		if (Empty())
		{
			Stack[0].SetAcceleration(Gravity);
			Body* newbody = Stack + Index;

			switch (primitive->GetType())
			{
			case Shape::Type::s_Box: {
				newbody->Primitive = new Box();
				Vec3 HalfSize = *(Vec3*)primitive->GetHalfSize();
				/* Its easy to detect whats a Boxand whats a Square by checking the
				 * z-axis of the Size vector. so therefore no other enum shape for
				 * squares would be implemented  (Check s_Shere comments below)
				 */
				newbody->Size = HalfSize * 2;
				newbody->Primitive->Set(HalfSize.x, HalfSize.y, HalfSize.z);
				break;
			}
 
			case Shape::Type::s_Sphere: {
				newbody->Primitive = new cmSphere();
                float Radius = *(float*)primitive->GetHalfSize();
				//Assuming its a Sphere....Note there would be a cirlce enum type later.
				//given the facts that circles are 2D while Spheres are 3D.
				newbody->Size = Vec3(Radius * 2, Radius * 2, Radius * 2);
				newbody->Primitive->Set(Radius);
				break;
			}
 
			default:
				 std::cerr << "Shape Type Does not Exist" << std::endl; assert(false);
				break;
			}
 
			newbody->m_pNext = nullptr;
			FreeStack[0] = false;
			Index++;
		    return newbody;
		}

		else if (Index == Size)
		{
			bool FoundSpace = false;
			for (int i = 0; i < Size; i++)
			{
				if (FreeStack[i])
				{
					Stack[i].SetAcceleration(Gravity);

					Body* newbody = Stack + i;

					switch (primitive->GetType())
					{
					case Shape::Type::s_Box: {
						newbody->Primitive = new Box();
						Vec3 HalfSize = *(Vec3*)primitive->GetHalfSize();
						/* Its easy to detect whats a Boxand whats a Square by checking the
					     * z-axis of the Size vector. so therefore no other enum shape for
					     * squares would be implemented  (Check s_Shere comments below)
						 */
						newbody->Size = HalfSize * 2;
						newbody->Primitive->Set(HalfSize.x, HalfSize.y, HalfSize.z);
						break;
					}
		 
					case Shape::Type::s_Sphere: {
						newbody->Primitive = new cmSphere();
						float Radius = *(float*)primitive->GetHalfSize();
					    /*Assuming its a Sphere....Note there would be a cirlce enum type later.
					     * given the facts that circles are 2D while Spheres are 3D.
						 */
					    newbody->Size = Vec3(Radius * 2, Radius * 2, Radius * 2);
						newbody->Primitive->Set(Radius);
						break;
					}
		 
					default:
						 std::cerr << "Shape Type Does not Exist" << std::endl; assert(false);
						break;
					}
					FreeStack[i] = false;

					return newbody;
				}
			}
			
			if (!FoundSpace)
			{
				if (m_pNext == nullptr)
				{
					//subsequent World blocks/nodes created from here are children 
					m_pNext = new World(Gravity, false);
					Body* newbody = m_pNext->CreateBody(primitive);

					int i = Index - 1;
					Body* Previous = Stack + i;
					Previous->m_pNext = newbody;
					
					return newbody;
				}

				else
				{
					Body* newbody = m_pNext->CreateBody(primitive);

					return newbody;
				}
			}
		}

		else
		{
		   Stack[Index].SetAcceleration(Gravity);

		   Body* newbody = Stack + Index;

			switch (primitive->GetType())
			{
			case Shape::Type::s_Box: {
				newbody->Primitive = new Box();
				Vec3 HalfSize = *(Vec3*)primitive->GetHalfSize();
			    /* Its easy to detect whats a Boxand whats a Square by checking the
				 * z-axis of the Size vector. so therefore no other enum shape for
				 * squares would be implemented  (Check s_Shere comments below)*/
				newbody->Size = HalfSize * 2;
				newbody->Primitive->Set(HalfSize.x, HalfSize.y, HalfSize.z);
				break;
			}

			case Shape::Type::s_Sphere: {
				newbody->Primitive = new cmSphere();
				float Radius = *(float*)primitive->GetHalfSize();
				/* Assuming its a Sphere....Note there would be a cirlce enum type later.
				 * given the facts that circles are 2D while Spheres are 3D.*/
			    newbody->Size = Vec3(Radius * 2, Radius * 2, Radius * 2);
				newbody->Primitive->Set(Radius);
				break;
			}

			default:
				 std::cerr << "Shape Type Does not Exist" << std::endl; assert(false);
				break;
			}
			
		   int i = Index - 1;
		   Body* Previous = Stack + i;
		   Previous->m_pNext = newbody;
		   newbody->m_pNext = nullptr;

		   FreeStack[Index] = false;
		   Index++;
		   return newbody;
		}
	}

	void World::SetIterations(uint32_t Position, uint32_t Velocity)
	{
		Resolver.SetIterations(Position, Velocity);
	}

	void World::Step(float dt)
	{
		CData.Reset(MaxContacts);
		CData.Friction = 0.9f;
		CData.Restitution = 0.1f;

		Body* ptrStack = Stack;
		while (ptrStack != nullptr)
		{
			(ptrStack)->Integrate(dt);
			ptrStack = ptrStack->m_pNext;
		}

		ptrStack = Stack;
		while (ptrStack != nullptr)
		{
			for (Body* i = ptrStack->m_pNext; i != nullptr; )
			{
				CrunchMath::CollisionDetector::Collision(ptrStack, i, &CData);
				i = i->m_pNext;
		    }
			ptrStack = ptrStack->m_pNext;
		}

		Resolver.ResolveContacts(CData.ContactArray, CData.ContactCount, dt);
	}
}