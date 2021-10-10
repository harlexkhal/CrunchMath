#include "World.h"

namespace CrunchMath {

	World::World(Vec3 gravity)
		:Gravity(gravity), Index(0)
	{
		Parent = true;
		memset(FreeStack, true, MaxNumberOfBodies);
		CData.ContactArray = Contacts;
		m_pNext = nullptr;
	}

	World::World(Vec3 gravity, bool parent)
		:Gravity(gravity), Index(0)
	{
		this->Parent = parent;
		memset(FreeStack, true, MaxNumberOfBodies);
		CData.ContactArray = Contacts;
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
				newbody->Primitive->Set(HalfSize.x, HalfSize.y, HalfSize.z);
				break;
			}
 
			case Shape::Type::s_Sphere: {
				newbody->Primitive = new cmSphere();
                float Radius = *(float*)primitive->GetHalfSize();
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
						newbody->Primitive->Set(HalfSize.x, HalfSize.y, HalfSize.z);
						break;
					}
		 
					case Shape::Type::s_Sphere: {
						newbody->Primitive = new cmSphere();
						float Radius = *(float*)primitive->GetHalfSize();
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
				newbody->Primitive->Set(HalfSize.x, HalfSize.y, HalfSize.z);
				break;
			}

			case Shape::Type::s_Sphere: {
				newbody->Primitive = new cmSphere();
				float Radius = *(float*)primitive->GetHalfSize();
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

		/*--------------------Temporary---------------------------------*/
		ptrStack = Stack;
		while (ptrStack != nullptr)
		{
			if ((ptrStack)->GetPosition().y <= -0.9f)
			{
				(ptrStack)->SetVelocity(0, 0, 0);
				(ptrStack)->SetRotation(0, 0, 0);
				(ptrStack)->SetPosition(0.0f, 0.9f, 0.0f);
			}

			ptrStack = ptrStack->m_pNext;
		}
		//-----------------------------------------------------------------

		Resolver.ResolveContacts(CData.ContactArray, CData.ContactCount, dt);
	}
}