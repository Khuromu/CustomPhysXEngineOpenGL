#include "Behaviors/SphereSimulation.h"

#include "PhysicEngine.h"
#include "GlobalVariables.h"
#include "Renderer.h"
#include "World.h"

void CSphereSimulation::InitChain(size_t count, const Vec2& start)
{
	for (size_t i = 0; i < count; ++i)
	{
		m_chain.push_back(AddCircle(start + Vec2(0.0f, -(float)i * DISTANCE)));
	}
}

void CSphereSimulation::SolveChainConstraints()
{
	//m_chain[0]->speed = Vec2();
}

void CSphereSimulation::Start()
{
	for (float x = -12.0f; x < 12.0f; x += 5.0f)
	{
		for (float y = -22.0f; y < 22.0f; y += 10.0f)
		{
			AddCircle(Vec2(x + Random(-0.1f, 0.1f), y + Random(-0.1f, 0.1f)))->speed.x = 50.0f;
		}
	}

	InitChain(8, Vec2(20.0f, 20.0f));

	gVars->pPhysicEngine->Activate(true);
}

void CSphereSimulation::Update(float frameTime)
{
	for (CPolygonPtr& circle : m_circles)
	{
		circle->speed.y -= 20.0f * frameTime;
		circle->speed -= circle->speed * 0.3f * frameTime;
	}

	for (size_t i = 0; i < m_circles.size(); ++i)
	{
		for (size_t j = i + 1; j < m_circles.size(); ++j)
		{
			CPolygonPtr c1 = m_circles[i];
			CPolygonPtr c2 = m_circles[j];

			Vec2 diffPos = c2->position - c1->position;
			Vec2 diffSpeed = c2->speed - c1->speed;
			if (diffPos.GetSqrLength() < 4.0f * RADIUS * RADIUS && ((diffSpeed | diffPos) < 0.0f))
			{
				OnCollision(c1, c2, Vec2(), 1.f, Vec2());
			}
		}
	}

	float hWidth = gVars->pRenderer->GetWorldWidth() * 0.5f;
	float hHeight = gVars->pRenderer->GetWorldHeight() * 0.5f;

	for (CPolygonPtr& circle : m_circles)
	{
		if (circle->position.x < -hWidth + RADIUS && circle->speed.x < 0)
		{
			circle->speed.x *= -1.0f;
		}
		else if (circle->position.x > hWidth - RADIUS && circle->speed.x > 0)
		{
			circle->speed.x *= -1.0f;
		}
		if (circle->position.y < -hHeight + RADIUS && circle->speed.y < 0)
		{
			circle->speed.y *= -1.0f;
		}
		else if (circle->position.y > hHeight - RADIUS && circle->speed.y > 0)
		{
			circle->speed.y *= -1.0f;
		}
	}

	SolveChainConstraints();

	for (CPolygonPtr& circle : m_circles)
	{
		circle->position += circle->speed * frameTime;
	}
}

CPolygonPtr CSphereSimulation::AddCircle(const Vec2& pos, float radius)
{
	CPolygonPtr circle = gVars->pWorld->AddSymetricPolygon(radius, 50);
	circle->density = 0.0f;
	circle->position = pos;
	m_circles.push_back(circle);

	return circle;
}

void CSphereSimulation::OnCollision(CPolygonPtr A, CPolygonPtr B, Vec2& normal, float penetration, Vec2& point)
{
	Vec2 diff = B->position - A->position;
	normal = diff.Normalized();

	// Position correction
	float damping = 0.2f;
	penetration = diff.GetLength() - RADIUS * 2.f;
	float correction = (penetration * damping) / (A->invMass + B->invMass);
	A->position += normal * A->invMass * correction;
	B->position -= normal * B->invMass * correction;

	
	float vRel = (A->speed - B->speed) | normal;
	float restitution = 0.9f;

	if (vRel > 0)
	{
		// Objects getting closer => bounce
		float J = (-(1 + restitution)*vRel) / (A->invMass + B->invMass);
		A->speed += normal * J * A->invMass;
		B->speed -= normal * J * B->invMass;
	}
}