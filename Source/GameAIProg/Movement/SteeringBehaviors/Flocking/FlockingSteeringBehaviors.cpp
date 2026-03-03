#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
	//make an output object
	SteeringOutput result;

	//set Velocity
	result.LinearVelocity = pFlock->GetAverageNeighborPos() - Agent.GetPosition();

	return result;
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
	//make an output object
	SteeringOutput result;
	FVector2D averageDistance2D{};

	for (ASteeringAgent* pAgent : pFlock->GetNeighbors())
	{
		const FVector2D distance2D{ pAgent->GetPosition() - Agent.GetPosition() };
		const float distance{ distance2D.Length() };
		const float inverseDistance{ 1.f / distance };

		averageDistance2D += distance2D * inverseDistance;
	}
	averageDistance2D /= pFlock->GetNrOfNeighbors();

	result.LinearVelocity = -averageDistance2D + Agent.GetPosition();

	return result;
}

//*************************
//VELOCITY MATCH (FLOCKING)

SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	//make an output object
	SteeringOutput result;

	//set Velocity
	result.LinearVelocity = pFlock->GetAverageNeighborVelocity();

	return result;
}
