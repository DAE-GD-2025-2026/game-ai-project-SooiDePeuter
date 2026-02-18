
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput BlendedSteering = {};

	//call steering functions for every part
	SteeringOutput result1{ WeightedBehaviors[0].pBehavior->CalculateSteering(DeltaT, Agent)};
	SteeringOutput result2{ WeightedBehaviors[1].pBehavior->CalculateSteering(DeltaT, Agent)};

	//set velocity + apply weight
	BlendedSteering.LinearVelocity += result1.LinearVelocity * WeightedBehaviors[0].Weight;
	BlendedSteering.LinearVelocity += result2.LinearVelocity * WeightedBehaviors[1].Weight;

	BlendedSteering.AngularVelocity += result1.AngularVelocity * WeightedBehaviors[0].Weight;
	BlendedSteering.AngularVelocity += result2.AngularVelocity * WeightedBehaviors[1].Weight;

	if (Agent.GetDebugRenderingEnabled())
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector{BlendedSteering.LinearVelocity, 0} * (Agent.GetMaxLinearSpeed() * DeltaT),
			30.f, FColor::Red
			);

	return BlendedSteering;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
		{
			break;
		}
	}

	//If non of the behavior return a valid output, last behavior is returned
	return Steering;
}