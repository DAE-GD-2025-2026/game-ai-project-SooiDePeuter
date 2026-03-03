
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
	std::vector<SteeringOutput> steeringOutputs{};
	steeringOutputs.reserve(WeightedBehaviors.size());

	for (BlendedSteering::WeightedBehavior& behavior : WeightedBehaviors)
	{
		steeringOutputs.emplace_back(behavior.pBehavior->CalculateSteering(DeltaT, Agent));
	}

	//set velocity + apply weight
	float totalWeight{};
	for (int index{}; index < steeringOutputs.size(); index++)
	{
		totalWeight += WeightedBehaviors[index].Weight;
	}

	//check for zero division
	if (totalWeight < FLT_EPSILON && totalWeight > -FLT_EPSILON)
	{
		return BlendedSteering;
	}
	const float inverseTotalWeight{ 1.f / totalWeight };

	for (int index{}; index < steeringOutputs.size(); index++)
	{
		BlendedSteering.LinearVelocity += steeringOutputs[index].LinearVelocity * WeightedBehaviors[index].Weight * inverseTotalWeight;
		BlendedSteering.AngularVelocity += steeringOutputs[index].AngularVelocity * WeightedBehaviors[index].Weight * inverseTotalWeight;
	}

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
	SteeringOutput Steering{};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
		{
			DeltaT = 10.f;
			break;
		}
	}

	//If none of the behavior return a valid output, last behavior is returned
	return Steering;
}