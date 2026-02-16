#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

FVector2D ISteeringBehavior::PredictTarget(const ASteeringAgent& agent, const FTargetData& target)
{
	//calculate time to reach target
	const float time{ (agent.GetPosition() - Target.Position).Length() / agent.GetMaxLinearSpeed() };

	//calculate predicted position
	const FVector2D predictPosition{ Target.Position + time * Target.LinearVelocity };

	return predictPosition;
}

//SEEK
SteeringOutput Seek::CalculateSteering(float DeltaTime, ASteeringAgent& agent)
{
	//make an output object
	SteeringOutput result;
	
	//set Velocity
	result.LinearVelocity = Target.Position - agent.GetPosition();
	
	return result;
}

//FLEE
SteeringOutput Flee::CalculateSteering(float DeltaTime, ASteeringAgent & agent)
{
	//make an output object
	SteeringOutput result;

	//set Velocity
	result.LinearVelocity = agent.GetPosition() - Target.Position;

	return result;
}

//ARRIVE
SteeringOutput Arrive::CalculateSteering(float DeltaTime, ASteeringAgent& agent)
{
	//hardcode arrive radiuses
	const float slowRadius{2000.f};
	const float targetRadius{200.0f};

	//make an output object
	SteeringOutput result;

	//set Velocity
	result.LinearVelocity = Target.Position - agent.GetPosition();

	//set max speed
	if ((Target.Position - agent.GetPosition()).Length() < targetRadius)
	{
		agent.SetMaxLinearSpeed(0);
	}
	if ((Target.Position - agent.GetPosition()).Length() < slowRadius)
	{
		agent.SetMaxLinearSpeed((Target.Position - agent.GetPosition()).Length() / slowRadius * agent.maxLinearSpeed);
	}
	else
	{
		agent.SetMaxLinearSpeed(agent.maxLinearSpeed);
	}

	//draw debug lines

	return result;
}

//FACE
SteeringOutput Face::CalculateSteering(float DeltaTime, ASteeringAgent& Agent)
{
	SteeringOutput result{};

	const FVector2D targetDirection { Target.Position - Agent.GetPosition() };
	const float actorAngle{ FMath::DegreesToRadians(Agent.GetActorRotation().Yaw) };
	const float rotationSpeed{ 5 };
	
	//rotate over the smallest angle
	const float rotation = FMath::FindDeltaAngleRadians(actorAngle, FMath::Atan2(targetDirection.Y, targetDirection.X));

	result.AngularVelocity = rotation * DeltaTime * rotationSpeed;

	//stop rotating if angle < 0.05 rads
	if (FMath::Abs(rotation) < 0.05f)
	{
		result.AngularVelocity = 0.f;
	}

	return result;
}

//PURSUIT
SteeringOutput Pursuit::CalculateSteering(float DeltaTime, ASteeringAgent& agent)
{
	//make an output object
	SteeringOutput result;

	//set direction if target is immobile VS mobile
	if (Target.LinearVelocity.Length() < FLT_EPSILON)
	{
		result.LinearVelocity = Target.Position - agent.GetPosition();
	}
	else
	{
		result.LinearVelocity = PredictTarget(agent, Target) - agent.GetPosition();
	}

	return result;
}

//EVADE
SteeringOutput Evade::CalculateSteering(float DeltaTime, ASteeringAgent& agent)
{
	//make an output object
	SteeringOutput result;

	//set direction if target is immobile VS mobile
	if (Target.LinearVelocity.Length() < FLT_EPSILON)
	{
		result.LinearVelocity = agent.GetPosition() - Target.Position;
	}
	else
	{
		result.LinearVelocity = agent.GetPosition() - PredictTarget(agent, Target);
	}

	return result;
}