#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"


Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
{
	Agents.SetNum(FlockSize);

	pNeighbors.Reserve(50); //should be enough considering the range

	//blended steering
	std::vector<BlendedSteering::WeightedBehavior> birdBehavior{};
	birdBehavior.reserve(5);
	birdBehavior.emplace_back( pCohesionBehavior.get(), 0.6f );
	birdBehavior.emplace_back( pSeparationBehavior.get(), 1.5f );
	birdBehavior.emplace_back( pVelMatchBehavior.get(), 0.8f );
	birdBehavior.emplace_back( pSeekBehavior.get(), 0.0f );
	birdBehavior.emplace_back( pWanderBehavior.get(), 0.4f );

	pBlendedSteering = std::make_unique<BlendedSteering>(birdBehavior);

	//priority steering
	std::vector<ISteeringBehavior*> evadeBehavior{};
	evadeBehavior.reserve(2);
	evadeBehavior.emplace_back(pEvadeBehavior);
	evadeBehavior.emplace_back(pBlendedSteering);

	pPrioritySteering = std::make_unique<PrioritySteering>(evadeBehavior);
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
  //TODO: update the flock
  //TODO: for every agent:
  //TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
  //TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
  //TODO: trim the agent to the world
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

  // TODO: implement ImGUI checkboxes for debug rendering here

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Cohesion",
			pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Separation",
			pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("VelMatch",
			pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
			pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");

		ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
			pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
			[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");

		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
 // TODO: Debugrender the neighbors for the first agent in the flock
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	int index{};
	for( ASteeringAgent* agent: Agents)
	{
		if ((agent->GetPosition() - pAgent->GetPosition()).Length() < NeighborhoodRadius &&
			agent != pAgent)
		{
			pNeighbors[index] = agent;
			index++;
		}
	}
	NrOfNeighbors = index;
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D averagePosition = FVector2D::ZeroVector;

	for (int index{}; index < NrOfNeighbors; index++)
	{
		averagePosition += pNeighbors[index]->GetPosition();
	}

	averagePosition /= NrOfNeighbors;
	
	return averagePosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D averageVelocity = FVector2D::ZeroVector;

	for (int index{}; index < NrOfNeighbors; index++)
	{
		averageVelocity += pNeighbors[index]->GetLinearVelocity();
	}

	averageVelocity /= NrOfNeighbors;

	return averageVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
 // TODO: Implement
}

