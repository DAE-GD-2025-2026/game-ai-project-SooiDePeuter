// Fill out your copyright notice in the Description page of Project Settings.


#include "FSMComponent.h"

#include "ComponentUtils.h"
#include "Shared/Level_Base.h"

// Sets default values for this component's properties
UFSMComponent::UFSMComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	
	ASteeringAgent* PoliceAgent{}; //get pawn through actor->pawn
	
	std::function<void()> TempAction;
	std::function<bool()> TempEvalFunction;
	
	TempAction = [](){};
	AddState(std::unique_ptr<GameAI::FSM::State>()); //patrol
	
	TempAction = [](){};
	AddState(std::unique_ptr<GameAI::FSM::State>()); //search
	
	TempAction = [](){};
	AddState(std::unique_ptr<GameAI::FSM::State>()); //chase
	
	TempEvalFunction = [](){return false;};
	AddTransition(
		FSMInstance->GetState(0),
		 FSMInstance->GetState(1),
		 TempEvalFunction);
	
	TempEvalFunction = [](){return false;};
	AddTransition(
		FSMInstance->GetState(0),
		 FSMInstance->GetState(1),
		 TempEvalFunction);
	
	TempEvalFunction = [](){return false;};
	AddTransition(
		FSMInstance->GetState(0),
		 FSMInstance->GetState(1),
		 TempEvalFunction);
	
	TempEvalFunction = [](){return false;};
	AddTransition(
		FSMInstance->GetState(0),
		 FSMInstance->GetState(1),
		 TempEvalFunction);
}


void UFSMComponent::AddState(std::unique_ptr<GameAI::FSM::State>&& NewState)
{
	FSMInstance.get()->AddState(std::move(NewState));
}

void UFSMComponent::AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To, std::function<bool()> EvalFunc) const
{
	std::unique_ptr<GameAI::FSM::Transition> NewTransition = std::make_unique<GameAI::FSM::Transition>(From, To, &EvalFunc);
	FSMInstance.get()->AddTransition(std::move(NewTransition));
}

// Called when the game starts
void UFSMComponent::BeginPlay()
{
	Super::BeginPlay();
}


// Called every frame
void UFSMComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	
	//check for transitions and play new action
	for (GameAI::FSM::Transition* pTransition :FSMInstance->GetTransitionsFrom(FSMInstance->GetActiveState()))
	{
		if (pTransition->GetEvalFunction()())
		{
			FSMInstance->SetActiveState(pTransition->GetTo());
			FSMInstance->GetActiveState()->GetAction()();
			break;
		}
	}
}

void UFSMComponent::StartLogic()
{
	Super::StartLogic();
}

void UFSMComponent::StopLogic(const FString& Reason)
{
}

bool UFSMComponent::IsRunning() const
{
	return bIsRunning;
}

