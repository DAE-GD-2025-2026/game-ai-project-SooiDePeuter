// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <functional>
#include <memory>
#include <vector>
#include <cassert>
#include <string>

#include "CoreMinimal.h"
#include "BrainComponent.h"
#include "FSMComponent.generated.h"

namespace GameAI::FSM
{
	class State;
	class Transition;
	class FSM; // contains FSM logic
}

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class GAMEAIPROG_API UFSMComponent : public UBrainComponent
{
	GENERATED_BODY()

public:
	// Sets default values for this component's properties
	UFSMComponent();

	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;
	
	virtual void StartLogic() override;
	virtual void StopLogic(const FString& Reason) override;
	
	virtual bool IsRunning() const override; 
	
	void AddState(std::unique_ptr<GameAI::FSM::State>&& NewState);
	void AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To, std::function<bool()> EvalFunc) const;

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

private:
	std::unique_ptr<GameAI::FSM::FSM> FSMInstance;
	bool bIsRunning{false};
};




class GameAI::FSM::State
{
public:
	State(std::function<void()>* action)
		: Action(action)
	{
	}
	
	std::function<void()>* GetAction() const
	{
		return Action;
	}
	
private:
	std::function<void()>* Action;
};




class GameAI::FSM::Transition
{
public:
	Transition(GameAI::FSM::State* from, GameAI::FSM::State* to, std::function<bool()>* evalFunction)
		:From(from), To(to), EvalFunction(evalFunction)
	{
	}
	
	State* GetFrom() const
	{
		return From;
	}
	State* GetTo() const
	{
		return To;
	}
	
	std::function<bool()>* GetEvalFunction() const
	{
		return EvalFunction;
	}
	
private:
	GameAI::FSM::State* From;
	GameAI::FSM::State* To;
	std::function<bool()>* EvalFunction;
};




class GameAI::FSM::FSM
{
public:
	
	State* GetState(int id) const
	{
		if (id < 0 || id >= States.size())
		{
			assert(false);
		}
		
		return States[id];
	}
	
	std::vector<Transition*>& GetTransitionsFrom(int id) const
	{
		return GetTransitionsFrom(GetState(id));
	}
	std::vector<Transition*>& GetTransitionsFrom(State* state) const
	{
		std::vector<Transition*> FoundTransitions;
		
		for (GameAI::FSM::Transition* Transition : Transitions)
		{
			if (state == Transition->GetFrom())
			{
				FoundTransitions.push_back(Transition);
			}
		}
		
		return  FoundTransitions;
	}
	std::vector<Transition*>& GetTransitionsTo(int id) const
	{
		return GetTransitionsTo(GetState(id));
	}
	std::vector<Transition*>& GetTransitionsTo(State* state) const
	{
		std::vector<Transition*> FoundTransitions;
		
		for (GameAI::FSM::Transition* Transition : Transitions)
		{
			if (state == Transition->GetTo())
			{
				FoundTransitions.push_back(Transition);
			}
		}
		
		return  FoundTransitions;
	}
	
	void AddState(std::unique_ptr<GameAI::FSM::State>&& NewState)
	{
		States.push_back(NewState.get());
	}
	void AddTransition(std::unique_ptr<GameAI::FSM::Transition>&& NewTransition)
	{
		Transitions.push_back(NewTransition.get());
	}
	
	State* GetActiveState() const
	{
		return ActiveState;
	}
	void SetActiveState(State* newState)
	{
		ActiveState = newState;
	}
	
private:
	std::vector<State*> States;
	std::vector<Transition*> Transitions;
	State* ActiveState = States[0];
};