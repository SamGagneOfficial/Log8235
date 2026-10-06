// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"

#include "SDTAIController.generated.h"

/**
 * 
 */
UCLASS(ClassGroup = AI, config = Game)
class SOFTDESIGNTRAINING_API ASDTAIController : public AAIController
{
    GENERATED_BODY()
public:
    virtual void Tick(float deltaTime) override;


protected:  //Member, organised by type size

    FVector m_Velocity = FVector::Zero();

    UPrimitiveComponent* m_Target = nullptr;

    float m_VisionRange = 600.0f;       //Radius of the Collision sphere for vision[cm]
    float m_Mass = 70.0f;               //[kg]

    int m_RayAmount = 9;

    //Variables to Initiate from Engine

    float m_Thickness = 0.0f;           //Half width of agent [m]
    float m_WalkSpeed = 0.0f;           //Base/Initial walk speed [cm/s]
    float m_EnergyBank = 0.0f;          //Extra energy stored for self-activated acceleration and steering [kg cm^(2) s^(-2)]
    float m_Inertia = 0.0f;             //Used to calculate rotation cost [kg cm^2]

    bool m_Initiated = false;

protected:  //Methods

    void Initialize(APawn* pawn);

    float GetKinEnergy() const { return 0.5f * m_Mass * m_WalkSpeed * m_WalkSpeed; };

    //React distance to a wall's field 3*thickness to avoid half-body collision
    float GetReactDist() const { return 2.5f * m_Thickness; }; 

    bool TargetOutOfSight(FVector pawnLoc) const {

        if (!m_Target) //No point re-assigning value
            return false;

        FVector targetLoc = m_Target->GetComponentLocation();

        //Is the target out of VisionRange, or are they outside the field of vision (here assumed 120 deg, cos(+/- 60) = 0.5f)
        return (FVector::DistSquared(targetLoc,pawnLoc) > m_VisionRange * m_VisionRange ||
                FVector::DotProduct((targetLoc - pawnLoc).GetSafeNormal(), m_Velocity.GetSafeNormal()) < 0.5f);
    };

    float EvaluatePotential(float distance);                       //Function used to calculate steering angle

    //Function used to calculate current acceleration
    FVector ApplyAttractionForce(const FVector direction, float deltaTime, bool isAttractive) {

        int attractiveness = 1;

        if (!isAttractive)
            attractiveness *= -1;

        return attractiveness * 3.0f * GetKinEnergy() * deltaTime / ((GetReactDist() - m_Thickness) * m_Mass) * direction;
    }

    //Picks a steering direction based on forward potential field
    float GetSteeringAngle(UWorld* world, APawn* pawn, FVector position, FVector forward, FVector up);

    FVector GetDeltaV(UWorld* world, APawn* pawn, FVector position, FVector forward, float deltaTime);
};
