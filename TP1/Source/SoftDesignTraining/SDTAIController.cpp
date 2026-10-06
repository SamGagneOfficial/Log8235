// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTAIController.h"
#include "SoftDesignTraining.h"

#include "SDTCollectible.h"
#include "SoftDesignTrainingMainCharacter.h"

#include "Engine/World.h"
//#include "DrawDebugHelpers.h"
#include "Engine/OverlapResult.h"

void ASDTAIController::Tick(float deltaTime)
{
    Super::Tick(deltaTime);

    UWorld* world = GetWorld();
    APawn* pawn = GetPawn();

    if (!world || !pawn)
        return;

    if (!m_Initiated)
        Initialize(pawn);

    FVector position = pawn->GetActorLocation();
    FVector forward = m_Velocity.GetSafeNormal();
    FVector up = pawn->GetActorUpVector();


    if(TargetOutOfSight(position))      //Check if agent loses target
        m_Target = nullptr;

    FVector deltaV = GetDeltaV(world, pawn, position, forward, deltaTime);

    //Apply Steering when no direct target, avoiding rotation while in potential because of unpredictable energy gain/loss
    if (!m_Target && deltaV.IsNearlyZero())
    {
        float steerAngle = GetSteeringAngle(world, pawn, position, forward, up);
        float smoothAngle = FMath::FInterpTo(0.0f, steerAngle, deltaTime, 5.0f);
        m_Velocity = m_Velocity.RotateAngleAxis(smoothAngle, up);
        //DrawDebugSphere(world, position + GetReactDist() * forward.RotateAngleAxis(steerAngle, up), GetReactDist(), 32, FColor::Yellow);
    }

    m_Velocity += deltaV;
    m_Velocity.Z = 0;

	pawn->SetActorRotation(m_Velocity.ToOrientationQuat());
    pawn->AddActorWorldOffset(m_Velocity * deltaTime, true);
}

void ASDTAIController::Initialize(APawn* pawn) {

    m_Thickness = pawn->FindComponentByClass<UCapsuleComponent>()->GetScaledCapsuleRadius();

    if (!m_Thickness)
        m_Thickness = 30.0f;

    //Forces rayAmount to be odd, so one ray points forward
    if (!(m_RayAmount % 2))
        m_RayAmount++;

    m_Inertia = m_Mass * m_Thickness * m_Thickness * 0.5f; //Inertia approximated by cylinder

    float maxSpeed = pawn->GetMovementComponent()->GetMaxSpeed();
    m_WalkSpeed = 0.4 * maxSpeed;

    m_EnergyBank = GetKinEnergy();

    m_Velocity = m_WalkSpeed * pawn->GetActorForwardVector();
    UE_LOG(LogTemp,Warning,TEXT("Agent Initiated"));
    m_Initiated = true;
}

float ASDTAIController::EvaluatePotential(float distance) {
    return GetKinEnergy() * (2 - distance / GetReactDist());
}

float ASDTAIController::GetSteeringAngle(UWorld* world, APawn* pawn, FVector position, FVector forward, FVector up) {

    /*
    Shoots m_RayAmount rays from -60.0 to 60.0 deg, evaluating where the potential related to walls is null in order to reduce sways
    */

    FCollisionQueryParams collisionParams;
    collisionParams.AddIgnoredActor(pawn);

    //Score/potential for each evaluated point to influence steering
    TArray<float> potentials;
    potentials.SetNumZeroed(m_RayAmount);

    float angleInc = (m_RayAmount > 1) ? 120.0f / (m_RayAmount - 1) : 0;

    //Evaluate the potential energy for points ahead, hoping to pick the
    for(int i = 0; i < m_RayAmount; i++)
    {
        TArray<FOverlapResult> overlaps;
        FVector endPos = position + GetReactDist() * forward.RotateAngleAxis(-60.0f + i * angleInc, up);

        //Skip that direction if wall is in the way
        FHitResult hit;
        if (world->LineTraceSingleByChannel(hit, position, endPos, ECC_Pawn, collisionParams))
        {
            potentials[i] = -1.0f;  //negative for later score check
            continue;
        }

        world->OverlapMultiByChannel(
            overlaps,
            endPos,
            FQuat::Identity,
            ECC_Pawn,
            FCollisionShape::MakeSphere(GetReactDist()),
            collisionParams
        );

        //Looping through each walls
        for (const FOverlapResult& overlap : overlaps)
        {
            UPrimitiveComponent* component = overlap.GetComponent();

            if (component)
            {
                ECollisionChannel channel = component->GetCollisionObjectType();
                FVector closestPoint = FVector::Zero();

                if ((channel == ECC_WorldStatic || channel == ECC_GameTraceChannel3))
                    if (component->GetClosestPointOnCollision(endPos, closestPoint))
                    {
                        float minDist = GetReactDist();
                        float distance = (closestPoint - endPos).Size2D();

                        if (distance < minDist)
                            potentials[i] += EvaluatePotential(distance);
                    }
            }
        }
    }

    //check for a ray matching the zero potential requirement, starting with those closest to forward vector (of middle of array)
    if (potentials.Contains(0.0f) && potentials[m_RayAmount / 2] != 0.0f)
    {
        int rand = FMath::RandRange(0, 1) ? 1 : -1; //-1 or 1 for right - left
        for (int i = 0; i < m_RayAmount / 2; i++)
        {
            if (potentials[(m_RayAmount / 2) + i * rand] == 0.0f)
                return rand * i * angleInc;

            if (potentials[(m_RayAmount / 2) - i * rand] == 0.0f)
                return -rand * i * angleInc;
        }
    }
    
    //Lets the potential force do its thing
    return 0.0f;
}

FVector ASDTAIController::GetDeltaV(UWorld* world, APawn* pawn, FVector position, FVector forward, float deltaTime) {

    FVector deltaV = FVector::Zero();

    FCollisionQueryParams params;
    params.AddIgnoredActor(pawn);

    TArray<FOverlapResult> overlaps;

    world->OverlapMultiByChannel(
        overlaps,
        position,
        FQuat::Identity,
        ECC_Pawn,
        FCollisionShape::MakeSphere(m_VisionRange),
        params
    );

    for (const FOverlapResult& overlap : overlaps)
    {
        UPrimitiveComponent* component = overlap.GetComponent();

        if (component)
        {
            ECollisionChannel channel = component->GetCollisionObjectType();
            FVector closestPoint = FVector::Zero();

            if (component->GetClosestPointOnCollision(position, closestPoint))
            {
                FVector pawnToTarget = closestPoint - position;
                FVector direction = pawnToTarget.GetSafeNormal();

                FColor color = FColor::White; //Color of non-interactive objects

                FHitResult hit;

                bool bHit = world->LineTraceSingleByChannel(hit, pawn->GetPawnViewLocation(), closestPoint + direction, ECC_Pawn, params);

                switch (channel)
                {
                case ECC_WorldStatic: //Wall and ground
                {
                    if (hit.GetComponent() != component) //no line of sight
                        break;

                    if (pawnToTarget.Length() < GetReactDist())
                        deltaV += ApplyAttractionForce(direction, deltaTime, false);

                    color = FColor::Green;
                    break;
                }
                case ECC_GameTraceChannel4:     //Player
                {
                    if (bHit)       //obstacle found
                        break;

                    if (m_Target != component && FVector::DotProduct(forward, direction) >= 0.5f) //always becomes priority target
                        m_Target = component;

                    ASoftDesignTrainingMainCharacter* player = (ASoftDesignTrainingMainCharacter*)overlap.GetActor();

                    FVector approxNextPos = player->GetActorLocation() + player->GetVelocity() * deltaTime;
                    FVector nextDir = (approxNextPos - position).GetSafeNormal();

                    if (!player->IsPoweredUp())
                        m_Velocity = FMath::VInterpTo(m_Velocity, 2.5f * m_WalkSpeed * nextDir, deltaTime, 1.0f);
                    else
                        m_Velocity = FMath::VInterpTo(m_Velocity, 2.5f * m_WalkSpeed * -nextDir, deltaTime, 1.0f);;

                    color = FColor::Red;
                    break;
                }
                case ECC_GameTraceChannel5:     //Collectible
                {

                    if (bHit) //no line of sightb broken
                        break;

                    if (!m_Target && FVector::DotProduct(forward, direction) >= 0.5f) //becomes main target if null
                        m_Target = component;

                    if (m_Target != component) //already has target
                        break;

                    ASDTCollectible* collectible = (ASDTCollectible*)overlap.GetActor();

                    if (!collectible->IsOnCooldown())
                        m_Velocity = FMath::VInterpTo(m_Velocity, m_WalkSpeed * direction, deltaTime, 1.0f);
                    else
                        m_Target = nullptr;

                    color = FColor::Orange;
                    break;
                }
                case ECC_GameTraceChannel3:     //DeathObject
                {
                    if (bHit) //no line of sight
                        break;

                    if (pawnToTarget.Length() < GetReactDist())
                        deltaV += ApplyAttractionForce(direction, deltaTime, false);

                    color = FColor::Blue;
                    break;
                }
                case ECC_GameTraceChannel1:     //Projectile
                {
                    color = FColor::Purple;
                    break;
                }
                }

                DrawDebugLine(world, position, closestPoint, color);
            }
        }
    }

    return deltaV;
}