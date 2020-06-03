// This program is free software; you can redistribute it and/ormodify it under the terms of the GNU General Public License as published by the Free Software Foundation

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/TextureRenderTarget2D.h"
#include "vehicles/car/api/CarApiBase.hpp"
#include "CamOperator.generated.h"

UCLASS()
class AIRSIM_API ACamOperator : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ACamOperator();

	UPROPERTY(EditAnywhere, Category = "Camera")
		UTextureRenderTarget2D *texture;


	msr::airlib::CarApiBase::CameraA getCamera();


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	msr::airlib::CarApiBase::CameraA exportCamera;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
