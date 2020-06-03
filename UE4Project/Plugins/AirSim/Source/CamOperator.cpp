// This program is free software; you can redistribute it and/ormodify it under the terms of the GNU General Public License as published by the Free Software Foundation


#include "CamOperator.h"

// Sets default values
ACamOperator::ACamOperator()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

msr::airlib::CarApiBase::CameraA ACamOperator::getCamera()
{
	return exportCamera;
}

// Called when the game starts or when spawned
void ACamOperator::BeginPlay()
{
	Super::BeginPlay();
	exportCamera.width = texture->SizeX;
	exportCamera.height = texture->SizeY;

	exportCamera.data.resize(exportCamera.width * exportCamera.height * 4);
	
}

// Called every frame
void ACamOperator::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	TArray<FColor> SurfData;
	FRenderTarget* RenderTarget = texture->GameThread_GetRenderTargetResource();
	RenderTarget->ReadPixels(SurfData);

	int j = 0;
	for (int i = 0; i < exportCamera.width * exportCamera.height * 4; i += 4)
	{
		exportCamera.data[i] = SurfData[j].R;
		exportCamera.data[i + 1] = SurfData[j].G;
		exportCamera.data[i + 2] = SurfData[j].B;
		exportCamera.data[i + 3] = SurfData[j].A;
		j++;
	}



}

