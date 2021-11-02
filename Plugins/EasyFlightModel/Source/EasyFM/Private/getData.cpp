// Copyright 2016 Mookie. All Rights Reserved.

#include "flightmodel.h"
#include "flightmodelcomponent.h"

FTransform UeasyFM::GetFMTransform(FBodyInstance* BodyInst) const {
	if (UseThisComponentTransform) {
		return this->GetComponentTransform();
	}
	else {
		return BodyInst->GetUnrealWorldTransform();
	}
}

FVector UeasyFM::GetFMVelocity(FBodyInstance* BodyInst) const{
	FVector RawVelocity = BodyInst->GetUnrealWorldVelocity();
	return(RawVelocity - GetWind(GetFMTransform(BodyInst).GetLocation()));
};

FVector UeasyFM::GetRawVelocity(FBodyInstance* BodyInst) const {
	return BodyInst->GetUnrealWorldVelocity();
};

FVector UeasyFM::GetAngularVelocity(FBodyInstance* BodyInst) const {
	return BodyInst->GetUnrealWorldAngularVelocityInRadians() / PI * 180.0f;
}

float UeasyFM::GetAltitude(FBodyInstance* BodyInst) const {
	FVector DistanceFromOrigin = (GetFMTransform(BodyInst).GetLocation() - WorldCenterLocation + FVector(GetWorld()->OriginLocation));
	if (SphericalAltitude)
	{
		return (DistanceFromOrigin.Size() - SeaLevelRadius);
	}
	else {
		return DistanceFromOrigin.Z;
	}
};

void UeasyFM::Extract(
	FBodyInstance* BodyInst,
	FVector &fwdvec,
	FVector &upvec,
	FVector &rightvec,
	float &pitchrate,
	float &yawrate,
	float &rollrate
	)const {

	FVector AngularVelocity = GetAngularVelocity(BodyInst);
	GetAxisVectors(GetFMTransform(BodyInst), fwdvec, upvec, rightvec);

	pitchrate = FVector::DotProduct(rightvec, AngularVelocity);
	rollrate = FVector::DotProduct(fwdvec, AngularVelocity);
	yawrate = FVector::DotProduct(upvec, AngularVelocity);
};


void UeasyFM::GetData(EGetDataFormat Units, bool UseGravity, FVector &Velocity, FVector &TrueVelocity, float &Altitude, float &IndicatedAirSpeed, float &TrueAirSpeed, float &GroundSpeed, float &Mach, 
	float &Alpha, float &Slip, float &PitchRate, float &YawRate, float &RollRate, float &AccelX, float &AccelY, float &AccelZ) const {
	UPrimitiveComponent *LocalParent = Cast<UPrimitiveComponent>(GetAttachParent());
	FBodyInstance *BodyInst = LocalParent->GetBodyInstance();

	if (BodyInst == nullptr) { return; }

	//multipliers
	float AltitudeMultiplier;
	float SpeedMultiplier;
	float AccelerationMultiplier;

	switch (Units) {
	case EGetDataFormat::GDF_UU:
		AltitudeMultiplier = 1.0f;
		SpeedMultiplier = 1.0f;
		AccelerationMultiplier = 1.0f;
		break;
	case EGetDataFormat::GDF_Metric:
		AltitudeMultiplier = 0.01f;
		SpeedMultiplier = 0.036f;
		AccelerationMultiplier = 1.0/980.0f;
		break;
	case EGetDataFormat::GDF_Imperial:
		AltitudeMultiplier = 0.0328084f;
		SpeedMultiplier = 0.0223694f;
		AccelerationMultiplier = 1.0/980.0f;
		break;
	case EGetDataFormat::GDF_Nautical:
		AltitudeMultiplier = 0.0328084f;
		SpeedMultiplier = 0.0194384f;
		AccelerationMultiplier = 1.0/980.0f;
		break;
	default:
		AltitudeMultiplier = 1.0f;
		SpeedMultiplier = 1.0f;
		AccelerationMultiplier = 1.0/980.0f;
	};

	AltitudeMultiplier /= WorldScale;
	SpeedMultiplier /= WorldScale;
	AccelerationMultiplier /= WorldScale;

	TrueVelocity = GetRawVelocity(BodyInst);
	Velocity = GetFMVelocity(BodyInst);
	FVector AngularVelocity = GetAngularVelocity(BodyInst);
	FVector fwdvec, upvec, rightvec;

	Extract(BodyInst, fwdvec, upvec, rightvec, PitchRate, YawRate, RollRate);

	float SpeedOfSound = GetSpeedOfSound(GetFMTransform(BodyInst).GetLocation());
	float AirDensity = GetAirDensity(GetFMTransform(BodyInst).GetLocation());
	Mach = Velocity.Size() / SpeedOfSound;

	float fwdspeed = FVector::DotProduct(Velocity, fwdvec);
	IndicatedAirSpeed = FMath::Sqrt((AirDensity / SeaLevelAirDensity)*FMath::Pow(fwdspeed, 2.0f))*SpeedMultiplier;
	TrueAirSpeed = fwdspeed*SpeedMultiplier;
	GroundSpeed = TrueVelocity.Size()*SpeedMultiplier;

	Altitude = GetAltitude(BodyInst) * AltitudeMultiplier;

	FVector newAcceleration = acceleration;
	if (UseGravity) {
		newAcceleration -= FVector(0.0f, 0.0f, GetWorld()->GetGravityZ());
	};

	AccelX = FVector::DotProduct(newAcceleration, fwdvec)*AccelerationMultiplier;
	AccelY = FVector::DotProduct(newAcceleration, rightvec)*AccelerationMultiplier;
	AccelZ = FVector::DotProduct(newAcceleration, upvec)*AccelerationMultiplier;

	Alpha = GetAlpha(GetFMTransform(BodyInst), Velocity);
	Slip = GetSlip(GetFMTransform(BodyInst), Velocity);
}

void UeasyFM::GetAtmosphereData(FVector & OutWind, float & OutDensity, float & OutPressure, float & OutTemperature) const
{
	UPrimitiveComponent *LocalParent = Cast<UPrimitiveComponent>(GetAttachParent());
	FBodyInstance *BodyInst = LocalParent->GetBodyInstance();

	if (BodyInst == nullptr) { return; }
	FTransform Transform;


	if (UseThisComponentTransform) {
		Transform = this->GetComponentTransform();
	}
	else {
		Transform = BodyInst->GetUnrealWorldTransform();
	}
	FVector Location = Transform.GetLocation();
	OutWind = GetWind(Location);
	float AltitudeM = GetAltitude(BodyInst)/100.0f;

	OutDensity = GetAltitudeDensity(AltitudeM);
	OutPressure = GetAltitudePressure(AltitudeM);
	OutTemperature = GetAltitudeTemperature(AltitudeM);
}

/*inline*/ float UeasyFM::GetAlpha(const FTransform &Transform, FVector velocity) const {
	FVector fwdvec, upvec, rightvec;
	GetAxisVectors(Transform, fwdvec, upvec, rightvec);

	float fwdspeed = FVector::DotProduct(velocity, fwdvec);
	float upspeed = FVector::DotProduct(velocity, upvec);

	return -FMath::Atan2(upspeed*FMath::Lerp(1.0f, GroundEffectAlphaMultiplier, GroundEffect), fwdspeed) * 180.0f / PI;
};

/*inline*/ float UeasyFM::GetSlip(const FTransform &Transform, FVector velocity) const {
	FVector fwdvec, upvec, rightvec;
	GetAxisVectors(Transform, fwdvec, upvec, rightvec);

	float fwdspeed = FVector::DotProduct(velocity, fwdvec);
	float rightspeed = FVector::DotProduct(velocity, rightvec);

	return -FMath::Atan2(rightspeed, fwdspeed) * 180.0f / PI;
};

/*inline*/ void UeasyFM::GetAxisVectors(const FTransform &Transform, FVector &fwdvec, FVector &upvec, FVector &rightvec) const {
	fwdvec = Transform.GetUnitAxis(EAxis::X);
	upvec = Transform.GetUnitAxis(EAxis::Z);
	rightvec = Transform.GetUnitAxis(EAxis::Y);
}