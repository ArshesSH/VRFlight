// Copyright 2016 Mookie. All Rights Reserved.

#include "flightmodel.h"
#include "flightmodelcomponent.h"

FVector UeasyFM::GetWind_Implementation(FVector Location) const {
	return Wind;
}

float UeasyFM::GetAirDensity_Implementation(FVector Location) const {
	float Altitude;

	FVector DistanceFromOrigin = (Location - WorldCenterLocation + FVector(GetWorld()->OriginLocation));
	if (SphericalAltitude)
	{
		Altitude = (DistanceFromOrigin.Size() - SeaLevelRadius);
	}
	else {
		Altitude = DistanceFromOrigin.Z;
	}

	switch (AtmosphereType) {
	case (EAtmosphereType::AT_Curve): {
		float airmp = SeaLevelAirDensity / GetCurveValue(AirDensityCurve, 0, SeaLevelAirDensity);
		return GetCurveValue(AirDensityCurve, Altitude / WorldScale, SeaLevelAirDensity)*airmp;
	}
	case (EAtmosphereType::AT_Earth): {
		return GetAltitudeDensity(Altitude / WorldScale /100.0f);
	}
	default: {
		return SeaLevelAirDensity;
	}
	}
}

float UeasyFM::GetSpeedOfSound_Implementation(FVector Location) const {
	if (SpeedOfSoundVariesWithAltitude) {

		float Altitude;

		FVector DistanceFromOrigin = (Location - WorldCenterLocation + FVector(GetWorld()->OriginLocation));
		if (SphericalAltitude)
		{
			Altitude = (DistanceFromOrigin.Size() - SeaLevelRadius);
		}
		else {
			Altitude = DistanceFromOrigin.Z;
		}

		float soundvmp = SeaLevelSpeedOfSound / GetCurveValue(SpeedOfSoundCurve, 0, SeaLevelSpeedOfSound);

		return SpeedOfSoundCurve->GetFloatValue((Altitude) / WorldScale)*WorldScale*soundvmp;
	}
	else {
		return SeaLevelSpeedOfSound*WorldScale;
	}
}

float UeasyFM::GetAltitudePressure(float AltitudeMeter) const{
	return FMath::Max(SeaLevelAirPressure * FMath::Pow((1 - (0.0000225577 * AltitudeMeter)), 5.25588), 0.0f);
}

float UeasyFM::GetAltitudeTemperature(float AltitudeMeter) const {
	return SeaLevelAirTemperature - (TemperatureLapseRate * FMath::Min(AltitudeMeter, TropopauseAltitude));
}

float UeasyFM::GetAltitudeDensity(float AltitudeMeter) const {
	float Temperature = GetAltitudeTemperature(AltitudeMeter);
	float Pressure = GetAltitudePressure(AltitudeMeter);
	return Pressure*100.0f / ((Temperature + 273.15)*SpecificGasConstant);
}