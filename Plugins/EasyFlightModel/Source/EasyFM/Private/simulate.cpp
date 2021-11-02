// Copyright 2016 Mookie. All Rights Reserved.

#include "flightmodel.h"
#include "flightmodelcomponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"

//DECLARE_CYCLE_STAT(TEXT("EasyFM Simulation Step"), STAT_SimulationStep, STATGROUP_EasyFM);

//Actual simulation
void UeasyFM::SimulationStep(FBodyInstance* BodyInst, float DeltaTime, bool apply, bool debug) {

	//SCOPE_CYCLE_COUNTER(STAT_SimulationStep);

	FVector fwdvec, upvec, rightvec;
	float PitchRate, YawRate, RollRate;
	this->Extract(BodyInst, fwdvec, upvec, rightvec, PitchRate, YawRate, RollRate);

	FVector RawVelocity = GetRawVelocity(BodyInst);
	FVector Velocity = GetFMVelocity(BodyInst);
	FVector AngularVelocity = GetAngularVelocity(BodyInst);
	float SpeedOfSound=GetSpeedOfSound(GetFMTransform(BodyInst).GetLocation());
	float AirDensity=GetAirDensity(GetFMTransform(BodyInst).GetLocation());

	//Recompute GE
	if (apply && DeltaTime > 0.0f && GroundEffectEnabled) {
		GroundEffect = GetGroundEffect(GetFMTransform(BodyInst));
	}

	float alpha = GetAlpha(GetFMTransform(BodyInst), Velocity);
	float slip = GetSlip(GetFMTransform(BodyInst), Velocity);

	if (apply && DeltaTime > 0.0f) {
		acceleration = (RawVelocity - PreviousVelocity) / previousDelta;
		PreviousVelocity = RawVelocity;
		previousDelta = DeltaTime;
		SimulateControls(DeltaTime, alpha, slip, PitchRate, YawRate, RollRate, FVector::DotProduct(Velocity, fwdvec), GetFMTransform(BodyInst), RawVelocity, AirDensity / SeaLevelAirDensity);
	}

	//torque
	FVector controltorque=Control(GetFMTransform(BodyInst), Velocity, AirDensity);
	FVector damptorque = Damping(PitchRate, RollRate, YawRate, Velocity.Size(), AirDensity);
	FVector stabtorque=Stability(GetFMTransform(BodyInst), Velocity, AirDensity, SpeedOfSound);

	float AlphaWithFlaps = alpha + (FlapsAlpha*FlapsPos) + (SpoilerAlpha*SpoilerPos);
	FVector LiftVector = (upvec.RotateAngleAxis(AlphaWithFlaps + PitchRate*DeltaTime, rightvec)).GetSafeNormal(); //alpha corrected
	FVector SideLiftVector = (rightvec.RotateAngleAxis(-slip + YawRate*DeltaTime, upvec)).GetSafeNormal();
	FVector DragVector = (Velocity*-1.0).GetSafeNormal();

	//engine
	float EngineThrust = 0.0f;
	FVector EngineVector = fwdvec;
	FVector WashVelocity = FVector (0,0,0);
	if (EngineEnabled) {
		EngineThrust = EnginePower(Velocity.Size(), Velocity.Size() / SpeedOfSound, AirDensity);

		if (VTOLEnabled) {
			EngineVector = EngineVector.RotateAngleAxis(-VTOLPos*VTOLMaxAngle, rightvec);
		};

		if (PropWash) {
			WashVelocity = (EngineVector*(EngineThrust*PropWashVelocity));

			FVector washcontroltorque = Control(GetFMTransform(BodyInst), Velocity + WashVelocity, AirDensity);
			FVector washdamptorque = Damping(PitchRate, RollRate, YawRate, (Velocity + WashVelocity).Size(), AirDensity);
			FVector washstabtorque = Stability(GetFMTransform(BodyInst), Velocity + (EngineVector*WashVelocity), AirDensity, SpeedOfSound);

			controltorque = FMath::Lerp(controltorque, washcontroltorque, PropWashPortionControl);
			damptorque= FMath::Lerp(damptorque, washdamptorque, PropWashPortionDamping);
			stabtorque = FMath::Lerp(stabtorque, washstabtorque, PropWashPortionStability);
		}
	}

	//Lift
	FVector Forces = AeroForces(GetFMTransform(BodyInst), Velocity, SpeedOfSound);
	if (PropWash && EngineEnabled) {
		FVector WashForces = AeroForces(GetFMTransform(BodyInst), Velocity + WashVelocity, SpeedOfSound);

		Forces.X = FMath::Lerp(Forces.X, WashForces.X, PropWashPortionLift);
		Forces.Y = FMath::Lerp(Forces.Y, WashForces.Y, PropWashPortionSideLift);
		Forces.Z = FMath::Lerp(Forces.Z, WashForces.Z, PropWashPortionDrag);
	};

	//Lift Asymmetry
	if (LiftAsymmetryEnabled && (AsymmetrySampleCount > 1) && (LiftAsymmetryPortion>0.0f)) {
		Forces *= (1.0f - LiftAsymmetryPortion); //remove lift portion
		FVector LocWind= GetWind(GetFMTransform(BodyInst).GetLocation());

		for(int i = 0; i < AsymmetrySampleCount; i++) {
			float Offset = FMath::Lerp(-WingSpan*0.5, WingSpan*0.5, float(i)/(AsymmetrySampleCount-1) );

			FVector LocLocation = BodyInst->GetCOMPosition() + (rightvec*Offset*WorldScale);
			FTransform LocTransform = FTransform(GetFMTransform(BodyInst).GetRotation(), LocLocation, FVector(1.0f));

			FVector LocVelocity = BodyInst->GetUnrealWorldVelocityAtPoint(LocLocation);
			LocVelocity -= LocWind;
			
			FVector LocForce = AeroForces(LocTransform, LocVelocity, SpeedOfSound);

			if (PropWash && EngineEnabled) {
				FVector WashForces = AeroForces(LocTransform, LocVelocity+WashVelocity, SpeedOfSound);

				LocForce.X = FMath::Lerp(LocForce.X, WashForces.X, PropWashPortionLift);
				LocForce.Y = FMath::Lerp(LocForce.Y, WashForces.Y, PropWashPortionSideLift);
				LocForce.Z = FMath::Lerp(LocForce.Z, WashForces.Z, PropWashPortionDrag);
			}

			if (Offset < 0.0f) {
				LocForce *= AsymLiftMultiplierLeft;
			}
			if (Offset > 0.0f) {
				LocForce *= AsymLiftMultiplierRight;
			}

			//asymm lift direction
			FVector LocForceRotated = (LiftVector*LocForce.X);
			LocForceRotated += (SideLiftVector*LocForce.Y);
			LocForceRotated += (DragVector*LocForce.Z);

			if (apply && DeltaTime>0.0f) {
				FVector FinalForceLoc=LocForceRotated*AirDensity;

				if (ScaleWithMass) {
					FinalForceLoc *= BodyInst->GetBodyMass() / 10000.0f;
				}

				FinalForceLoc *= LiftAsymmetryPortion / float(AsymmetrySampleCount);

				FVector ForcePoint = LocLocation; //+( RawVelocity * DeltaTime * 0.5f );
				BodyInst->AddForceAtPosition(FinalForceLoc,ForcePoint, false);
			}

#ifdef WITH_EDITOR
			if (debug) {
				FVector DebugLoc = LocLocation;
				if (DebugLagCompensation) {
					DebugLoc += RawVelocity*DeltaTime;
				}

				DrawDebugLine(GetWorld(), DebugLoc, DebugLoc+(LocForceRotated*WorldScale*DebugForceScale), FColor::Green, false, -1.0f, 1, 0);
			}
#endif
		}
	}

	//lift direction
	FVector AeroForcesRotated = (LiftVector*Forces.X);
	AeroForcesRotated += (SideLiftVector*Forces.Y);
	AeroForcesRotated += (DragVector*Forces.Z);

	//RCS
	FVector rcs=RCSTorque(EngineThrust);

	//torque combine
	FVector torque = stabtorque + damptorque + controltorque + rcs;
	torque = torque.GetClampedToMaxSize(TorqueLimit);
	FVector torqueLoc = GetFMTransform(BodyInst).GetRotation()*torque;

	FVector finalForce = ((AeroForcesRotated*AirDensity) + FVector(0.0f, 0.0f, AirDensity*Buoyancy) + (EngineVector*EngineThrust*EngineFullPower));

	if (ScaleWithMass) {
		finalForce *= BodyInst->GetBodyMass() / 10000.0f;
	}

	//prop torque 
	FVector finalTorque = (torqueLoc + 
		(fwdvec*EngineThrust*PropTorqueRoll) + 
		(upvec*EngineThrust*PropTorqueYaw) + 
		(rightvec*EngineThrust*PropTorquePitch)
		);

	if (ScaleWithMomentOfInertia) {
		finalTorque=Cast<UPrimitiveComponent>(GetAttachParent())->ScaleByMomentOfInertia(finalTorque,GetAttachSocketName())*0.000001f;
	}

	if (apply && DeltaTime>0.0f) {
		BodyInst->AddForce(finalForce, false, false);
		BodyInst->AddTorqueInRadians(finalTorque, false, false);
	}

#ifdef WITH_EDITOR
	if (debug) {
		float accfwd = FVector::DotProduct(acceleration, fwdvec) /WorldScale;
		float accright = FVector::DotProduct(acceleration, rightvec) /WorldScale;
		float accup = FVector::DotProduct(acceleration, upvec) / WorldScale;
		FVector DebugLoc = GetFMTransform(BodyInst).GetLocation();
		if (DebugLagCompensation) {
			DebugLoc += RawVelocity*DeltaTime;
		}

		if (PrintFMData) {
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("groundeffect: ") + FString::SanitizeFloat(GroundEffect));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("accelZ: ") + FString::SanitizeFloat(accup));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("accelY: ") + FString::SanitizeFloat(accright));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("accelX: ") + FString::SanitizeFloat(accfwd));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("trimRoll: ") + FString::SanitizeFloat(RollTrimPos));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("trimYaw: ") + FString::SanitizeFloat(YawTrimPos));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("trimPitch: ") + FString::SanitizeFloat(PitchTrimPos));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("autothrottle: ") + FString::SanitizeFloat(AutoThrottlePos));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("glideRatio (no asymm): ") + FString::SanitizeFloat(Forces.X / Forces.Z));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("drag: ") + FString::SanitizeFloat(Forces.Z));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("lift: ") + FString::SanitizeFloat(Forces.X));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("ALT: ") + FString::SanitizeFloat(GetAltitude(BodyInst)));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("IAS: ") + FString::SanitizeFloat(FVector::DotProduct(Velocity, fwdvec) * FMath::Sqrt(AirDensity / SeaLevelAirDensity)/WorldScale));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("mach: ") + FString::SanitizeFloat(Velocity.Size() / SpeedOfSound));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("AoA: ") + FString::SanitizeFloat(alpha));
			GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White, TEXT("slip: ") + FString::SanitizeFloat(slip));
		};

		//debug trace
		if (DebugLineLength > 0) {
			DrawDebugLine(GetWorld(), DebugLoc, DebugLoc + (fwdvec * DebugLineLength), FColor::Red, false, -1.0f, 0, 0);
			DrawDebugLine(GetWorld(), DebugLoc, DebugLoc + (upvec * DebugLineLength), FColor::Blue, false, -1.0f, 0, 0);
			DrawDebugLine(GetWorld(), DebugLoc, DebugLoc + (RawVelocity.GetSafeNormal() * DebugLineLength), FColor::Green, false, -1.0f, 1, 0);
			if (Guided) {
				if (GuidedToLocation) {
					DrawDebugLine(GetWorld(), DebugLoc, GuidedVector, FColor::Yellow, false, -1.0f, 1, 0);
				}
				else {
					DrawDebugLine(GetWorld(), DebugLoc, DebugLoc + ((GuidedVector.GetSafeNormal() * DebugLineLength)), FColor::Yellow, false, -1.0f, 1, 0);
				};
			};
		};

		if (DebugTrailTime > 0) {
			DrawDebugLine(GetWorld(), DebugLoc, DebugLoc - (RawVelocity*DeltaTime), FColor::White, false, DebugTrailTime, 0, 0);
		};

		//GE debug
		if (GroundEffectEnabled) {
			if (GroundEffect > 0.0f) {
				DrawDebugLine(GetWorld(), DebugLoc, DebugLoc - (upvec*GroundTraceLength*(1.0f - GroundEffect)), FColor::White, false, -1.0f, 1, 0);
			}else{
				DrawDebugLine(GetWorld(), DebugLoc, DebugLoc - (upvec*GroundTraceLength), FColor::Cyan, false, -1.0f, 1, 0);
			};
		};
	};
#endif
}