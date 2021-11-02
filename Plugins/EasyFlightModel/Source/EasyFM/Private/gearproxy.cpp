// Copyright 2020 Mookie. All Rights Reserved.

#ifdef WITH_EDITOR
#include "easygearcomponent.h"
#include "PrimitiveSceneProxy.h"

FPrimitiveSceneProxy* UEasyGear::CreateSceneProxy() {
	{
		class FGearProxy : public FPrimitiveSceneProxy
		{
		public:
			FGearProxy(UEasyGear* InComponent) : FPrimitiveSceneProxy(InComponent)
			{
				bWillEverBeLit = false;
				Component = InComponent;
			}

			virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const override
			{
				QUICK_SCOPE_CYCLE_COUNTER(STAT_GearSceneProxy_GetDynamicMeshElements);

				FMatrix Transform = GetLocalToWorld();

				if (!Component->UseThisComponentTransform) {
					UPrimitiveComponent* Parent = Cast<UPrimitiveComponent>(Component->GetAttachParent());
					if (Parent) {
						FName ParentSocket = Component->GetAttachSocketName();
						FBodyInstance* bodyInst = Parent->GetBodyInstance(ParentSocket);
						if (bodyInst) {
							Transform = bodyInst->GetUnrealWorldTransform().ToMatrixNoScale();
						}
					}
				}

				for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex++)
				{
					if (VisibilityMap && ((1 << ViewIndex) != 0))
					{
						const FSceneView* View = Views[ViewIndex];
						const FLinearColor DrawColor = GetViewSelectionColor(FColor::Green, *View, IsSelected(), IsHovered(), true, IsIndividuallySelected());

						FPrimitiveDrawInterface* PDI = Collector.GetPDI(ViewIndex);

						{
							FTransform WheelTransform = FTransform(FVector(Component->MainLocationLong, Component->MainLocationLat, (Component->MainLocationVert - Component->MainStrutLength))) * FTransform(Transform);
							DrawWireSphereAutoSides(PDI, WheelTransform, DrawColor, Component->MainWheelRadius, 16);
						}

						{
							FTransform WheelTransform = FTransform(FVector(Component->MainLocationLong, -Component->MainLocationLat, (Component->MainLocationVert - Component->MainStrutLength))) * FTransform(Transform);
							DrawWireSphereAutoSides(PDI, WheelTransform, DrawColor, Component->MainWheelRadius, 16);
						}

						{
							FTransform WheelTransform = FTransform(FVector(Component->NoseLocationLong, 0, (Component->NoseLocationVert - Component->NoseStrutLength))) * FTransform(Transform);
							DrawWireSphereAutoSides(PDI, WheelTransform, DrawColor, Component->NoseWheelRadius, 16);
						}
					}
				}
			}

			virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override
			{
				const bool bProxyVisible = IsSelected();

				FPrimitiveViewRelevance Result;
				Result.bDrawRelevance = (IsShown(View));
				Result.bDynamicRelevance = true;
				Result.bShadowRelevance = false;
				Result.bEditorPrimitiveRelevance = UseEditorCompositing(View);
				return Result;
			}
			virtual uint32 GetMemoryFootprint(void) const override { return(sizeof(*this) + GetAllocatedSize()); }
			uint32 GetAllocatedSize(void) const { return(FPrimitiveSceneProxy::GetAllocatedSize()); }
			virtual SIZE_T GetTypeHash() const override { return 0; }

		private:
			UEasyGear* Component;
		};

		return new FGearProxy(this);
	}
};

#endif