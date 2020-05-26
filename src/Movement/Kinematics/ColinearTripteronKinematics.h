/*
 * ColinearTripteronKinematics.h
 *
 * Created on: 24 May 2020
 *    Authors: oliof, apsu
 *
 */

#ifndef SRC_MOVEMENT_KINEMATICS_COLINEARTRIPTERONKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_COLINEARTRIPTERONKINEMATICS_H_

#include "Kinematics.h"

class ColinearTripteronKinematics: public Kinematics
{
public:
	// Constructor
	ColinearTripteronKinematics() noexcept;
        // Overriden base classes
        const char *GetName(bool forStatusReport) const noexcept override;
        bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
		bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
        void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
        bool IsReachable(float x, float y, bool isCoordinated) const noexcept override;
		LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords,
												size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const noexcept override;
		AxesBitmap AxesToHomeBeforeProbing() const noexcept { return XyzAxes; }
		void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
		size_t NumHomingButtons(size_t numVisibleAxes) const noexcept override { return 0; }
		AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
		bool QueryTerminateHomingMove(size_t axis) const noexcept override;
		void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;
		HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }
    	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
		AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
		void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept override;

	#if HAS_MASS_STORAGE
		bool WriteResumeSettings(FileStore *f) const noexcept override;
	#endif

protected:
	DECLARE_OBJECT_MODEL

private:
	static constexpr float DefaultArmAngle = 30;
	static constexpr float DefaultATowerRotation = 0;
	static constexpr float DefaultBTowerRotation = 1;
	static constexpr float DefaultCTowerRotation = 2;
	static constexpr float DefaultPrintRadius = 120;
    static constexpr float DefaultHomedHeight = 220;

	static constexpr size_t NumTowers = 3;
	static constexpr size_t A_TOWER = 0;
	static constexpr size_t B_TOWER = 1;
	static constexpr size_t C_TOWER = 2;

    void Init();
    void Recalc(); 
}
