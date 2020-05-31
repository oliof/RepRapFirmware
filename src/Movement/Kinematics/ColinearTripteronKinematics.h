/*
 * ColinearTripteronKinematics.h
 *
 * Created on: 24 May 2020
 *    Authors: oliof, apsu
 * 
 * Copyright 2020, Harald Wagener (hwa@haraldwagener.com)
 */

#ifndef SRC_MOVEMENT_KINEMATICS_COLINEARTRIPTERONKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_COLINEARTRIPTERONKINEMATICS_H_

#include "RepRapFirmware.h"
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
    LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const noexcept override;
    AxesBitmap AxesToHomeBeforeProbing() const noexcept { return XyzAxes; }
    void GetAssumedInitialPosition(size_t NumTowers, float positions[]) const noexcept override;
    size_t NumHomingButtons(size_t numVisibleAxes) const noexcept override { return 0; };
    AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t NumTowers, const StringRef& filename) const noexcept override;
    bool QueryTerminateHomingMove(size_t axis) const noexcept override;
    void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;
    HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }
    AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
    AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
    void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t NumTowers, bool continuousRotationShortcut) const noexcept override;
    AxesBitmap GetLinearAxes() const noexcept override;
 protected:
    DECLARE_OBJECT_MODEL
    OBJECT_MODEL_ARRAY(towers)

 private:
    void Init() noexcept;
    void Recalc() noexcept;
    void NormaliseEndstopAdjustments() noexcept;                                                                                                    // Make the average of the endstop
    // colinear tripteron default values
    #define PIOVER180 0.01745329251994329576923690768489F
    // arm angle in degrees
    static constexpr float DefaultArmAngle = 30;
    // tower rotations in degrees. a tower is assumed not rotated
    static constexpr float DefaultATowerRotation = 0;
    static constexpr float DefaultBTowerRotation = 120;
    static constexpr float DefaultCTowerRotation = 240;
    // print area limits in mm
    static constexpr float DefaultPrintRadius = 120;
    static constexpr float DefaultHomedHeight = 220;
    static constexpr size_t NumTowers = 3;
    // internal tower names
    static constexpr size_t A_TOWER = 0;
    static constexpr size_t B_TOWER = 1;
    static constexpr size_t C_TOWER = 2;
    // core parameters
    float armAngle;              // angle of the arms
    float printRadius;           // printable radius
    float homedHeight;
    float aTowerRotation;
    float bTowerRotation;
    float cTowerRotation;
    float numTowers;
    float endstopAdjustments[NumTowers];

    // derived parameters
    float towerX[NumTowers];     // X position of every tower
    float towerY[NumTowers];     // Y position of every tower
    float printRadiusSquared;
    float alwaysReachableHeight;
    float homedCarriageHeights[NumTowers];
    float a_x;
    float a_y;
    float b_x;
    float b_y;
    float c_x;
    float c_y;
    float denominator;
};

#endif  // SRC_MOVEMENT_KINEMATICS_COLINEARTRIPTERONKINEMATICS_H_

// End
