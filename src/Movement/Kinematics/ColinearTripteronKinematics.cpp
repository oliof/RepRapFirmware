/*
 * ColinearTripteronKinematics.cpp
 *
 *  Created on: 24 May 2020
 *      Author: oliof, apsu
 *
 */

#include "Movement/Move.h"
#include "ColinearTripteronKinematics.h"
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

#include <limits>

 
#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(ColinearTripteronKinematics, __VA_ARGS__)

constexpr ObjectModelTableEntry ColinearTripteronKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
	{ "name",	OBJECT_MODEL_FUNC(self->GetName(true)), 	ObjectModelEntryFlags::none },
};

constexpr uint8_t ColinearTripteronKinematics::objectModelTableDescriptor[] = { 1, 1 };

DEFINE_GET_OBJECT_MODEL_TABLE(ColinearTripteronKinematics);

#endif

ColinearTripteronKinematics::ColinearTripteronKinematics() noexcept : Kinematics(KinematicsType::colinearTripteron, 0.0, 0, true ), numTowers(NumTowers)
{
	Init();
}

// Return the name of the current kinematics
const char *ColinearTripteronKinematics::GetName(bool forStatusReport) const noexcept
{
        return (forStatusReport) ? "colineartripteron" : "Colinear Tripteron";
}


void ColinearTripteronKinematics::Init() noexcept
{
	armAngle       = DefaultArmAngle;
	aTowerRotation = DefaultATowerRotation;
	bTowerRotation = DefaultBTowerRotation;
	cTowerRotation = DefaultCTowerRotation;
	printRadius    = DefaultPrintRadius;
	homedHeight    = DefaultHomedHeight;
	numTowers      = NumTowers;

     Recalc();
}


void ColinearTripteronKinematics::Recalc() noexcept
{
	// arm angle and tangent
	float arm_angle_tan = tanf(PIOVER180*armAngle);
	// tower rotations
	float a_rotation = PIOVER180*aTowerRotation;
	float b_rotation = PIOVER180*bTowerRotation;
	float c_rotation = PIOVER180*cTowerRotation;
    // tower reductions and tangent coefficients
	a_x = sinf(a_rotation) * arm_angle_tan;
	a_y = cosf(a_rotation) * arm_angle_tan;
	b_x = sinf(b_rotation) * arm_angle_tan;
	b_y = cosf(b_rotation) * arm_angle_tan;
	c_x = sinf(c_rotation) * arm_angle_tan;
	c_y = cosf(c_rotation) * arm_angle_tan;
    // forward kinematics matrix denominator
    //     d = a_y*b_x - g_y*b_x - a_x*b_y - a_y*g_x + b_y*g_x + a_x*g_y;
    denominator = a_y * b_x - c_y * b_x - a_x * b_y - a_y * c_x + b_y * c_x + a_x * c_y;
	printRadiusSquared = fsquare(printRadius);
	alwaysReachableHeight = homedHeight; // naive approach for now.
}


bool ColinearTripteronKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException)
{
// M669 K12 A30 R120 H220 T0:120:240 

    if (mCode == 669)
    {
        bool seen = false;
        gb.TryGetFValue('A', armAngle, seen);
        gb.TryGetFValue('R', printRadius, seen);
        gb.TryGetFValue('H', homedHeight, seen);
        if (gb.Seen('T'))
	{
            seen = true;
	    float towerRotations[3];
            size_t numTowerRotations = 3;
   	    gb.GetFloatArray(towerRotations, numTowerRotations, false);
            if(numTowerRotations == 3)
	    {
		aTowerRotation = towerRotations[0];
		bTowerRotation = towerRotations[1];
		cTowerRotation = towerRotations[2];
            }
	} 
	Recalc();
        return seen;
    }	
    else
    {
        return Kinematics::Configure(mCode, gb, reply, error);
    }
}


bool ColinearTripteronKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept 
{
	 /* Inverse Kinematics
     *
     * 3x3 reduction coefficient matrix * 1x3 coordinate matrix
     * [a_x -a_y 1]   [x]
     * [b_x -b_y 1] * [y]
     * [g_x -g_y 1]   [z]
     */

	// Store for cleaner equations
	//   float a_x = this->a_x, a_y = this->a_y;
    //   float b_x = this->b_x, b_y = this->b_y;
    //   float g_x = this->g_x, g_y = this->g_y;

    // actuator_mm[ALPHA_STEPPER] = a_x*cartesian_mm[X_AXIS] - a_y*cartesian_mm[Y_AXIS] + cartesian_mm[Z_AXIS];
    // actuator_mm[BETA_STEPPER ] = b_x*cartesian_mm[X_AXIS] - b_y*cartesian_mm[Y_AXIS] + cartesian_mm[Z_AXIS];
    // actuator_mm[GAMMA_STEPPER] = g_x*cartesian_mm[X_AXIS] - g_y*cartesian_mm[Y_AXIS] + cartesian_mm[Z_AXIS];

	bool ok = true;
        motorPos[A_TOWER] = (a_x*machinePos[X_AXIS] - a_y*machinePos[Y_AXIS] + machinePos[Z_AXIS])*stepsPerMm[X_AXIS];
	motorPos[B_TOWER] = (b_x*machinePos[X_AXIS] - b_y*machinePos[Y_AXIS] + machinePos[Z_AXIS])*stepsPerMm[X_AXIS];
        motorPos[C_TOWER] = (c_x*machinePos[X_AXIS] - c_y*machinePos[Y_AXIS] + machinePos[Z_AXIS])*stepsPerMm[X_AXIS];
	return ok;
}


void ColinearTripteronKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	 /* Forward Kinematics
     *
     * 3x3 inverted reduction coefficient matrix * 1x3 actuator matrix
     * [a_x -a_y 1]      [a]
     * [b_x -b_y 1]^-1 * [b]
     * [g_x -g_y 1]      [g]
     */

    // Store for cleaner equations
    // float a_x = this->a_x, a_y = this->a_y;
    // float b_x = this->b_x, b_y = this->b_y;
    // float g_x = this->g_x, g_y = this->g_y;
    // float d = this->d;

    // cartesian_mm[X_AXIS] = (actuator_mm[ALPHA_STEPPER]*(g_y-b_y) + actuator_mm[BETA_STEPPER]*(a_y-g_y) + actuator_mm[GAMMA_STEPPER]*(b_y-a_y))/d;
    // cartesian_mm[Y_AXIS] = (actuator_mm[ALPHA_STEPPER]*(g_x-b_x) + actuator_mm[BETA_STEPPER]*(a_x-g_x) + actuator_mm[GAMMA_STEPPER]*(b_x-a_x))/d;
    // cartesian_mm[Z_AXIS] = (actuator_mm[ALPHA_STEPPER]*(b_y*g_x-b_x*g_y) + actuator_mm[BETA_STEPPER]*(a_x*g_y-a_y*g_x) + actuator_mm[GAMMA_STEPPER]*(a_y*b_x-a_x*b_y))/d;
    // float a_tower_x = this->a_tower_x, a_tower_y = this->a_tower_y;
    // float b_tower_x = this->b_tower_x, b_tower_y = this->b_tower_y;
    // float c_tower_x = this->b_tower_x, c_tower_y = this->c_tower_y;

    machinePos[X_AXIS] = ((motorPos[A_TOWER]*(c_y-b_y) + motorPos[B_TOWER]*(a_y-c_y) + motorPos[C_TOWER]*(b_y-a_y)) / denominator)/stepsPerMm[X_AXIS];
    machinePos[Y_AXIS] = ((motorPos[A_TOWER]*(c_x-b_x) + motorPos[B_TOWER]*(a_x-c_x) + motorPos[C_TOWER]*(b_x-a_x)) / denominator)/stepsPerMm[Y_AXIS];
    machinePos[Z_AXIS] = ((motorPos[A_TOWER]*(b_y*c_x-b_x*c_y) + motorPos[B_TOWER]*(a_x*c_y-a_y*c_x) + motorPos[C_TOWER]*(a_y*b_x-a_x*b_y)) / denominator)/stepsPerMm[Z_AXIS];
}



// Return true if the specified XY position is reachable by the print head reference point.
bool ColinearTripteronKinematics::IsReachable(float x, float y, bool isCoordinated) const noexcept
{
	return fsquare(x) + fsquare(y) < printRadiusSquared;
}


// Limit position if instructed to movr outside reachable area
LimitPositionResult ColinearTripteronKinematics::LimitPosition(float finalCoords[], const float * null initialCoords,
												size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const noexcept
{
bool limited = false;

	// If axes have been homed on a colinear tripteron printer and this isn't a homing move, check for movements outside limits.
	// Skip this check if axes have not been homed, so that extruder-only moves are allowed before homing
	if ((axesHomed & XyzAxes) == XyzAxes)
	{
		// Constrain the move to be within the build radius
		const float diagonalSquared = fsquare(finalCoords[X_AXIS]) + fsquare(finalCoords[Y_AXIS]);
		if (applyM208Limits && diagonalSquared > printRadiusSquared)
		{
			const float factor = sqrtf(printRadiusSquared / diagonalSquared);
			finalCoords[X_AXIS] *= factor;
			finalCoords[Y_AXIS] *= factor;
			limited = true;
		}
		// Limit move to not go beyond Z axis maximum.
		// TODO hwa: find formula to get Z max limited by arm positions, the below will only stop too late!

		if (applyM208Limits && finalCoords[Z_AXIS] > reprap.GetPlatform().AxisMaximum(Z_AXIS))
		{
			finalCoords[Z_AXIS] = reprap.GetPlatform().AxisMaximum(Z_AXIS);
			limited = true;
		}

		// Limit move to not go beyond Z axis minimum.
		if (applyM208Limits && finalCoords[Z_AXIS] < reprap.GetPlatform().AxisMinimum(Z_AXIS))
		{
			finalCoords[Z_AXIS] = reprap.GetPlatform().AxisMinimum(Z_AXIS);
			limited = true;
		}
	}

	// Limit any additional axes according to the M208 limits
	if (applyM208Limits && Kinematics::LimitPositionFromAxis(finalCoords, numTowers, numVisibleAxes, axesHomed))
	{
		limited = true;
	}

	return (limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;
}


// Return the initial Cartesian coordinates we assume after switching to this kinematics
void ColinearTripteronKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept
{
	for (size_t i = 0; i < numAxes; ++i)
	{
		positions[i] = 0.0;
	}
	positions[Z_AXIS] = 0;
}


// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
AxesBitmap ColinearTripteronKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept
{
	// If homing X, Y or Z we must home all the towers
	if (toBeHomed.Intersects(XyzAxes))
	{
		filename.copy("homect.g");
		return AxesBitmap();
	}

	return Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);
}


// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool ColinearTripteronKinematics::QueryTerminateHomingMove(size_t axis) const noexcept
{
	return false;
}


// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate().
void ColinearTripteronKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept
{
	if (axis < numTowers)
	{
		if (highEnd)
		{
			dda.SetDriveCoordinate(	(homedHeight * stepsPerMm[axis]), axis);
		}
	}
	else
	{
		// Assume that any additional axes are linear
		const float hitPoint = (highEnd) ? reprap.GetPlatform().AxisMaximum(axis) : reprap.GetPlatform().AxisMinimum(axis);
		dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
	}
}


// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
AxesBitmap ColinearTripteronKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept
{
	// If all of X, Y and Z have been specified then we know the positions of all 3 tower motors, otherwise we don't
	if ((g92Axes & XyzAxes) != XyzAxes)
	{
		g92Axes &= ~XyzAxes;
	}
	return g92Axes;
}


// Return the set of axes that must be homed prior to regular movement of the specified axes
AxesBitmap ColinearTripteronKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept
{
	if (axesMoving.Intersects(XyzAxes))
	{
		axesMoving |= XyzAxes;
	}
	return axesMoving;
}


// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds in Cartesian space have already been limited.
void ColinearTripteronKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept
{
	// Limit the speed in the XY plane to the lower of the X and Y maximum speeds, and similarly for the acceleration
	const float xyFactor = sqrtf(fsquare(normalisedDirectionVector[X_AXIS]) + fsquare(normalisedDirectionVector[Y_AXIS]));
	if (xyFactor > 0.01)
	{
		const Platform& platform = reprap.GetPlatform();
		const float maxSpeed = min<float>(platform.MaxFeedrate(X_AXIS), platform.MaxFeedrate(Y_AXIS));
		const float maxAcceleration = min<float>(platform.Acceleration(X_AXIS), platform.Acceleration(Y_AXIS));
		dda.LimitSpeedAndAcceleration(maxSpeed/xyFactor, maxAcceleration/xyFactor);
	}
}

AxesBitmap ColinearTripteronKinematics::GetLinearAxes() const noexcept
{
	        return AxesBitmap::MakeFromBits(Z_AXIS);
}

/*
 *  # TODO
 *  ## Must
 *  - LimitPosition() "done"
 *  ## Should
 *  - EndStop Adjustment
 *  - Tower Tilt?
 *
 */
