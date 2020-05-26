/*
 * ColinearTripteronKinematics.cpp
 *
 *  Created on: 24 May 2020
 *      Author: oliof, apsu
 *
 */

#include "ColinearTripteronKinematics.h"
#include "RepRap.h"
#include "Platform.h"
#include "Storage/MassStorage.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

#include <limits>

#define PIOVER180 0.01745329251994329576923690768489F


// Return the name of the current kinematics
const char *ColinearTripteronKinematics::GetName(bool forStatusReport) const noexcept
{
        return (forStatusReport) ? "colineartripteron" : "Colinear Tripteron";
}


void ColinearTripteronKinematics::Init() const noexcept
{
	armAngle = DefaultArmAngle;
	aTowerRotation = DefaultATowerRotation;
	bTowerRotation = DefaultBTowerRotation;
	cTowerRotation = DefaultCTowerRotation;
	printRadius    = DefaultPrintRadius;
	homedHeight    = DefaultHomedHeight;

        Recalc();
}


void ColinearTripteronKinematics::Recalc()
{
	// arm angle and tangent
	arm_angle = PIOVER180*armAngle;
	arm_angle_tan = tanf(arm_angle);
	// tower rotations
	a_tower_rotation = PIOVER180*aTowerRotation;
	b_tower_rotation = PIOVER180*bTowerRotation;
	c_tower_rotation = PIOVER180*bTowerRotation;
        // tower reductions and tangent coefficients
	a_tower_x = sinf(a_tower_rotation) * arm_angle_tan;
	a_tower_y = cosf(a_tower_rotation) * arm_angle_tan;
	b_tower_x = sinf(b_tower_rotation) * arm_angle_tan;
	b_tower_x = cosf(b_tower_rotation) * arm_angle_tan;
	c_tower_x = sinf(c_tower_rotation) * arm_angle_tan;
	c_tower_x = cosf(c_tower_rotation) * arm_angle_tan;
        // forward kinematics matrix denominator
        denominator = a_tower_y * b_tower_x -
                      c_tower_y * b_tower_x -
                      a_tower_y * b_tower_y -
                      a_tower_y * c_tower_x +
                      b_tower_y * c_tower_x +
                      a_tower_x * g_tower_y;
}


bool ColinearTripteronKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException)
{
// M669 K12 A30 R120 H220 T0:120:240 

	if (mcode = 669)
	{
		bool seen = false;
                gb.TryGetFValue('A', armAngle, seen);
                gb.TryGetFValue('R', printRadius, seen);
                gb.TryGetFValue('H', homedHeight, seen);
                if (gb.Seen('T')
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
        return seen;
	}
}


// Return true if the specified XY position is reachable by the print head reference point.
bool ColinearTripteronKinematics::IsReachable(float x, float y, bool isCoordinated) const noexcept
{
	return fsquare(x) + fsquare(y) < printRadiusSquared;
}


// Return the initial Cartesian coordinates we assume after switching to this kinematics
void ColinearTripteronKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept
{
	for (size_t i = 0; i < numAxes; ++i)
	{
		positions[i] = 0.0;
	}
	positions[Z_AXIS] = homedHeight;
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