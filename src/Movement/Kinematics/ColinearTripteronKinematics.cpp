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
