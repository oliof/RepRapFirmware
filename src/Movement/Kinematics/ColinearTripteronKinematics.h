/*
 * ColinearTripteronKinematics.h
 *
 *  Created on: 24 May 2020
 *      Author: oliof, apsu
 *
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

protected:
	DECLARE_OBJECT_MODEL

private:
	static constexpr float DefaultArmAngle = 30;
	static constexpr float DefaultATowerRotation = 0;
	static constexpr float DefaultBTowerRotation = 1;
	static constexpr float DefaultCTowerRotation = 2;
	static constexpr float DefaultPrintRadius = 120;
        static constexpr float DefaultHomedHeight = 220;

        void Init();
        void Recalc(); 
}
