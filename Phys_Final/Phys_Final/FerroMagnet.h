#pragma once
#include "reactphysics3d.h"
class FerroMagnet
{
public:
	const rp3d::Vector3& GetPosition() { return m_ferroMagnetPosition; }
	const rp3d::Vector3& GetMagnetizationVector() { return m_magnetizationVector; }

	void Update(
		const rp3d::Vector3& PendulumPosition, const rp3d::Vector3& PendulumMagnetizationDir,
		const float PendulumMagnetizationStrength, float DelataTime
	);
private:
	rp3d::Vector3 m_magnetizationVector;
	float m_maxResidualStrength;

	// if we remove external field right now how much residual strength would be left
	float m_currentResidualStrength;

	float m_maxStrength;

	// how much external strength we need to reach max strength
	float m_maxExternalStrength;

	rp3d::Vector3 m_ferroMagnetPosition;
};