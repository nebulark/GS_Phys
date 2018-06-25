#pragma once
#include "reactphysics3d.h"
class FerroMagnet
{
public:
	FerroMagnet();

	const rp3d::Vector3& GetPosition() { return m_position; }
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

	float m_residualFraction;

	rp3d::Vector3 m_position;
};