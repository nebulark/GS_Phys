#pragma once
#include "reactphysics3d.h"
class FerroMagnet
{
public:
	FerroMagnet();

	const rp3d::Vector3& GetPosition() { return m_position; }
	const rp3d::Vector3& GetMagnetizationVector() { return m_magnetizationVector * m_strengthMulitplier; }

	void Update(
		const rp3d::Vector3& PendulumPosition, const rp3d::Vector3& PendulumMagnetizationDir,
		const float PendulumMagnetizationStrength, float DelataTime
	);
private:
	rp3d::Vector3 m_magnetizationVector;
	float m_maxResidualStrength;

	// the Magnetization vector  gest multiplied by this value, before accessed by user
	// it resembles the differnce betwwen the field that enabled the magnet and the real magnetic field (much higher)
	float m_strengthMulitplier;

	// if we remove external field right now how much residual strength would be left
	float m_currentResidualStrength;

	float m_residualFraction;

	rp3d::Vector3 m_position;
};