#include "stdafx.h"
#include "FerroMagnet.h"


FerroMagnet::FerroMagnet()
	: m_magnetizationVector(0.f,0.001f,0.f)
	, m_currentResidualStrength(0.f)
	, m_strengthMulitplier(5.f)
	, m_residualFraction(0.5f)
	, m_position(0.f, 2.f,0.f)
{
}

void FerroMagnet::Update(
	const rp3d::Vector3& PendulumPosition, const rp3d::Vector3& PendulumMagnetizationDir,
	const float PendulumMagnetizationStrength, float DeltaTime)
{
	float DelataAsFractionMultiplier = (1 / DeltaTime);

	const float DistanceSq = (PendulumPosition - m_position).lengthSquare();
	const float Distance = std::sqrt(DistanceSq);

	const float ReducedPendulumStrength = PendulumMagnetizationStrength / (1 + Distance);

	const rp3d::Vector3 EffektivePendulumMagnetization = ReducedPendulumStrength * PendulumMagnetizationDir;

	const float CurrentMagnetizationStrengthSq = m_magnetizationVector.lengthSquare();
	const float CurrentMagnetizationStrength = std::sqrt(CurrentMagnetizationStrengthSq);

	// if 1 we don't decay a we have reach the residual strength
	float decayMultiplier = (m_currentResidualStrength / CurrentMagnetizationStrength);
	decayMultiplier = 1 - ((1 - decayMultiplier) * DeltaTime);

	assert(0 <= decayMultiplier && decayMultiplier <= 1.f);


	const rp3d::Vector3 CurrentMagnetizationDir = m_magnetizationVector / CurrentMagnetizationStrength;

	const rp3d::Vector3 MagnetizationDelta = EffektivePendulumMagnetization - m_magnetizationVector;
	float inverseCurrentStrength = 10 / (1 + CurrentMagnetizationStrengthSq);

	const rp3d::Vector3 NewMagnetizationVector = m_magnetizationVector * decayMultiplier + MagnetizationDelta * inverseCurrentStrength * DeltaTime;
	const float NewMagnetizationVectorStrength = NewMagnetizationVector.length();
	const float StrengthDelta = NewMagnetizationVectorStrength - CurrentMagnetizationStrength;

	m_currentResidualStrength += StrengthDelta * m_residualFraction;
	m_magnetizationVector = NewMagnetizationVector;
}
