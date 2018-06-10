#include "stdafx.h"
#include "Simulation.h"

namespace
{
	float DegreesToRadians(float Degrees)
	{
		return Degrees / 180.f * rp3d::PI;
	}

	float RadiansToDegrees(float Radians)
	{
		return Radians / rp3d::PI * 180.f;
	}

	rp3d::Vector3 RadiansToVectorRotateInZAxis(float Radians)
	{
		return rp3d::Vector3(std::cos(Radians), std::sin(Radians), 0.f);
	}

	float VectorZAxisRotationInRadians(rp3d::Vector3 vector)
	{
		return std::atan2(vector.y, vector.x);
	}

	const float DrawDistanceForMeter = 100;


	const rp3d::Vector3 gravity = rp3d::Vector3(0.f, 9.81f , 0.f);
	const float pendulumStartingAngle = DegreesToRadians(-1.f);
	const float pendulumLength = 4.f;
	const rp3d::Vector3 ferroMagnetPosition = rp3d::Vector3(0.f, 5.f, 0.f);
	const rp3d::Vector3 pendulumAnchorPostion = rp3d::Vector3(0.f, -3.f,0.f);
	const rp3d::Vector3 pendulumStartingOffset = RadiansToVectorRotateInZAxis(pendulumStartingAngle) * pendulumLength;


	struct AttractionAndTorque
	{
		rp3d::Vector3 attraction;
		rp3d::Vector3 torque;
	};
	
	AttractionAndTorque CalcFerroMagnetAttractionAndTorque(
		rp3d::Vector3 PendulumPosition, rp3d::Vector3 PendulumMagnetizationDirection, float PendulumMagnetForceStrenth,
		rp3d::Vector3 FerroMagnetPosition, rp3d::Vector3 FerroMagnetMagnetizationDirection, float FerroMagnetForceStrenth)
	{
		assert(std::abs(PendulumMagnetizationDirection.lengthSquare() - 1) < 0.00001f);
		assert(std::abs(FerroMagnetMagnetizationDirection.lengthSquare() - 1) < 0.00001f);

		const rp3d::Vector3 PendulumToFerroMagnet = FerroMagnetPosition - PendulumPosition;
		const float PendulumToFerroMagnetDistanceSq = PendulumToFerroMagnet.lengthSquare();
		const float PendulumToFerroMagnetDistance = std::sqrt(PendulumToFerroMagnetDistanceSq);
		const rp3d::Vector3 PendulumToFerroMagnetDirection = PendulumToFerroMagnet / PendulumToFerroMagnetDistance;

		const float totalForce = (PendulumMagnetForceStrenth + FerroMagnetForceStrenth) / PendulumToFerroMagnetDistanceSq;

		// the more parallel, the higher the attraction
		const float relativeAttraction = (FerroMagnetMagnetizationDirection).dot(PendulumMagnetizationDirection);
		const rp3d::Vector3 relativeAttractionVector = relativeAttraction * PendulumToFerroMagnetDirection;
		const rp3d::Vector3 attractionVector = relativeAttractionVector * totalForce;

		// the more perpendicular, the higher the z component of the resulting vector, also has the right direction sign
		const rp3d::Vector3 relativeTorqueVector = (PendulumMagnetizationDirection).cross(FerroMagnetMagnetizationDirection);
		const rp3d::Vector3 TorqueVector = relativeTorqueVector * totalForce;

		return { attractionVector, TorqueVector };
	}



	void DrawArrow(sf::RenderWindow& rw, sf::Vector2f begin, sf::Vector2f end, sf::Color color)
	{
		const sf::Vector2f endToBegin = begin - end;
		const sf::Vector2f perpendicular(-endToBegin.y, endToBegin.x);

		const sf::Vector2f arrowVec1 = endToBegin * 0.3f + perpendicular * 0.2f;
		const sf::Vector2f arrowVec2 = endToBegin * 0.3f - perpendicular * 0.2f;


		sf::Vertex lines[6];
		for (sf::Vertex& line : lines)
		{
			line.color = color;
		}

		lines[0].position = begin;
		lines[1].position = end;

		lines[2].position = end;
		lines[3].position = end + arrowVec1;

		lines[4].position = end;
		lines[5].position = end + arrowVec2;

		rw.draw(lines, 6, sf::Lines);
	}

	rp3d::Vector3 CalcForwardVector(const rp3d::Transform& trans)
	{
		return trans.getOrientation() * rp3d::Vector3(1.f, 0.f, 0.f);
	}
}


Simulation::Simulation()
	: m_world(gravity)
	, m_ferroMagnetPosition(0.f, 3.f, 0.f)
	, m_ferroMagnetShape(sf::Vector2f(50.f,50.f))
	, m_pendulumShape(sf::Vector2f(50.f, 50.f))
	, m_pendulumAnchorShape(20.f)
	, m_pendulumMagneticStrength(20.f)
	, m_ferroMagnetMagnetizationVector(RadiansToVectorRotateInZAxis(DegreesToRadians(30.f)))
{
}

void Simulation::Init()
{
	const rp3d::Transform pendulumStartTransform(
		rp3d::Vector3(pendulumAnchorPostion.x + pendulumStartingOffset.x, pendulumAnchorPostion.y + pendulumStartingOffset.y, 0.f),
		rp3d::Quaternion(0.f, 0.f, pendulumStartingAngle)
	);
	m_pendulum = m_world.createRigidBody(pendulumStartTransform);
	m_pendulum->setInertiaTensorLocal(rp3d::Matrix3x3::identity());

	m_dummyBody = m_world.createRigidBody(rp3d::Transform(rp3d::Vector3(0.f, 0.f, 0.f),
		rp3d::Quaternion(0.f, 0.f, 0.f)));
	

	m_dummyBody->setType(rp3d::BodyType::STATIC);

	rp3d::HingeJointInfo jointInfo(m_pendulum, m_dummyBody, 
		rp3d::Vector3( pendulumAnchorPostion.x, pendulumAnchorPostion.y,0.f), rp3d::Vector3(0.f, 0.f, 1.f));
	m_pendulumJoint = static_cast<rp3d::HingeJoint*>(m_world.createJoint(jointInfo));

	m_pendulum->setLinearDamping(0);
}




void Simulation::Update(float deltaTime)
{
	
	const rp3d::Transform pendulumTranform = m_pendulum->getTransform();
	const rp3d::Vector3 pendulumPosition = pendulumTranform.getPosition();
	const rp3d::Vector3 pendulumMagnetizationDirection = CalcForwardVector(pendulumTranform);

	// update Pendulum
	const float FerroMagneticStrengthSq = m_ferroMagnetMagnetizationVector.lengthSquare();
	const float FerroMagneticStrength = std::sqrt(FerroMagneticStrengthSq);
	AttractionAndTorque forcesOnPendulum = CalcFerroMagnetAttractionAndTorque(pendulumPosition,
		pendulumMagnetizationDirection, m_pendulumMagneticStrength,
		m_ferroMagnetPosition, m_ferroMagnetMagnetizationVector / FerroMagneticStrength, FerroMagneticStrength);
	
	m_pendulum->applyForceToCenterOfMass(forcesOnPendulum.attraction);
	m_pendulum->applyTorque(forcesOnPendulum.torque);

	// ferromagnet
	const rp3d::Vector3 pendulumMagnetizationVector = pendulumMagnetizationDirection * m_pendulumMagneticStrength;
	const float distancePendulumFerroSq = (pendulumPosition - m_ferroMagnetPosition).lengthSquare();
	// the nearer it is the more effect it has
	const float pendulumMagnetisationEffect = 1 / (1 + distancePendulumFerroSq);

	// how much the ferromagnet wants to change, the more magnetized the less he wants to change
	const float ferroMagnetizationChangeRate = 1 / (1 + (std::pow(FerroMagneticStrengthSq, 1)));

	constexpr float LearnRateModifer = 10.f;

	const float totalChangeRate = pendulumMagnetisationEffect * ferroMagnetizationChangeRate * deltaTime * LearnRateModifer;

	// mix, old vector with new Vector
	m_ferroMagnetMagnetizationVector = 
			(m_ferroMagnetMagnetizationVector * (1 - totalChangeRate))
		+	(pendulumMagnetizationVector)	* (totalChangeRate);


	m_world.update(deltaTime);
}

void Simulation::Render(sf::RenderWindow& renderWindow)
{
	
	const sf::Vector2f sfAnchorPosition(pendulumAnchorPostion.x * DrawDistanceForMeter,pendulumAnchorPostion.y* DrawDistanceForMeter);

	m_pendulumAnchorShape.setOrigin(m_pendulumAnchorShape.getRadius(), m_pendulumAnchorShape.getRadius());
	m_pendulumAnchorShape.setPosition(sfAnchorPosition);
	renderWindow.draw(m_pendulumAnchorShape);

	rp3d::Transform pendulumTranform = m_pendulum->getTransform();
	const rp3d::Vector3 pendulumForwardVector = CalcForwardVector(pendulumTranform);
	const rp3d::Vector3 pendulumMagnetizationDir = pendulumForwardVector;
	sf::Vector2f sfpendulumPosition(pendulumTranform.getPosition().x* DrawDistanceForMeter, pendulumTranform.getPosition().y* DrawDistanceForMeter);

	m_pendulumShape.setOrigin(m_pendulumShape.getSize() / 2.f);
	m_pendulumShape.setRotation(RadiansToDegrees(VectorZAxisRotationInRadians(pendulumForwardVector)));
	m_pendulumShape.setPosition(sfpendulumPosition);
	renderWindow.draw(m_pendulumShape);

	{
		sf::Vertex line[2];
		line[0].position = sfpendulumPosition;
		line[0].color = sf::Color::Green;

		line[1].position = sfAnchorPosition;
		line[1].color = sf::Color::Green;
		renderWindow.draw(line, 2, sf::Lines);
	}
	{
		sf::Vector2f endPos = sfpendulumPosition + sf::Vector2f(pendulumMagnetizationDir.x, pendulumMagnetizationDir.y) * m_pendulumMagneticStrength * 5.f;
		DrawArrow(renderWindow, sfpendulumPosition, endPos, sf::Color::Red);
	}

	sf::Vector2f sfFerroMagnetPositon = sf::Vector2f(m_ferroMagnetPosition.x , m_ferroMagnetPosition.y) * DrawDistanceForMeter;
	m_ferroMagnetShape.setOrigin(m_ferroMagnetShape.getSize() / 2.f);
	m_ferroMagnetShape.setPosition(sfFerroMagnetPositon);
	renderWindow.draw(m_ferroMagnetShape);
	{
		sf::Vector2f endPos = sfFerroMagnetPositon + sf::Vector2f(m_ferroMagnetMagnetizationVector.x, m_ferroMagnetMagnetizationVector.y) * 5.f;
		DrawArrow(renderWindow, sfFerroMagnetPositon, endPos, sf::Color::Magenta);
	}

	/*{
		AttractionAndTorque forcesOnPendulum = CalcFerroMagnetAttractionAndTorque(pendulumTranform.getPosition(),
			pendulumMagnetizationDir, 1.f,
			m_ferroMagnetPosition, m_ferroMagnetMagnetizationDirection, m_ferroMagnetForceStrenth);
		sf::Vector2f endPosOffset = sf::Vector2f(forcesOnPendulum.attraction.x, forcesOnPendulum.attraction.y);

		DrawArrow(renderWindow, sfpendulumPosition, sfpendulumPosition + endPosOffset * DrawDistanceForForces, sf::Color::Magenta);
	}

	{
		rp3d::Vector3 velocity = m_pendulum->getLinearVelocity();
		sf::Vector2f sfVelocity(velocity.x, velocity.y);
		DrawArrow(renderWindow, sfpendulumPosition, sfpendulumPosition + sf::Vector2f(0.f,gravity.y) * DrawDistanceForForces, sf::Color::Blue);
	}*/
}
