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

	rp3d::Vector2 RadiansToVector(float Radians)
	{
		return rp3d::Vector2(std::cos(Radians), std::sin(Radians));
	}

	const float Meter = 100;

	const rp3d::Vector3 gravity = rp3d::Vector3(0.f, 9.81f , 0.f) * Meter;
	const float pendulumStartingAngle = DegreesToRadians(-1.f);
	const float pendulumLength = 4.f * Meter;
	const rp3d::Vector2 pendulumAnchorPostion = rp3d::Vector2(0.f, -3.f) * Meter;
	const rp3d::Vector2 pendulumStartingOffset = RadiansToVector(pendulumStartingAngle) * pendulumLength;
}


Simulation::Simulation()
	: m_world(gravity)
	, m_metalBlockPosition(0.f, 4 * Meter)
	, m_metalBlockShape(sf::Vector2f(50.f,50.f))
	, m_pendulumShape(sf::Vector2f(50.f, 50.f))
	, m_pendulumAnchorShape(20.f)
{
}

void Simulation::Init()
{
	const rp3d::Transform pendulumStartTransform(
		rp3d::Vector3(pendulumAnchorPostion.x + pendulumStartingOffset.x, pendulumAnchorPostion.y + pendulumStartingOffset.y, 0.f),
		rp3d::Quaternion(0.f, 0.f, pendulumStartingAngle)
	);
	m_pendulum = m_world.createRigidBody(pendulumStartTransform);
	m_pendulum->setMass(10.f);
	m_pendulum->setInertiaTensorLocal(rp3d::Matrix3x3::identity());

	m_dummyBody = m_world.createRigidBody(rp3d::Transform(rp3d::Vector3(0.f, 100.f, 0.f),
		rp3d::Quaternion(0.f, 0.f, 0.f)));
	m_dummyBody->setMass(10.f);
	

	m_dummyBody->setType(rp3d::BodyType::STATIC);

	rp3d::HingeJointInfo jointInfo(m_pendulum, m_dummyBody, 
		rp3d::Vector3( pendulumAnchorPostion.x, pendulumAnchorPostion.y,0.f), rp3d::Vector3(0.f, 0.f, 1.f));
	m_pendulumJoint = static_cast<rp3d::HingeJoint*>(m_world.createJoint(jointInfo));
}

void Simulation::Update(float deltaTime)
{
	m_world.update(deltaTime);
}

void Simulation::Render(sf::RenderWindow& renderWindow)
{
	
	const sf::Vector2f sfAnchorPosition(pendulumAnchorPostion.x,pendulumAnchorPostion.y);

	m_pendulumAnchorShape.setOrigin(m_pendulumAnchorShape.getRadius(), m_pendulumAnchorShape.getRadius());
	m_pendulumAnchorShape.setPosition(sfAnchorPosition);
	renderWindow.draw(m_pendulumAnchorShape);

	rp3d::Transform pendulumTranform = m_pendulum->getTransform();
	float pendulumRadiansAngle;
	sf::Vector2f sfpendulumPosition(pendulumTranform.getPosition().x, pendulumTranform.getPosition().y);
	rp3d::Vector3 axis;
	pendulumTranform.getOrientation().getRotationAngleAxis(pendulumRadiansAngle, axis);
	m_pendulumShape.setOrigin(m_pendulumShape.getSize() / 2.f);
	m_pendulumShape.setRotation(RadiansToDegrees(pendulumRadiansAngle));
	m_pendulumShape.setPosition(sfpendulumPosition);
	renderWindow.draw(m_pendulumShape);

	sf::Vertex line[2];
	line[0].position = sfpendulumPosition;
	line[0].color = sf::Color::Green;

	line[1].position = sfAnchorPosition;
	line[1].color = sf::Color::Green;
	renderWindow.draw(line, 2, sf::Lines);
	
}
