#pragma once
#include "reactphysics3d.h"
#include <SFML/System/Vector2.hpp>
#include <SFML/Graphics/RectangleShape.hpp>


namespace sf
{
	class RenderWindow;
}


class Simulation
{
public:
	Simulation();

	void Init();
	void Update(float deltaTime);
	void Render(sf::RenderWindow& renderWindow);
private:
	rp3d::DynamicsWorld m_world;

	sf::RectangleShape m_ferroMagnetShape;
	sf::RectangleShape m_pendulumShape;
	sf::CircleShape m_pendulumAnchorShape;

	rp3d::RigidBody* m_dummyBody;
	rp3d::RigidBody* m_pendulum;
	rp3d::HingeJoint* m_pendulumJoint;

	float m_ferroMagnetForceStrenth;
	rp3d::Vector3 m_ferroMagnetMagnetizationDirection;
	rp3d::Vector3 m_ferroMagnetPosition;

};

