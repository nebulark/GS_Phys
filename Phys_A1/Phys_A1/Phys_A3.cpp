#include "stdafx.h"
#include "Phys_A3.h"

namespace
{
	const rp3d::Vector3 earthGravity(0.0, -9.81, 0.0);
	constexpr float gravitationalConstant = 6.67408e-11;

	float RadToDeg(float angleInRadians) {
		return angleInRadians * 180.f / M_PI;
	}

	float DegToRad(float angleInDegrees) {
		return angleInDegrees * M_PI / 180.f;
	}

	void Phys_A3_1()
	{
		std::puts(" === Phys A3 1) === ");
		rp3d::DynamicsWorld world(rp3d::Vector3::zero());
		world.setNbIterationsVelocitySolver(15);

		const float moonRadius = 1740 * 1000;
		const float orbitDistance = 95 * 1000;
		const float moonGravityField = 4.9039e12;

		const float totalRadius = moonRadius + orbitDistance;
		const float preCalculatedGravity = moonGravityField / (totalRadius * totalRadius);

		const rp3d::Transform sateliteInitialPos(rp3d::Vector3(0.f, totalRadius, 0.f), rp3d::Matrix3x3::identity());
		const rp3d::Vector3 sateliteInitialVelocity(1634.8f, 0.f, 0.f);

		rp3d::RigidBody* satelite = world.createRigidBody(sateliteInitialPos);
		satelite->setLinearVelocity(sateliteInitialVelocity);
		satelite->setType(reactphysics3d::BodyType::DYNAMIC);
		satelite->setMass(1.f);
		satelite->enableGravity(false);

		float lastSpeed = 0;
		float boxSpeed = 0;

		constexpr float tickRate = 0.01f;
		constexpr float simulationSeconds = 7052.829f;
		const int ticks = static_cast<int>(std::ceil(simulationSeconds / tickRate));

		for (int i = 0; i < ticks; ++i) {
			const rp3d::Vector3 satelitePos = satelite->getTransform().getPosition();
			const float distance = satelitePos.length();
			//std::printf("the satelite is %f meters away from the moon Center", distance);
			const rp3d::Vector3 forceDir = -satelitePos / distance;
			const float forceMultiplier = preCalculatedGravity;//moonGravityField / (distance * distance);

			satelite->applyForceToCenterOfMass(forceMultiplier * -satelitePos.getUnit());
			world.update(tickRate);
		}

		float distanceToStart = (satelite->getTransform().getPosition() - sateliteInitialPos.getPosition()).length();
		std::printf("the satelite is %f meters away from the starting position \n" , distanceToStart);

	}
	void Phys_A3_2()
	{
		std::puts(" === Phys A3 2) === ");
		rp3d::DynamicsWorld world(rp3d::Vector3::zero());
		world.setNbIterationsVelocitySolver(15);

		const float closesDistanceToSun = 149597870700.f;
		const float sunMass = 1.98855e30;
		const float sunGravityParam = gravitationalConstant * sunMass;
		const float speedClose = 4.2064e+04;

		const rp3d::Transform cometInitialPos(rp3d::Vector3(0.f, closesDistanceToSun, 0.f), rp3d::Matrix3x3::identity());
		const rp3d::Vector3 cometInitialVelocity(speedClose, 0.f, 0.f);

		rp3d::RigidBody* comet = world.createRigidBody(cometInitialPos);
		comet->setLinearVelocity(cometInitialVelocity);
		comet->setType(reactphysics3d::BodyType::DYNAMIC);
		comet->setMass(1.f);
		comet->enableGravity(false);

		constexpr float tickRate = 10'000.f;
		constexpr float secondsInYear = 365.24f * 24.f * 60.f * 60.f;
		constexpr float simulationSeconds = secondsInYear * 2400.f;
		const int ticks = static_cast<int>(std::ceil(simulationSeconds * 1.1f / tickRate));

		bool hasHadNegativeXPos = false;
		for (int i = 0; i < ticks; ++i) {
			const rp3d::Vector3 cometPos = comet->getTransform().getPosition();
			const rp3d::Vector3 cometVel = comet->getLinearVelocity();

			if (cometPos.x < 0)
			{
				hasHadNegativeXPos = true;
			}
	
			if (hasHadNegativeXPos && cometPos.x >= 0)
			{
				const float timeTaken = i * tickRate;
				std::printf("the comet is took %e seconds / %f years \n", timeTaken, timeTaken / secondsInYear);
				break;
			}
			const float distance = cometPos.length();
			//std::printf("the satelite is %f meters away from the moon Center", distance);
			const rp3d::Vector3 forceDir = -cometPos / distance;
			const float forceMultiplier = sunGravityParam / (distance * distance);

			comet->applyForceToCenterOfMass(forceMultiplier * forceDir);
			world.update(tickRate);
			
		}

		float distanceToStart = (comet->getTransform().getPosition() - cometInitialPos.getPosition()).length();
		std::printf("the comet is %e meters away from the starting position \n", distanceToStart);
	}

	void Phys_A3_3()
	{
		std::puts(" === Phys A3 3) === ");

		const rp3d::Vector3 boxHalfExtents(0.5, 0.5, 0.5);
		const rp3d::Vector3 slopeHalfExtents(100.0, 1.0, 3.0);
		const rp3d::Vector3 boxStartPosition(0.0, 1.60, 0.0);
		const rp3d::Vector3 slopePosition(0.0, 0.0, 0.0);

		rp3d::DynamicsWorld world(earthGravity);
		world.setNbIterationsVelocitySolver(15);

		float inclineDegrees = 25.f;
		float boxWeight = 380.f;
		float frictionCoefficient = 0.f;
		const rp3d::Vector3 localBoxVelocity(1.f, 0, 0);

		rp3d::Quaternion slopeOrientation(rp3d::Vector3(0, 0, -DegToRad(inclineDegrees)));

		const rp3d::Vector3 worldBoxVelocity = slopeOrientation * localBoxVelocity;

		//Create Box Object
		rp3d::BoxShape boxBoxShape(boxHalfExtents);
		rp3d::Transform boxTransform(boxStartPosition, slopeOrientation);
		rp3d::RigidBody* box;
		box = world.createRigidBody(boxTransform);
		box->setType(reactphysics3d::BodyType::DYNAMIC);
		box->addCollisionShape(&boxBoxShape, rp3d::Transform::identity(), boxWeight);
		box->setLinearVelocity(worldBoxVelocity);

		//Create Slope Object
		rp3d::BoxShape slopeBoxShape(slopeHalfExtents);
		rp3d::Transform slopeTransform(slopePosition, slopeOrientation);
		rp3d::RigidBody* slope;
		slope = world.createRigidBody(slopeTransform);
		slope->setType(reactphysics3d::BodyType::STATIC);
		slope->addCollisionShape(&slopeBoxShape, rp3d::Transform::identity(), 1);

		//Set Material of Slope
		rp3d::Material& slopeMaterial = slope->getMaterial();
		slopeMaterial.setBounciness(rp3d::decimal(0));
		slopeMaterial.setFrictionCoefficient(rp3d::decimal(frictionCoefficient));

		//Set Material of Box
		rp3d::Material& boxMaterial = box->getMaterial();
		boxMaterial.setBounciness(rp3d::decimal(0));
		boxMaterial.setFrictionCoefficient(rp3d::decimal(frictionCoefficient));

		float lastSpeed = worldBoxVelocity.length();

		constexpr float tickRate = 0.001f;
		constexpr float simulationSeconds = 5.f;
		constexpr int ticks = simulationSeconds / tickRate + 1;
		const float measuredTimeSpan = 0.1f;
		float secondsPassedSinceMeasurement = 0.f;
		rp3d::Vector3 boxHighestPoint;
		constexpr float manForce = -1575.4364;

		int totalTicks = 0;
		for (int i = 0; i < ticks; ++i) 
		{

			const rp3d::Vector3 boxPosition = box->getTransform().getPosition();
			const rp3d::Vector3 boxVelocity = box->getLinearVelocity();
			const rp3d::Vector3 boxMovingDirection = boxVelocity.getUnit();
			const float boxSpeed = boxVelocity.length();

			box->applyForceToCenterOfMass(manForce * boxMovingDirection);
			world.update(tickRate);

			const rp3d::Vector3 newBoxVelocity = box->getLinearVelocity();

			std::printf("Box Velocity = (%f, %f) \n", newBoxVelocity.x, newBoxVelocity.y);
			
		}
	}

	void Phys_A3_5()
	{
		std::puts(" === Phys A3 5) === ");

		rp3d::DynamicsWorld world(earthGravity);
		world.setNbIterationsVelocitySolver(15);

		float personMass = 62.f;
			
		const rp3d::Vector3 personInitialVelocity(0.f, 4.5f, 0);



		
		rp3d::Transform personStartTransform(rp3d::Vector3(0.f,2.f,0.f), rp3d::Matrix3x3::identity());
		rp3d::RigidBody* person;
		person = world.createRigidBody(personStartTransform);
		person->setType(reactphysics3d::BodyType::DYNAMIC);
		person->setMass(personMass);
		
		person->setLinearVelocity(personInitialVelocity);


			
		constexpr float springConstant = 5.8e4;
		constexpr float tickRate = 0.001f;
		constexpr float simulationSeconds = 5.f;
		const int ticks = static_cast<int>(std::ceil(simulationSeconds / tickRate));
		float secondsPassed = 0.f;
		float secondsPassedSinceMeasurement = 0.f;
		const float measuredTimeSpan = 1.f;

		rp3d::Vector3 boxHighestPoint;
		float lowestPoint = 0;
		bool hadContact = false;
		for (int i = 0; i < ticks; ++i)
		{
			world.update(tickRate);
			const rp3d::Vector3 personPosition = person->getTransform().getPosition();
			const rp3d::Vector3 personVelocity = person->getLinearVelocity();
			const float boxSpeed = personVelocity.length();

			// if < 0 aplly spring force
			if (personPosition.y < 0)
			{
				const float force = springConstant * std::abs(personPosition.y);
				person->applyForceToCenterOfMass(rp3d::Vector3(0.f, 1.f, 0.f) * force);
				lowestPoint = std::min(lowestPoint, personPosition.y);
				if (!hadContact)
				{
					hadContact = true;
					std::printf("Contact Velocity: %f m/s \n", personVelocity.y);
				}
			}
		}

		std::printf("Spring was depressed by  %f m \n", std::abs(lowestPoint));
		
	}
}

void Phys_A3()
{
	//Phys_A3_1();
	//Phys_A3_2();
	Phys_A3_3();
	//Phys_A3_5();
}
