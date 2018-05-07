#include "stdafx.h"
#include "Phys_A3.h"

namespace
{
	const rp3d::Vector3 earthGravity(0.0, -9.81, 0.0);

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
	void Phys_A2_2()
	{
		//std::puts(" === Phys A2 2) === ");

		//rp3d::DynamicsWorld world(earthGravity);
		//world.setNbIterationsVelocitySolver(15);

		//float inclineDegrees = 22.f;
		//float boxWeight = 7.f;
		//float frictionCoefficient = 0.f;

		//const rp3d::Quaternion slopeOrientation(rp3d::Vector3(0, 0, -DegToRad(inclineDegrees)));

		////Create Box Object
		//rp3d::BoxShape boxBoxShape(boxHalfExtents);
		//rp3d::Transform boxTransform(boxStartPosition, slopeOrientation);
		//rp3d::RigidBody* box;
		//box = world.createRigidBody(boxTransform);
		//box->setType(reactphysics3d::BodyType::DYNAMIC);
		//box->addCollisionShape(&boxBoxShape, rp3d::Transform::identity(), boxWeight);

		////Create Slope Object
		//rp3d::BoxShape slopeBoxShape(slopeHalfExtents);
		//rp3d::Transform slopeTransform(slopePosition, slopeOrientation);
		//rp3d::RigidBody* slope;
		//slope = world.createRigidBody(slopeTransform);
		//slope->setType(reactphysics3d::BodyType::STATIC);
		//slope->addCollisionShape(&slopeBoxShape, rp3d::Transform::identity(), 1);

		////Set Material of Slope
		//rp3d::Material& slopeMaterial = slope->getMaterial();
		//slopeMaterial.setBounciness(rp3d::decimal(0));
		//slopeMaterial.setFrictionCoefficient(rp3d::decimal(frictionCoefficient));

		////Set Material of Box
		//rp3d::Material& boxMaterial = box->getMaterial();
		//boxMaterial.setBounciness(rp3d::decimal(0));
		//boxMaterial.setFrictionCoefficient(rp3d::decimal(frictionCoefficient));

		//float lastSpeed = 0;

		//constexpr float tickRate = 0.001f;
		//constexpr float simulationSeconds = 5.f;
		//constexpr int ticks = simulationSeconds / tickRate + 1;
		//float secondsPassed = 0.f;
		//const float measuredTimeSpan = 1.0f;

		////Main Loop
		//while (true) {
		//	world.update(tickRate);
		//	const rp3d::Vector3 boxPosition = box->getTransform().getPosition();
		//	const rp3d::Vector3 boxVelocity = box->getLinearVelocity();
		//	const float boxSpeed = boxVelocity.length();

		//	secondsPassed += tickRate;

		//	if (secondsPassed > measuredTimeSpan) {
		//		float currentSpeed = box->getLinearVelocity().length();
		//		float deltaSpeed = currentSpeed - lastSpeed;
		//		lastSpeed = currentSpeed;
		//		secondsPassed -= measuredTimeSpan;

		//		float boxAcceleration = deltaSpeed / measuredTimeSpan;
		//		std::printf("Acceleration: %f\n", boxAcceleration);
		//	}

		//	if ((boxPosition - boxStartPosition).length() > 12.f)
		//	{
		//		std::printf("Box Speed After traveling 12m downwards : %f\n", boxSpeed);
		//		break;
		//	}
		//}
	}

	void Phys_A2_3()
	{
		//std::puts(" === Phys A2 3) === ");

		//rp3d::DynamicsWorld world(earthGravity);
		//world.setNbIterationsVelocitySolver(15);

		//float inclineDegrees = 22.f;
		//float boxWeight = 7.f;
		//float frictionCoefficient = 0.f;
		//const rp3d::Vector3 localBoxVelocity(-4.5f, 0, 0);

		//rp3d::Quaternion slopeOrientation(rp3d::Vector3(0, 0, -DegToRad(inclineDegrees)));

		//const rp3d::Vector3 worldBoxVelocity = slopeOrientation * localBoxVelocity;

		////Create Box Object
		//rp3d::BoxShape boxBoxShape(boxHalfExtents);
		//rp3d::Transform boxTransform(boxStartPosition, slopeOrientation);
		//rp3d::RigidBody* box;
		//box = world.createRigidBody(boxTransform);
		//box->setType(reactphysics3d::BodyType::DYNAMIC);
		//box->addCollisionShape(&boxBoxShape, rp3d::Transform::identity(), boxWeight);
		//box->setLinearVelocity(worldBoxVelocity);

		////Create Slope Object
		//rp3d::BoxShape slopeBoxShape(slopeHalfExtents);
		//rp3d::Transform slopeTransform(slopePosition, slopeOrientation);
		//rp3d::RigidBody* slope;
		//slope = world.createRigidBody(slopeTransform);
		//slope->setType(reactphysics3d::BodyType::STATIC);
		//slope->addCollisionShape(&slopeBoxShape, rp3d::Transform::identity(), 1);

		////Set Material of Slope
		//rp3d::Material& slopeMaterial = slope->getMaterial();
		//slopeMaterial.setBounciness(rp3d::decimal(0));
		//slopeMaterial.setFrictionCoefficient(rp3d::decimal(frictionCoefficient));

		////Set Material of Box
		//rp3d::Material& boxMaterial = box->getMaterial();
		//boxMaterial.setBounciness(rp3d::decimal(0));
		//boxMaterial.setFrictionCoefficient(rp3d::decimal(frictionCoefficient));

		//float lastSpeed = worldBoxVelocity.length();

		//constexpr float tickRate = 0.001f;
		//constexpr float simulationSeconds = 5.f;
		//constexpr int ticks = simulationSeconds / tickRate + 1;
		//const float measuredTimeSpan = 0.1f;
		//float secondsPassedSinceMeasurement = 0.f;
		//rp3d::Vector3 boxHighestPoint;

		//int totalTicks = 0;
		//while (true) {

		//	world.update(tickRate);
		//	const rp3d::Vector3 boxPosition = box->getTransform().getPosition();
		//	const rp3d::Vector3 boxVelocity = box->getLinearVelocity();
		//	const float boxSpeed = boxVelocity.length();

		//	if (boxPosition.y > boxHighestPoint.y)
		//	{
		//		boxHighestPoint = boxPosition;
		//	}
		//	++totalTicks;
		//	secondsPassedSinceMeasurement += tickRate;

		//	if (std::abs(boxPosition.x - boxStartPosition.x) < 0.01 && boxVelocity.y < 0)
		//	{
		//		const float upwardsDisanceTravelled = (boxStartPosition - boxHighestPoint).length();

		//		std::printf("Box went %f meters upward\nBox reached initial location after %f seconds\n", upwardsDisanceTravelled, totalTicks * tickRate);
		//		break;
		//	}

		//	/*if (secondsPassedSinceMeasurement > measuredTimeSpan) {
		//	float currentSpeed = box->getLinearVelocity().length();
		//	float deltaSpeed = currentSpeed - lastSpeed;
		//	lastSpeed = currentSpeed;
		//	secondsPassedSinceMeasurement -= measuredTimeSpan;

		//	float boxAcceleration = deltaSpeed / measuredTimeSpan;
		//	std::printf("Acceleration: %f\n", boxAcceleration);
		//	}*/
		//}
	}

	void Phys_A2_4()
	{
		//std::puts(" === Phys A2 4) === ");

		//rp3d::DynamicsWorld world(earthGravity);
		//world.setNbIterationsVelocitySolver(15);

		//const float inclineDegrees = 25.f;
		//const float boxWeight = 7.f;
		//const float frictionCoefficient = 0.19f;
		//const float downwardsDistance = 8.15;

		//const rp3d::Quaternion slopeOrientation(rp3d::Vector3(0, 0, -DegToRad(inclineDegrees)));

		////Create Box Object
		//rp3d::BoxShape boxBoxShape(boxHalfExtents);
		//rp3d::Transform boxTransform(boxStartPosition, slopeOrientation);
		//rp3d::RigidBody* box;
		//box = world.createRigidBody(boxTransform);
		//box->setType(reactphysics3d::BodyType::DYNAMIC);
		//box->addCollisionShape(&boxBoxShape, rp3d::Transform::identity(), boxWeight);

		////Create Slope Object
		//rp3d::BoxShape slopeBoxShape(slopeHalfExtents);
		//rp3d::Transform slopeTransform(slopePosition, slopeOrientation);
		//rp3d::RigidBody* slope;
		//slope = world.createRigidBody(slopeTransform);
		//slope->setType(reactphysics3d::BodyType::STATIC);
		//slope->addCollisionShape(&slopeBoxShape, rp3d::Transform::identity(), 1);

		////Set Material of Slope
		//rp3d::Material& slopeMaterial = slope->getMaterial();
		//slopeMaterial.setBounciness(rp3d::decimal(0));
		//slopeMaterial.setFrictionCoefficient(rp3d::decimal(frictionCoefficient));

		////Set Material of Box
		//rp3d::Material& boxMaterial = box->getMaterial();
		//boxMaterial.setBounciness(rp3d::decimal(0));
		//boxMaterial.setFrictionCoefficient(rp3d::decimal(frictionCoefficient));

		//float lastSpeed = 0;

		//constexpr float tickRate = 0.001f;
		//constexpr float simulationSeconds = 5.f;
		//constexpr int ticks = simulationSeconds / tickRate + 1;
		//float secondsPassed = 0.f;
		//const float measuredTimeSpan = 1.f;

		////Main Loop
		//while (true) {
		//	world.update(tickRate);
		//	const rp3d::Vector3 boxPosition = box->getTransform().getPosition();
		//	const rp3d::Vector3 boxVelocity = box->getLinearVelocity();
		//	const float boxSpeed = boxVelocity.length();

		//	secondsPassed += tickRate;

		//	if (secondsPassed > measuredTimeSpan) {
		//		float currentSpeed = box->getLinearVelocity().length();
		//		float deltaSpeed = currentSpeed - lastSpeed;
		//		lastSpeed = currentSpeed;
		//		secondsPassed -= measuredTimeSpan;

		//		float boxAcceleration = deltaSpeed / measuredTimeSpan;
		//		std::printf("Acceleration: %f\n", boxAcceleration);
		//	}

		//	if ((boxPosition - boxStartPosition).length() > downwardsDistance)
		//	{
		//		std::printf("Box Speed After traveling 8.5 m downwards : %f\n", boxSpeed);
		//		break;
		//	}


		//}
	}

	void Phys_A2_5()
	{

		//	std::puts(" === Phys A2 5) === ");

		//	rp3d::DynamicsWorld world(earthGravity);
		//	world.setNbIterationsVelocitySolver(15);

		//	float inclineDegrees = 25.f;
		//	float boxWeight = 7.f;
		//	float frictionCoefficient = 0.12f;
		//	const rp3d::Vector3 localBoxVelocity(-3.0f, 0, 0);

		//	rp3d::Quaternion slopeOrientation(rp3d::Vector3(0, 0, -DegToRad(inclineDegrees)));

		//	const rp3d::Vector3 worldBoxVelocity = slopeOrientation * localBoxVelocity;

		//	//Create Box Object
		//	rp3d::BoxShape boxBoxShape(boxHalfExtents);
		//	rp3d::Transform boxTransform(boxStartPosition, slopeOrientation);
		//	rp3d::RigidBody* box;
		//	box = world.createRigidBody(boxTransform);
		//	box->setType(reactphysics3d::BodyType::DYNAMIC);
		//	box->addCollisionShape(&boxBoxShape, rp3d::Transform::identity(), boxWeight);
		//	box->setLinearVelocity(worldBoxVelocity);

		//	//Create Slope Object
		//	rp3d::BoxShape slopeBoxShape(slopeHalfExtents);
		//	rp3d::Transform slopeTransform(slopePosition, slopeOrientation);
		//	rp3d::RigidBody* slope;
		//	slope = world.createRigidBody(slopeTransform);
		//	slope->setType(reactphysics3d::BodyType::STATIC);
		//	slope->addCollisionShape(&slopeBoxShape, rp3d::Transform::identity(), 1);

		//	//Set Material of Slope
		//	rp3d::Material& slopeMaterial = slope->getMaterial();
		//	slopeMaterial.setBounciness(rp3d::decimal(0));
		//	slopeMaterial.setFrictionCoefficient(rp3d::decimal(frictionCoefficient));

		//	//Set Material of Box
		//	rp3d::Material& boxMaterial = box->getMaterial();
		//	boxMaterial.setBounciness(rp3d::decimal(0));
		//	boxMaterial.setFrictionCoefficient(rp3d::decimal(frictionCoefficient));

		//	float lastSpeed = worldBoxVelocity.length();

		//	constexpr float tickRate = 0.001f;
		//	constexpr float simulationSeconds = 5.f;
		//	constexpr int ticks = simulationSeconds / tickRate + 1;
		//	float secondsPassed = 0.f;
		//	float secondsPassedSinceMeasurement = 0.f;
		//	const float measuredTimeSpan = 1.f;

		//	rp3d::Vector3 boxHighestPoint;

		//	while (true) {

		//		world.update(tickRate);
		//		const rp3d::Vector3 boxPosition = box->getTransform().getPosition();
		//		const rp3d::Vector3 boxVelocity = box->getLinearVelocity();
		//		const float boxSpeed = boxVelocity.length();

		//		if (boxPosition.y > boxHighestPoint.y)
		//		{
		//			boxHighestPoint = boxPosition;
		//		}

		//		//std::printf("%f;%f;%f;\n", boxPosition.x, boxPosition.y, boxPosition.z);

		//		secondsPassed += tickRate;
		//		secondsPassedSinceMeasurement += tickRate;

		//		if ((boxPosition.x > boxStartPosition.x) && boxVelocity.y < 0)
		//		{
		//			const float upwardsDisanceTravelled = (boxStartPosition - boxHighestPoint).length();

		//			std::printf("Box went %f meters upward\nBox reached initial location after %f seconds\n", upwardsDisanceTravelled, secondsPassed);
		//			break;
		//		}

		//		/*if (secondsPassedSinceMeasurement > measuredTimeSpan) {
		//		float currentSpeed = box->getLinearVelocity().length();
		//		float deltaSpeed = currentSpeed - lastSpeed;
		//		lastSpeed = currentSpeed;
		//		secondsPassedSinceMeasurement -= measuredTimeSpan;

		//		float boxAcceleration = deltaSpeed / measuredTimeSpan;
		//		std::printf("Acceleration: %f\n", boxAcceleration);
		//		}*/
		//	}
		//}
	}

}

void Phys_A3()
{
	Phys_A3_1();
}
