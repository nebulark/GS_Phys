// Phys_A1.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

const rp3d::Vector3 gravity(0.0f, -9.81f, 0.0f);
const float timeStep = 0.001f;

void CarTrain(const float carSpeed, const float trainSpeed, const float simulationTime)
{
	rp3d::DynamicsWorld world(gravity);

	// Initial position and orientation of the rigid body 
	const rp3d::Vector3 carPos(0.0, 0.0, 0.0);

	const rp3d::Vector3 trainPos(1300.0, 0, 0.0);

	const rp3d::Transform carTrans(carPos, rp3d::Quaternion::identity());
	rp3d::RigidBody* car = world.createRigidBody(carTrans);
	car->enableGravity(false);
	car->setLinearVelocity(rp3d::Vector3(carSpeed, 0.f, 0.f));

	const rp3d::Transform trainTrans(trainPos, rp3d::Quaternion::identity());
	rp3d::RigidBody* train = world.createRigidBody(trainTrans);
	train->enableGravity(false);
	train->setLinearVelocity(rp3d::Vector3(trainSpeed, 0.f, 0.f));

	float timePassed = 0.f;
	while (timePassed <= simulationTime)
	{
		world.update(timeStep);
		timePassed += timeStep;
	}

	printf("\n===== Train - Car ====\nafter %f seconds \ncar Position: %f \ntrainPosition %f\n",
		timePassed,
		car->getWorldPoint(rp3d::Vector3::zero()).x,
		train->getWorldPoint(rp3d::Vector3::zero()).x);

}

void WaterNozzle(const float waterSpeed, const float simulationTime)
{
	rp3d::DynamicsWorld world(gravity);

	// Initial position and orientation of the rigid body 
	const rp3d::Vector3 waterPos(0.0, 1.8, 0.0);

	const rp3d::Transform waterTrans(waterPos, rp3d::Quaternion::identity());
	rp3d::RigidBody* water = world.createRigidBody(waterTrans);
	water->setLinearVelocity(rp3d::Vector3(0.f, waterSpeed, 0.f));

	float timePassed = 0.f;
	while (timePassed <= simulationTime)
	{
		world.update(timeStep);
		timePassed += timeStep;
	}

	printf("\n===== Water Nozzle ====\nafter %f seconds \nwater Position: %f\n",
		timePassed,
		water->getWorldPoint(rp3d::Vector3::zero()).y);

}

void PlaneDrop(const float horizontalDropDistance, const float simulationTime)
{
	const float planeSpeed = 69.4f;
	const rp3d::Vector3 planePos(-horizontalDropDistance, 235.f, 0.f);

	rp3d::DynamicsWorld world(gravity);

	// Initial position and orientation of the rigid body 

	const rp3d::Transform dropTrans(planePos, rp3d::Quaternion::identity());
	rp3d::RigidBody* drop = world.createRigidBody(dropTrans);
	drop->setLinearVelocity(rp3d::Vector3(planeSpeed, 0.f, 0.f));

	float timePassed = 0.f;
	while (timePassed <= simulationTime)
	{
		world.update(timeStep);
		timePassed += timeStep;
	}

	const rp3d::Vector3 dropEndloation = drop->getWorldPoint(rp3d::Vector3::zero());
	printf("\n=====Plane Drop ====\nafter %f seconds \ndrop Location relative to target: (%f,%f)\n",
		simulationTime,
		dropEndloation.x,
		dropEndloation.y);


}

int main()
{
	CarTrain(
		95.f * 1000.f / 3600.f,
		75.f * 1000.f / 3600.f,
		233.81f
	);

	CarTrain(
		95.f * 1000.f / 3600.f,
		-75.f * 1000.f / 3600.f,
		27.53f
	);

	WaterNozzle(11.54f, 2.5f);
	PlaneDrop(480.25f, 6.92f);


	return 0;
}

