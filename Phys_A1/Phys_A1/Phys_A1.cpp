// Phys_A1.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

const rp3d::Vector3 gravity(0.0f, -9.81f, 0.0f);

int main()
{
	rp3d::DynamicsWorld world(gravity);

	// Initial position and orientation of the rigid body 
	rp3d::Vector3 initPosition(0.0, 3.0, 0.0);
	rp3d::Transform transform(initPosition, rp3d::Quaternion::identity());
	rp3d::RigidBody* rb = world.createRigidBody(transform);


    return 0;
}

