//Colin White
//C14509343
//Games engines assignment

#include "Assignment.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

using namespace BGE;

Assignment::Assignment(void)
{
}

Assignment::~Assignment(void)
{
}

shared_ptr<PhysicsController> cyl;
std::shared_ptr<GameComponent> station;

bool Assignment::Initialise()
{
	Game::dynamicsWorld->setGravity(btVector3(0, -20, 0));
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();
	float wheelPos = 7;
	float spin = 100;
	speed = 0.4;

	//body
	//this is a rectangular box
	shared_ptr<PhysicsController> body = physicsFactory->CreateBox(4, 1, 10, glm::vec3(0, 2.5, 0), glm::quat());

	//front legs
	//these are boxes conntected to the body with hinges
	shared_ptr<PhysicsController> leg1 = physicsFactory->CreateBox(1, 4, 1, glm::vec3(2, 4, 10), glm::quat());
	btHingeConstraint * hinge1 = new btHingeConstraint(*body->rigidBody, *leg1->rigidBody, btVector3(3, -1, 5), btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(1, 0, 0), true);
	shared_ptr<PhysicsController> leg2 = physicsFactory->CreateBox(1, 4, 1, glm::vec3(-2, 7, 1), glm::quat());
	btHingeConstraint * hinge2 = new btHingeConstraint(*body->rigidBody, *leg2->rigidBody, btVector3(-3, -1, 5), btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(1, 0, 0), true);

	//back wheels
	//these are cylinders connected to the body with hinges
	shared_ptr<PhysicsController> leg3 = physicsFactory->CreateCylinder(2, 2, glm::vec3(4, 2, -wheelPos), glm::angleAxis(90.0f, glm::vec3(0, 0, 1)));
	btHingeConstraint * hinge3 = new btHingeConstraint(*body->rigidBody, *leg3->rigidBody, btVector3(4, 1, -wheelPos), btVector3(1, 0, 0), btVector3(0, 0, 0), btVector3(0, 1, 0), true);

	shared_ptr<PhysicsController> leg4 = physicsFactory->CreateCylinder(2, 2, glm::vec3(-4, 2, -wheelPos), glm::angleAxis(90.0f, glm::vec3(0, 0, 1)));
	btHingeConstraint * hinge4 = new btHingeConstraint(*body->rigidBody, *leg4->rigidBody, btVector3(-4, 1, -wheelPos), btVector3(1, 0, 0), btVector3(0, 0, 0), btVector3(0, 1, 0), true);

	//head
	//this is a spehere connected to the body with a ball and socket joint
	shared_ptr<PhysicsController> head = physicsFactory->CreateSphere(1, glm::vec3(-2, 6, -4), glm::quat());
	btPoint2PointConstraint * bs = new btPoint2PointConstraint(*body->rigidBody, *head->rigidBody, btVector3(0, 5, 5), btVector3(0, 0, 0));

	//set speed
	hinge1->setLimit(0, speed);
	hinge2->setLimit(0, speed);
	hinge3->setLimit(0, speed);
	hinge4->setLimit(0, speed);

	//give power
	hinge1->enableAngularMotor(true, 100, 100);
	hinge2->enableAngularMotor(true, 100, 100);
	hinge3->enableAngularMotor(true, -.7, -.7);
	hinge4->enableAngularMotor(true, -.7, -.7);

	dynamicsWorld->addConstraint(hinge1);
	dynamicsWorld->addConstraint(hinge2);
	dynamicsWorld->addConstraint(hinge3);
	dynamicsWorld->addConstraint(hinge4);
	dynamicsWorld->addConstraint(bs);


	if (!Game::Initialise()) {
		return false;
	}

	camera->transform->position = glm::vec3(0, 20, 40);

	return true;
}

void BGE::Assignment::Update(float timeDelta)
{
	Game::Update(timeDelta);
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}
