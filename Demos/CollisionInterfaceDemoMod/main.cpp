
#include "CollisionInterfaceDemo.h"
#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"

int screenWidth = 640;
int screenHeight = 480;


int main(int argc,char** argv)
{
	CollisionInterfaceDemo* collisionInterfaceDemo = new CollisionInterfaceDemo();

	collisionInterfaceDemo->initPhysics();

	collisionInterfaceDemo->clientResetScene();

 collisionInterfaceDemo->setCameraForwardAxis(2);
 collisionInterfaceDemo->setCameraDistance(-10.f);

 collisionInterfaceDemo->setOrthographicProjection();
 collisionInterfaceDemo->updateCamera();
	return glutmain(argc, argv,screenWidth,screenHeight,"Collision Interface Demo",collisionInterfaceDemo);
}
