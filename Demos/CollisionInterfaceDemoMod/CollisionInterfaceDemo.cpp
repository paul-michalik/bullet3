/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


///
/// CollisionInterfaceDemo shows high level usage of the Collision Detection.
///
#define TEST_NOT_ADDING_OBJECTS_TO_WORLD

#include "GL_Simplex1to4.h"

//include common Bullet Collision Detection headerfiles
#include "btBulletCollisionCommon.h"

#include "LinearMath/btIDebugDraw.h"
#include "GL_ShapeDrawer.h"
#include "CollisionInterfaceDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"

#include <iostream>

btScalar yaw = 0.f, pitch = 0.f, roll = 0.f;
const int maxNumObjects = 4;
const int numObjects = 2;

GL_Simplex1to4 simplex;

btCollisionObject	objects[maxNumObjects];
btCollisionWorld*	collisionWorld = 0;

GLDebugDrawer debugDrawer;

#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

#include <array>

namespace test {
    namespace data {
        btVector3 c_shape_0_data[3] = {
            {-4, 0, 0},
            {+4, 0, 0},
            {0, 4, 0}
        };

        btVector3 c_shape_1_data[3] = {
            {-2, 0, 0},
            {+2, 0, 0},
            {0, 2, 0}
        };

        const auto c_pyramid_factor = 5;
        //btTriangleShape c_shape_0(c_shape_0_data[0], c_shape_0_data[1], c_shape_0_data[2]);
        //btTriangleShape c_shape_1(c_shape_1_data[0], c_shape_1_data[1], c_shape_1_data[2]);
        std::array<int, 4 * 3> c_open_pyramid_indices = {
            0, 1, 4,
            1, 2, 4,
            2, 3, 4,
            0, 4, 3
        };
        std::array<btScalar, 5 * 3> c_open_pyramid_coors = {
            -c_pyramid_factor, 0, +c_pyramid_factor,
            +c_pyramid_factor, 0, +c_pyramid_factor,
            +c_pyramid_factor, 0, -c_pyramid_factor,
            -c_pyramid_factor, 0, -c_pyramid_factor,
            0, +c_pyramid_factor, 0
        };

        btTriangleIndexVertexArray c_triangles_open_pyramid(
            c_open_pyramid_indices.size() / 3u, &c_open_pyramid_indices[0], 3 * sizeof(int),
            c_open_pyramid_coors.size() / 3u, &c_open_pyramid_coors[0], 3 * sizeof(btScalar));

        //btTriangleMesh c_triangles_open_pyramid;
        //triangle_mesh_initializer c_initializer(c_triangles_open_pyramid);
        btGImpactMeshShape c_shape_0(&c_triangles_open_pyramid);
        //btSphereShape c_shape_0(5);
        //btGImpactMeshShape c_shape_1(&c_triangles_open_pyramid);
        //btBoxShape c_shape_1(btVector3(0.5, 0.5, 0.5));
        btBoxShape c_shape_1(btVector3(c_pyramid_factor, c_pyramid_factor, c_pyramid_factor));


        double margin = 0.01;

        template<class T>
        void init_shape(T& p_shape)
        {

        }

        void init_shape(btGImpactMeshShape& p_mesh)
        {
            p_mesh.updateBound();
        }
    }
}

void	CollisionInterfaceDemo::initPhysics()
{

    m_debugMode |= btIDebugDraw::DBG_DrawWireframe;

    btMatrix3x3 basisA;
    basisA.setIdentity();

    btMatrix3x3 basisB;
    basisB.setIdentity();

    objects[0].getWorldTransform().setBasis(basisA);
    objects[1].getWorldTransform().setBasis(basisB);

    //btBoxShape* boxA = new btBoxShape(btVector3(5., 5., 5.));
    //boxA->setMargin(0.f);

    //btBoxShape* boxB = new btBoxShape(btVector3(1., 1., 1.));
    //boxB->setMargin(0.f);
    //ConvexHullShape	hullA(points0,3);
    //hullA.setLocalScaling(btVector3(3,3,3));
    //ConvexHullShape	hullB(points1,4);
    //hullB.setLocalScaling(btVector3(4,4,4));

    test::data::c_shape_0.setMargin(test::data::margin);
    test::data::init_shape(test::data::c_shape_0);

    test::data::c_shape_1.setMargin(test::data::margin);
    test::data::init_shape(test::data::c_shape_1);

    objects[0].setCollisionShape(std::addressof(test::data::c_shape_0));//&hullA;
    objects[1].setCollisionShape(std::addressof(test::data::c_shape_1));//&hullB;

    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btVector3	worldAabbMin(-1000, -1000, -1000);
    btVector3	worldAabbMax(1000, 1000, 1000);

    //auto	broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax);
    auto broadphase = new btDbvtBroadphase;
    //SimpleBroadphase is a brute force alternative, performing N^2 aabb overlap tests
    //auto	broadphase = new btSimpleBroadphase;

    collisionWorld = new btCollisionWorld(dispatcher, broadphase, collisionConfiguration);
    collisionWorld->setDebugDrawer(&debugDrawer);

#ifdef TEST_NOT_ADDING_OBJECTS_TO_WORLD
    collisionWorld->addCollisionObject(&objects[0]);
    collisionWorld->addCollisionObject(&objects[1]);
#endif //TEST_NOT_ADDING_OBJECTS_TO_WORLD

}


//to be implemented by the demo

void CollisionInterfaceDemo::clientMoveAndDisplay()
{

    displayCallback();
}

struct btDrawingResult : public btCollisionWorld::ContactResultCallback
{
    virtual	btScalar	addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper*, int, int, const btCollisionObjectWrapper*, int, int)
    {

        glBegin(GL_LINES);
        glColor3f(0, 0, 0);

        btVector3 ptA = cp.getPositionWorldOnA();
        btVector3 ptB = cp.getPositionWorldOnB();

        std::cout
            << "A: " << ptA.getX() << ", " << ptA.getY() << ", " << ptA.getZ() << std::endl
            << "B: " << ptB.getX() << ", " << ptB.getY() << ", " << ptB.getZ() << std::endl
            << "distance = " << cp.getDistance() << std::endl;

        glVertex3d(ptA.x(), ptA.y(), ptA.z());
        glVertex3d(ptB.x(), ptB.y(), ptB.z());
        glEnd();

        return 0;
    }
};

#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "GLDebugDrawer.h"

void draw_and_dump(char const* p_caller, btPointCollector const& p_result, bool p_dump = false)
{
    if (p_dump) {
        std::cout
            << p_caller << std::endl
            << "has result = " << p_result.m_hasResult << std::endl;
    }

    if (p_result.m_hasResult) {

        btVector3 endPt =
            p_result.m_pointInWorld + p_result.m_normalOnBInWorld * p_result.m_distance;

        if (p_dump) {
            std::cout
                << p_result.m_pointInWorld.getX() << ", "
                << p_result.m_pointInWorld.getY() << ", "
                << p_result.m_pointInWorld.getZ() << std::endl
                << endPt.getX() << ", "
                << endPt.getY() << ", "
                << endPt.getZ() << std::endl
                << "Minimal distance: " << p_result.m_distance << std::endl;
        }


        debugDrawer.drawLine(p_result.m_pointInWorld, endPt, btVector3(255, 0, 0));
        debugDrawer.drawSphere(p_result.m_pointInWorld, 0.05, btVector3(255, 0, 0));
        debugDrawer.drawSphere(endPt, 0.05, btVector3(255, 0, 0));
    }
}

void draw_and_dump(char const* p_caller, btVector3 const& ptA, btVector3 const& ptB, bool p_dump = false)
{
    if (p_dump) {
        std::cout
            << p_caller << std::endl
            << ptA.getX() << ", " << ptA.getY() << ", " << ptA.getZ() << std::endl
            << ptB.getX() << ", " << ptB.getY() << ", " << ptB.getZ() << std::endl;
    }
    debugDrawer.drawLine(ptA, ptB, btVector3(0, 0, 0));
}

void draw_and_dump(char const* p_caller, btManifoldPoint const& cp, bool p_dump = false)
{
    btVector3 ptA = cp.getPositionWorldOnA();
    btVector3 ptB = cp.getPositionWorldOnB();

    if (p_dump) {
        std::cout
            << p_caller << std::endl
            << "A: " << ptA.getX() << ", " << ptA.getY() << ", " << ptA.getZ() << std::endl
            << "B: " << ptB.getX() << ", " << ptB.getY() << ", " << ptB.getZ() << std::endl
            << "distance = " << cp.getDistance() << std::endl;
    }

    debugDrawer.drawLine(ptA, ptB, btVector3(0, 0, 255));
}

void draw_and_dump(char const* p_caller, btCollisionWorld* p_collision_world, bool p_dump = false)
{
    // draw results:
    btDrawingResult renderCallback;
    auto t_dispatcher = p_collision_world->getDispatcher();

    if (p_dump) {
        std::cout
            << "Number of manifolds: " << t_dispatcher->getNumManifolds() << std::endl;
    }

    for (auto t_manifold_id = 0, t_manifold_num = t_dispatcher->getNumManifolds();
        t_manifold_id < t_manifold_num;
        ++t_manifold_id)
    {

        auto t_manifold = t_dispatcher->getManifoldByIndexInternal(t_manifold_id);

        if (p_dump) {
            std::cout
                << "Number of contacts: " << t_manifold->getNumContacts() << std::endl;
        }

        for (auto t_contact_id = 0, t_contact_num = t_manifold->getNumContacts();
            t_contact_id < t_contact_num;
            ++t_contact_id)
        {
            draw_and_dump(__FUNCTION__, t_manifold->getContactPoint(t_contact_id), p_dump);
        }
    }
}

btPointCollector g_output;

void calculate_closest_points(const btTransform& transA, const btTransform& transB, bool p_dump = false)
{
    // this is basically a clone from btContinuousConvexCollision::computeClosestPoints:
    //static btVoronoiSimplexSolver sGjkSimplexSolver;
    //static btGjkEpaPenetrationDepthSolver sEpaSolver;


    //btSimplexSolverInterface *m_simplexSolver = &sGjkSimplexSolver;
    //btConvexPenetrationDepthSolver *m_penetrationDepthSolver = &sEpaSolver;
    //btConvexShape *m_convexA = &test::data::c_shape_0;
    //btConvexShape *m_convexB1 = &test::data::c_shape_1;


    //m_simplexSolver->reset();
    //btGjkPairDetector gjk(
    //    m_convexA, m_convexB1,
    //    m_convexA->getShapeType(), m_convexB1->getShapeType(),
    //    m_convexA->getMargin(), m_convexB1->getMargin(),
    //    m_simplexSolver, m_penetrationDepthSolver);

    //btGjkPairDetector::ClosestPointInput input;
    //input.m_transformA = transA;
    //input.m_transformB = transB;
    //// reset!
    //g_output = btPointCollector();
    //gjk.getClosestPoints(input, g_output, nullptr);
}

void perform_contact_detection(btCollisionWorld* collisionWorld)
{
    collisionWorld->performDiscreteCollisionDetection();
}

double get_random_from(double p_range_min, double p_range_max)
{
   return (double)rand() / (RAND_MAX + 1) * (p_range_max - p_range_min)
        + p_range_min;
}

void CollisionInterfaceDemo::displayCallback(void)
{
    static const auto c_max_counter = 10;
    static auto s_counter = 0;
    static auto s_sign = 1;
    static auto s_prev_time_in_seconds = m_clock.getTimeSeconds();

    btScalar t_curr_time_in_seconds = m_clock.getTimeSeconds();

    bool t_do_change = t_curr_time_in_seconds - s_prev_time_in_seconds > 1 && !m_idle;

    if (t_do_change) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDisable(GL_LIGHTING);

        auto t_origin0 = objects[0]
            .getWorldTransform()
            .getOrigin();

        auto t_origin1 = objects[1]
            .getWorldTransform()
            .getOrigin();

        auto t_rotation_angle1 = objects[1]
            .getWorldTransform()
            .getRotation()
            .getAngle();

        if (s_counter++ < c_max_counter) {
            auto t_new_transform =
                objects[1].getWorldTransform();

            t_new_transform.setRotation(
                btQuaternion(((btScalar)s_counter / (btScalar)c_max_counter) * SIMD_2_PI, 0., 0.));
            //t_new_transform.setOrigin(t_origin1 + btVector3(-s_sign, 0., 0.));
            //t_new_transform.setOrigin(btVector3(get_random_from(-1, +1), get_random_from(-1, +1), get_random_from(-1, +1)));

            objects[1]
                .setWorldTransform(t_new_transform);
        } else {
            s_counter = 0;
            s_sign *= -1;
        }

        perform_contact_detection(collisionWorld);
        calculate_closest_points(
            objects[0].getWorldTransform(),
            objects[1].getWorldTransform());


        for (auto i = 0; i < numObjects; i++)
        {
            collisionWorld->debugDrawObject(objects[i].getWorldTransform(), objects[i].getCollisionShape(), btVector3(1, 1, 0));
        }

        draw_and_dump(__FUNCTION__, collisionWorld, t_do_change);
        draw_and_dump(__FUNCTION__, g_output, t_do_change);

        s_prev_time_in_seconds = t_curr_time_in_seconds;

        glFlush();
        swapBuffers();
    }
}

void CollisionInterfaceDemo::clientResetScene()
{
    objects[0].getWorldTransform().setOrigin(btVector3(0.f, 0.f, 0.f));

    //btQuaternion rotA(0.739f,-0.204f,0.587f,0.257f);
    //rotA.normalize();

    //objects[0].getWorldTransform().setRotation(rotA);

    //objects[1].getWorldTransform().setOrigin(btVector3(0.0f,4.248f,0.f));

    objects[1].getWorldTransform().setOrigin(btVector3(0., 0., 0.));
}


