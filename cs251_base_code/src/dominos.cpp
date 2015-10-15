/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/*
 * Base code for CS 251 Software Systems Lab
 * Department of Computer Science and Engineering, IIT Bombay
 *
 */


#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs251
{
/**  The is the constructor
 * This is the documentation block for the constructor.
 */

dominos_t::dominos_t()
{
    //Ground
    /*! \var b1
     * \brief pointer to the body ground
     */

    b2Body* b1;
    {

        b2EdgeShape shape;
        shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
        b1->CreateFixture(&shape, 0.0f);
    }

    //Top horizontal shelf
    {
        b2PolygonShape shape;
        shape.SetAsBox(6.0f, 0.25f);

        b2BodyDef bd;
        bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }

    //Dominos
    {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.0f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 20.0f;
        fd.friction = 0.1f;

        for (int i = 0; i < 10; ++i)
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-35.5f + 1.0f * i, 31.25f);
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd);
        }
    }

    //Another horizontal shelf
    {
        b2PolygonShape shape;
        shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);

        b2BodyDef bd;
        bd.position.Set(1.0f, 6.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }


    //The pendulum that knocks the dominos off
    {
        b2Body* b2;
        {
            b2PolygonShape shape;
            shape.SetAsBox(0.25f, 1.5f);

            b2BodyDef bd;
            bd.position.Set(-36.5f, 28.0f);
            b2 = m_world->CreateBody(&bd);
            b2->CreateFixture(&shape, 10.0f);
        }

        b2Body* b4;
        {
            b2PolygonShape shape;
            shape.SetAsBox(0.25f, 0.25f);

            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-40.0f, 33.0f);
            b4 = m_world->CreateBody(&bd);
            b4->CreateFixture(&shape, 2.0f);
        }

        b2RevoluteJointDef jd;
        b2Vec2 anchor;
        anchor.Set(-37.0f, 40.0f);
        jd.Initialize(b2, b4, anchor);
        m_world->CreateJoint(&jd);
    }

    //The train of small spheres
    {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 0.5;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 1.0f;
        ballfd.friction = 0.0f;
        ballfd.restitution = 0.0f;

        for (int i = 0; i < 0; ++i)
        {
            b2BodyDef ballbd;
            ballbd.type = b2_dynamicBody;
            ballbd.position.Set(-22.2f + i*1.0, 26.6f);
            spherebody = m_world->CreateBody(&ballbd);
            spherebody->CreateFixture(&ballfd);
        }
    }

    //The pulley system
    {
        b2BodyDef *bd = new b2BodyDef;
        bd->type = b2_dynamicBody;
        bd->position.Set(-10,15);
        bd->fixedRotation = true;

        //The open box
        b2FixtureDef *fd1 = new b2FixtureDef;
        fd1->density = 10.0;
        fd1->friction = 0.5;
        fd1->restitution = 0.f;
        fd1->shape = new b2PolygonShape;
        b2PolygonShape bs1;
        bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
        fd1->shape = &bs1;
        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 10.0;
        fd2->friction = 0.5;
        fd2->restitution = 0.f;
        fd2->shape = new b2PolygonShape;
        b2PolygonShape bs2;
        bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
        fd2->shape = &bs2;
        b2FixtureDef *fd3 = new b2FixtureDef;
        fd3->density = 10.0;
        fd3->friction = 0.5;
        fd3->restitution = 0.f;
        fd3->shape = new b2PolygonShape;
        b2PolygonShape bs3;
        bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
        fd3->shape = &bs3;

        b2Body* box1 = m_world->CreateBody(bd);
        box1->CreateFixture(fd1);
        box1->CreateFixture(fd2);
        box1->CreateFixture(fd3);

        //The bar
        bd->position.Set(10,15);
        fd1->density = 34.0;
        b2Body* box2 = m_world->CreateBody(bd);
        box2->CreateFixture(fd1);

        // The pulley joint
        b2PulleyJointDef* myjoint = new b2PulleyJointDef();
        b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
        b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
        b2Vec2 worldAnchorGround1(-10, 20); // Anchor point for ground 1 in world axis
        b2Vec2 worldAnchorGround2(10, 20); // Anchor point for ground 2 in world axis
        float32 ratio = 1.0f; // Define ratio
        myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
        m_world->CreateJoint(myjoint);
    }

//the rotating wings with cover
    {
        {
            b2PolygonShape funnelsr;
            funnelsr.SetAsBox(0.0f, 20.0f);
            b2BodyDef funnelbdr;
            funnelbdr.position.Set(23.0f,20.0f);
            funnelbdr.type= b2_staticBody;

            b2Body* funnelbr = m_world->CreateBody(&funnelbdr);
            b2FixtureDef *funnelfr = new b2FixtureDef;
            funnelfr->density = 40.0f;
            funnelfr->shape = new b2PolygonShape;
            funnelfr->shape = &funnelsr;
            funnelfr->filter.groupIndex=-1;
            funnelbr->CreateFixture(funnelfr);
        }
        {
            b2PolygonShape funnelsr;
            funnelsr.SetAsBox(0.0f, 20.0f);
            b2BodyDef funnelbdr;
            funnelbdr.position.Set(33.0f,20.0f);
            funnelbdr.type= b2_staticBody;

            b2Body* funnelbr = m_world->CreateBody(&funnelbdr);
            b2FixtureDef *funnelfr = new b2FixtureDef;
            funnelfr->density = 40.0f;
            funnelfr->shape = new b2PolygonShape;
            funnelfr->shape = &funnelsr;
            funnelfr->filter.groupIndex=-1;
            funnelbr->CreateFixture(funnelfr);
        }
        {
            b2PolygonShape funnelsr;
            funnelsr.SetAsBox(8.0f, 0.0f);
            b2BodyDef funnelbdr;
            funnelbdr.position.Set(28.0f,3.0f);
            funnelbdr.type= b2_staticBody;

            b2Body* funnelbr = m_world->CreateBody(&funnelbdr);
            b2FixtureDef *funnelfr = new b2FixtureDef;
            funnelfr->density = 40.0f;
            funnelfr->shape = new b2PolygonShape;
            funnelfr->shape = &funnelsr;
            funnelfr->filter.groupIndex=-1;
            funnelbr->CreateFixture(funnelfr);
        }




        {
            //group index -1 here,the wont collide with themslevs
            //change groupindex of other objects
            //this is the new rotating wind mill


            b2PolygonShape shape,shape3;
            shape.SetAsBox(8.0f, 0.5f);
            shape3.SetAsBox(0.5f,8.0f);
            for(int i=0; i<5; i++)
            {
                b2BodyDef bd;
                bd.position.Set(28.0f, 30.0f-5.0f*i);
                if(i%2==0)
                {
                    bd.angle=3.14/2-0.614;
                }
                else
                {
                    bd.angle=3.14/2+0.614;
                }
                bd.type=b2_dynamicBody;
                b2Body* body = m_world->CreateBody(&bd);
                b2FixtureDef *fd = new b2FixtureDef;
                fd->density = 1.f;
                fd->filter.groupIndex = -1;
                fd->shape = new b2PolygonShape;
                fd->shape = &shape;
                b2FixtureDef *fd1 = new b2FixtureDef;
                fd1->density = 1.f;
                fd1->filter.groupIndex = -1;
                fd1->shape = new b2PolygonShape;
                fd1->shape = &shape3;
                body->CreateFixture(fd);
                body->CreateFixture(fd1);
                body->SetAngularVelocity(5);



                //useless body
                b2PolygonShape shape2;
                shape2.SetAsBox(0.5f, 3.0f);
                b2BodyDef bd2;
                bd2.position.Set(28.0f, 30.0f-5.0f*i);
                b2Body* body2 = m_world->CreateBody(&bd2);

                //joint is defined here
                b2RevoluteJointDef jointDef;
                jointDef.bodyA = body;
                jointDef.bodyB = body2;
                jointDef.localAnchorA.Set(0,0);
                jointDef.localAnchorB.Set(0,0);
                jointDef.collideConnected = false;
                m_world->CreateJoint(&jointDef);
            }


        }




    }


    {
        //First Tilted Line
        b2PolygonShape funnelsr;
        funnelsr.SetAsBox(0.0f, 2.0f);
        b2BodyDef funnelbdr;
        funnelbdr.angle = -65;
        funnelbdr.position.Set(10.0f,40.0f);
        funnelbdr.type= b2_staticBody;

        b2Body* funnelbr = m_world->CreateBody(&funnelbdr);
        b2FixtureDef *funnelfr = new b2FixtureDef;
        funnelfr->density = 40.0f;
        funnelfr->shape = new b2PolygonShape;
        funnelfr->shape = &funnelsr;
        funnelbr->CreateFixture(funnelfr);

    }
    {
        // First Vertical Line
        b2PolygonShape funnelsr;
        funnelsr.SetAsBox(0.0f, 2.0f);
        b2BodyDef funnelbdr;
        funnelbdr.position.Set(11.5f,36.9f);
        funnelbdr.type= b2_staticBody;

        b2Body* funnelbr = m_world->CreateBody(&funnelbdr);
        b2FixtureDef *funnelfr = new b2FixtureDef;
        funnelfr->density = 40.0f;
        funnelfr->shape = new b2PolygonShape;
        funnelfr->shape = &funnelsr;
        funnelbr->CreateFixture(funnelfr);

    }
    {
        //Second Tilted Line
        b2PolygonShape funnelsr;
        funnelsr.SetAsBox(0.0f, 2.0f);
        b2BodyDef funnelbdr;
        funnelbdr.angle = 65;
        funnelbdr.position.Set(15.0f,40.0f);
        funnelbdr.type= b2_staticBody;

        b2Body* funnelbr = m_world->CreateBody(&funnelbdr);
        b2FixtureDef *funnelfr = new b2FixtureDef;
        funnelfr->density = 40.0f;
        funnelfr->shape = new b2PolygonShape;
        funnelfr->shape = &funnelsr;
        funnelbr->CreateFixture(funnelfr);

    }
    {
        // Second Vertical Line
        b2PolygonShape funnelsr;
        funnelsr.SetAsBox(0.0f, 2.0f);
        b2BodyDef funnelbdr;
        funnelbdr.position.Set(13.4f,36.9f);
        funnelbdr.type= b2_staticBody;

        b2Body* funnelbr = m_world->CreateBody(&funnelbdr);
        b2FixtureDef *funnelfr = new b2FixtureDef;
        funnelfr->density = 40.0f;
        funnelfr->shape = new b2PolygonShape;
        funnelfr->shape = &funnelsr;
        funnelbr->CreateFixture(funnelfr);

    }
    {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 0.7;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 1.0f;
        ballfd.friction = 0.0f;
        ballfd.restitution = 0.0f;

        for (int i = 0; i < 10; i+=2)
        {
            b2BodyDef ballbd;
            ballbd.type = b2_dynamicBody;
            //change here
            ballbd.position.Set(24.0f, 39.6f + i*1.0f);
            spherebody = m_world->CreateBody(&ballbd);
            spherebody->CreateFixture(&ballfd);
        }
    }
    {
        // Second Vertical Line
        b2PolygonShape funnelsr;
        funnelsr.SetAsBox(2.0f , 0.2f);
        b2BodyDef funnelbdr;
        funnelbdr.position.Set(9.4f,33.9f);
        funnelbdr.type= b2_staticBody;
        b2Body* funnelbr = m_world->CreateBody(&funnelbdr);
        b2FixtureDef *funnelfr = new b2FixtureDef;
        funnelfr->density = 40.0f;
        funnelfr->shape = new b2PolygonShape;
        funnelfr->shape = &funnelsr;
        funnelbr->CreateFixture(funnelfr);

    }
    {
        // Second Vertical Line
        b2PolygonShape funnelsr;
        funnelsr.SetAsBox(2.0f , 0.2f);
        b2BodyDef funnelbdr;
        funnelbdr.position.Set(15.6f,33.9f);
        funnelbdr.type= b2_staticBody;
        b2Body* funnelbr = m_world->CreateBody(&funnelbdr);
        b2FixtureDef *funnelfr = new b2FixtureDef;
        funnelfr->density = 40.0f;
        funnelfr->friction=2.0f;
        funnelfr->shape = new b2PolygonShape;
        funnelfr->shape = &funnelsr;
        funnelbr->CreateFixture(funnelfr);

    }
    {

    }
    //second pulley system
    {
        b2BodyDef *bd = new b2BodyDef;
        bd->type = b2_dynamicBody;
        bd->position.Set(-10,15);
        bd->fixedRotation = true;

        //The open box
        b2FixtureDef *fd1 = new b2FixtureDef;
        fd1->density = 3.0;
        fd1->friction = 0.5;
        fd1->restitution = 0.f;
        fd1->shape = new b2PolygonShape;
        b2PolygonShape bs1;
        bs1.SetAsBox(2,2, b2Vec2(0.f,-1.9f), 0);
        fd1->shape = &bs1;

        b2Body* box1 = m_world->CreateBody(bd);
        box1->CreateFixture(fd1);

        b2PolygonShape funnelsr;
        funnelsr.SetAsBox(2.0f , 0.2f);
        b2BodyDef funnelbdr;
        funnelbdr.position.Set(12.5f,34.9f);
        funnelbdr.type= b2_dynamicBody;
        b2Body* funnelbr = m_world->CreateBody(&funnelbdr);
        b2FixtureDef *funnelfr = new b2FixtureDef;
        funnelfr->density = 40.0f;
        funnelfr->shape = new b2PolygonShape;
        funnelfr->shape = &funnelsr;
        funnelfr->friction = 0.5f;
        funnelbr->CreateFixture(funnelfr);
        //box2->CreateFixture(fd1);

        // The pulley joint
        b2PulleyJointDef* myjoint = new b2PulleyJointDef();
        b2Vec2 worldAnchorOnBody1(-8, 29); // Anchor point on body 1 in world axis
        b2Vec2 worldAnchorOnBody2(8, 29); // Anchor point on body 2 in world axis
        b2Vec2 worldAnchorGround1(-8, 34); // Anchor point for ground 1 in world axis
        b2Vec2 worldAnchorGround2(8, 34); // Anchor point for ground 2 in world axis
        float32 ratio = 1.0f; // Define ratio
        myjoint->Initialize(box1, funnelbr, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), funnelbr->GetWorldCenter(), ratio);
        m_world->CreateJoint(myjoint);
    }

    //code for explosion 
      {
          int numRays=100;
          float blastPower=100000.0f;
        for (int i = 0; i < numRays; i++) {
          float angle = (i / (float)numRays) * 2*3.14;
          b2Vec2 rayDir( sinf(angle), cosf(angle) );
      
          b2BodyDef bd;
          bd.type = b2_dynamicBody;
          bd.fixedRotation = true; // rotation not necessary
          bd.bullet = true; // prevent tunneling at high speed
          bd.linearDamping = 2.5; // drag due to moving through air
          bd.gravityScale = 0; // ignore gravity
          //bd.position = center; // start at blast center
          bd.position.Set(0.0f,0.0f);
          bd.linearVelocity = blastPower * rayDir;
          b2Body* body = m_world->CreateBody( &bd );
      
          b2CircleShape circleShape;
          circleShape.m_radius = 0.1; // very small
      
          b2FixtureDef fd;
          fd.shape = &circleShape;
          fd.density = 60 / (float)numRays; // very high - shared across all particles
          fd.friction = 0; // friction not necessary
          fd.restitution = 10.99f; // high restitution to reflect off obstacles
          fd.filter.groupIndex = -3; // particles should not collide with each other -1 already used
          body->CreateFixture( &fd );
      }


    }





}

sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
