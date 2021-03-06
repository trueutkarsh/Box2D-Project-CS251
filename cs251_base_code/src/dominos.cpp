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
   
   void base_sim_t :: mouse_down(const b2Vec2& p) {//when any object is clicked
    m_mouseWorld = p;
     if(base_sim_t::mousej!=NULL)//if the joint is already there return.
 {
   return;
 }
  //form a joint btw mouse and object
    b2Vec2 d;
    d.Set(1000.0f/3, 1000.0f/3);
    //this is where the range is set for the region in which point is suppose to be checked.
    m_world_AABB.lowerBound = p - d;//lower bound
    m_world_AABB.upperBound = p + d;//upperbound

    QueryCallback callback(p);//declare a callback.
    m_world->QueryAABB(&callback, m_world_AABB);//from QueryAABB m_fixture

    if (callback.m_fixture) {// if query is successfull,there exists a body with such fixtures
      // and returned.

        b2Body* body = callback.m_fixture->GetBody();//body is the "body" which will drag.
        b2MouseJointDef md;//define a mousejoint
        md.bodyA = m_ground_body;
        md.bodyB = body;//body to move
        md.target = p;//the vevtor towards which it moves
        md.maxForce = 1000.0f * body->GetMass();
        md.collideConnected =true;
        mousej = (b2MouseJoint*)m_world->CreateJoint(&md);//create a mousejoint
        body->SetAwake(true);


    }
  }

void  base_sim_t:: mouse_up(const b2Vec2& p) {//when mouse is released

    if (mousej)//if mouse joint exists
    {
        //destroy the mouse joint.
        m_world->DestroyJoint(mousej);
        mousej= NULL;
       
    }
    //else do nothing

    }

void  base_sim_t:: mouse_move(const b2Vec2& p) {//when pressed
        m_mouseWorld = p;
        if (mousej != NULL)//if mouse joint exists
        {
            //change the target
            mousej->SetTarget(p);
        }
        } 

  
  dominos_t::dominos_t()
  {
 //! **************************************************************************************************************************************\n

    //! Base ---------------------------------------------------------------------------------------------------------------------------------\n
//! This is the base of the simulation , the ground on which the complete simulation occurs
  

    b2Body* b1;  
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }
    /// \n

 //! **************************************************************************************************************************************\n

    //! Dominos---------------------------------------------------------------------------------------------------------------------------------\n
//! These are 10 dominos seperated by 1 unit distance.
    ///They collide with each other in a sequential manner to achieve the effect. 
    ///description of shape of dominos\n
    

    {
      	
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);/**$ Shape :Box Dimension: 0.1 * 1.0 */
	
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;    /**$ Density : 20.0 units*/ 
      fd.friction = 0.1f;    /**$ Friction : 0.1 */
		
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(25.0f + 1.0f * i, 33.0f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }
//-------------------------------------------------------------------------------//
//! **************************************************************************************************************************************\n

    //! Seperator ---------------------------------------------------------------------------------------------------------------------------------\n
    ///----------------------------------------------------------------------------
    ///Five sided polygon(divider) present at the divider(left top) makes the coming balls to seperate.

    {
      b2Body* pentagon;
      b2PolygonShape pentagons;
      b2Vec2 vertices[5];
      vertices[0].Set(-1.0,0); /** $Five vertices of polygon : (-1,0)(1,0) (1,0.5) (0 , 1.0) (-1.1,0.5) */
      vertices[1].Set(1.0,0);
      vertices[2].Set(1.0,0.5);
      vertices[3].Set(0.0,1.0);
      vertices[4].Set(-1.1,0.5);
      pentagons.Set(vertices, 5);
      b2FixtureDef pentagonf;
      pentagonf.shape = &pentagons;
      pentagonf.density = 10.0f;
      pentagonf.friction = 0.0f;
      pentagonf.restitution = 0.0f;
      b2BodyDef pentagonbd;
      pentagonbd.position.Set(-20.0f, 40.0f);
       float DEGTORAD = 3.14/180;
      pentagonbd.type = b2_dynamicBody;
      pentagon = m_world->CreateBody(&pentagonbd);
      pentagon->CreateFixture(&pentagonf);
      pentagon->SetGravityScale(0);

      pentagon->SetTransform( b2Vec2( -20.0f, 40.0f ), 10*DEGTORAD );

      b2PolygonShape points;
      
      points.SetAsBox(0.2f, 2.0f);
      b2BodyDef pointbd;
      pointbd.position.Set(-20.0f, 40.0f);
      b2Body* point = m_world->CreateBody(&pointbd);
     
      b2RevoluteJointDef jointDef; /** It has been hinged to a point at its top and set to rotate a specified angle of 25 degree max*/
      jointDef.bodyA = point;
      jointDef.bodyB = pentagon;
      jointDef.enableLimit = true;
      jointDef.lowerAngle = -25 * DEGTORAD;
      jointDef.upperAngle =  25 * DEGTORAD;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
  }
//! \n

    ///Lines in seperator
    ///These lines protect the balls from going here and there , making them fall straight down\n
	/// There are in total 4 lines for same purpose
    {
      b2PolygonShape line1s;
      line1s.SetAsBox(0.0f, 4.0f); /** $Their height is set to 4 with width 0 */
      
      b2BodyDef line1bd;
      line1bd.position.Set(-20.0f, 35.0f); /** $Position set to -20,35*/
      b2Body* line1b = m_world->CreateBody(&line1bd);
      
      b2FixtureDef *line1fd = new b2FixtureDef;
      line1fd->shape = new b2PolygonShape;
      line1fd->shape = &line1s;
      line1b->CreateFixture(line1fd);
    }
    {
          b2PolygonShape line1s;
      line1s.SetAsBox(0.0f, 6.0f);
      
      b2BodyDef line1bd;
      line1bd.position.Set(-22.2f, 37.0f);
      b2Body* line1b = m_world->CreateBody(&line1bd);
      
      b2FixtureDef *line1fd = new b2FixtureDef;
      line1fd->shape = new b2PolygonShape;
      line1fd->shape = &line1s;
      line1b->CreateFixture(line1fd);
    }
    {
          b2PolygonShape line1s;
      line1s.SetAsBox(0.0f, 6.0f);
      
      b2BodyDef line1bd;
      line1bd.position.Set(-17.8f, 37.0f);
      b2Body* line1b = m_world->CreateBody(&line1bd);
      
      b2FixtureDef *line1fd = new b2FixtureDef;
      line1fd->shape = new b2PolygonShape;
      line1fd->shape = &line1s;
      line1b->CreateFixture(line1fd);


    }
    //line upper
    {
      b2PolygonShape line1s;
      line1s.SetAsBox(0.0f, 4.0f);
      
      b2BodyDef line1bd;
      line1bd.position.Set(-19.5f, 47.0f);
      b2Body* line1b = m_world->CreateBody(&line1bd);
      
      b2FixtureDef *line1fd = new b2FixtureDef;
      line1fd->shape = new b2PolygonShape;
      line1fd->shape = &line1s;
      line1b->CreateFixture(line1fd);
    }
    {
          b2PolygonShape line1s;
      line1s.SetAsBox(0.0f, 4.0f);
      
      b2BodyDef line1bd;
      line1bd.position.Set(-20.5f, 47.0f);
      b2Body* line1b = m_world->CreateBody(&line1bd);
      
      b2FixtureDef *line1fd = new b2FixtureDef;
      line1fd->shape = new b2PolygonShape;
      line1fd->shape = &line1s;
      line1b->CreateFixture(line1fd);
    }
//! **************************************************************************************************************************************\n

    //! Starting Sphere Balls ---------------------------------------------------------------------------------------------------------------------------------\n
//spherballs starting
	///These sphere balls replicate the purpose of fruits to be added in our machine
    /// \n There are 4 balls with one non fruit ball as to make the three other go into the jar

    {
      b2Body* spherebody;
  
      b2CircleShape circle;
      circle.m_radius = 0.5f; /** $radius : 0.5 */
  
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 25.0f; /** $density :25.0 */
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
        
      for (int i = 0; i < 4; ++i)
  {
    b2BodyDef ballbd;
    ballbd.type = b2_dynamicBody;
    ballbd.position.Set(-20.0f, 44.0f+ i*2.0f);
    spherebody = m_world->CreateBody(&ballbd);
    spherebody->CreateFixture(&ballfd);
  }
    }
//  the plank and the ball system
//! **************************************************************************************************************************************\n

    //! The SeeSaw System ---------------------------------------------------------------------------------------------------------------------------------\n
  	///Description of code for triangular wedge,system(see saw ) just present below the 
  	///entry of the balls.  
  	///It has three components a traingular wedge , a plank attached to it and ball present over it..\n

  {
      float DEGTORAD = 3.14/180;
	/// -----
      ///The triangle wedge
      
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];    /** Trinagular in shape , Fixed in space */
      vertices[0].Set(-1,0); /** $Vertices : (-1,0) (0,1.5) (1,0)*/
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f; 
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-15.0f, 25.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
      ///the plank present on the
      ///top of the wedge.\n
      b2PolygonShape shape;
      shape.SetAsBox(5.0f, 0.2f); /** $Shape: Box with Dimension : 5*0.2 */ 
      b2BodyDef bd2;
      bd2.position.Set(-15.0f, 26.5f); /** $Position : -15 , 26.5 */
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-15.0f, 26.5f);
      jd.enableLimit = true;
      jd.lowerAngle = -20 * DEGTORAD;
      jd.upperAngle =  20 * DEGTORAD;
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //the sphere on the plank
      ///Sphere present just over see saw initially.
      b2Body* spherebody;
  
      b2CircleShape circle; /** $Shape : Circle */
      circle.m_radius = 0.5f;/** $Radius : 0.5 */
  
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;

      b2BodyDef ballbd;
    ballbd.type = b2_dynamicBody;
    ballbd.position.Set(-14.9f, 26.5f);/** $Position : -14.9 , 26.5 */
    spherebody = m_world->CreateBody(&ballbd);
    spherebody->CreateFixture(&ballfd);
///-------------------------------------------------------------------------------------------------------------------------------\n
    ///Reflection lines
    ///It consists of two lines,both  with high coefficient of resitution so as to increase
    ///the speed of ball.
    {
    	///For first line \n
          b2PolygonShape line1s;
      line1s.SetAsBox(0.0f, 10.0f); /** $Shape: Box */
      
      b2BodyDef line1bd;
      b2Body* line1b = m_world->CreateBody(&line1bd);
      line1b->SetTransform( b2Vec2( -1.5f, 40.0f ), -80*DEGTORAD );
      b2FixtureDef *line1fd = new b2FixtureDef;
      line1fd->shape = new b2PolygonShape;
      line1fd->shape = &line1s;
      line1fd->restitution=5.0f; /** $Restitution : 5 */
      line1b->CreateFixture(line1fd);
    }
//! **************************************************************************************************************************************\n

    //! Reflector Lines ---------------------------------------------------------------------------------------------------------------------------------\n
    {
    	///This is the code for second line.
	/// Similarly they are defined
          b2PolygonShape line1s;
      line1s.SetAsBox(0.0f, 8.0f);
      
      b2BodyDef line1bd;
      b2Body* line1b = m_world->CreateBody(&line1bd);
      line1b->SetTransform( b2Vec2( -0.0f, 35.0f ), -95*DEGTORAD );
      b2FixtureDef *line1fd = new b2FixtureDef;
      line1fd->shape = new b2PolygonShape;
      line1fd->shape = &line1s;
      line1fd->restitution=5.0f;
      line1b->CreateFixture(line1fd);
    }

     {
          b2PolygonShape line1s;
      line1s.SetAsBox(0.0f, 1.0f);
      
      b2BodyDef line1bd;
      b2Body* line1b = m_world->CreateBody(&line1bd);
      line1b->SetTransform( b2Vec2( 10.0f, 41.9f ), -90*DEGTORAD );
      b2FixtureDef *line1fd = new b2FixtureDef;
      line1fd->shape = new b2PolygonShape;
      line1fd->shape = &line1s;
      line1fd->restitution=0.01f;
      line1b->CreateFixture(line1fd);
    }

    {
          b2PolygonShape line1s;
      line1s.SetAsBox(0.0f, 1.0f);
      
      b2BodyDef line1bd;
      b2Body* line1b = m_world->CreateBody(&line1bd);
      line1b->SetTransform( b2Vec2( 12.5f, 40.0f ),0);
      b2FixtureDef *line1fd = new b2FixtureDef;
      line1fd->shape = new b2PolygonShape;
      line1fd->shape = &line1s;
      line1fd->restitution=0.0f;
      line1b->CreateFixture(line1fd);
    }
//! **************************************************************************************************************************************\n

    //! Pulley System ---------------------------------------------------------------------------------------------------------------------------------\n
   /// Following is code for pulley system present on right. \n
    ///It consists of an open box and the horizontal  bar.
    ///Both of them are joint by pullry joint.\n
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(12.5,35);
      bd->fixedRotation = true;
      ///Open box \n

      //The open box
	/** $Shape : A hollow Box , made of three lines attached together at 90deg */
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0; /** $Density : 10.0 */
      fd1->friction = 0.5; /** $Friction : 0.5 */
      fd1->restitution = 0.0f;/** $The same values are set for other part of the box too.*/
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(0.9,0.02, b2Vec2(0.f,-0.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.02,0.9, b2Vec2(0.9f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.0f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.02,0.9, b2Vec2(-0.9f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);
      
      ///The bar \n
      /// It is the other side of the pulley system  for the purpose of balancing
      bd->position.Set(16,35);  
      fd1->density = 34.0;   /** $Density 34.0 */
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);
    	///----------------------------------------------------------------------------------------------------------------------\n
      /// The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(12.5, 35); /** $Anchor point on body 1 in world axis 12.5, 35 */
      b2Vec2 worldAnchorOnBody2(16, 35); /** $Anchor point on body 2 in world axis 16 , 35 */
      b2Vec2 worldAnchorGround1(12.5, 37); /** $Anchor point for ground 1 in world axis 12.5 , 37 */
      b2Vec2 worldAnchorGround2(16, 37); /** $Anchor point for ground 2 in world axis 16, 37 */
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }


    {
    	
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.2f, 0.5f);
	
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 0.005f;
      fd.friction = 1000.0f;
		
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(25.5f + 1.0f * i, 31.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->SetGravityScale(-1);
	  body->CreateFixture(&fd);
	}

    }
//! **************************************************************************************************************************************\n

    //! The Platform ---------------------------------------------------------------------------------------------------------------------------------\n
    // the plaform 
    {
    	    //Another horizontal shelf
    //Following is the definition of horizontal shelf to which the upsided dominos are attached.	
    {
      b2PolygonShape shape; /** $Shape :Box with Dimension 7 * 0.25 */
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);
	
      b2BodyDef bd;
      bd.position.Set(51.0f, 12.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
     
    
    }
//! **************************************************************************************************************************************\n

    //! Hydraulics ---------------------------------------------------------------------------------------------------------------------------------\n
    ///The liquid is has been denoted by collection very tiny balls(liquid particles).
    ///It is contained in a U shaped box(two open joint container) open from two sides with a platform on one side.
    ///pushing the platform down,pushes the liquid out of container

    	{	 //The open box 1

    	///Follwoing is the structural defintion of the open container 1.
    		b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_staticBody;
      bd->position.Set(5.5,21);/** $position : 5.5 21 */
      bd->fixedRotation = true;

      b2FixtureDef *fd1 = new b2FixtureDef; /** $Shape : Box with one side open */
      fd1->density = 10.0; /** $Density 10.0 */
      fd1->friction = 0.0; /** $Friction 0 */
      fd1->restitution = 0.0f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(0.9,0.02, b2Vec2(0.f,-2.0f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.0;
      fd2->restitution = 0.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.02,2.0, b2Vec2(0.9f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.0;
      fd3->restitution = 0.0f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.02,2.0, b2Vec2(-0.9f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

  }

   ///The open box2
	///Its Definition is same as the one before but is of a smaller size
  {

  		
  		b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_staticBody;
      bd->position.Set(5,19.8);
      bd->fixedRotation = true;

      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.0;
      fd1->restitution = 0.0f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(3.0,0.02, b2Vec2(0.f,-2.0f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.0;
      fd2->restitution = 0.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.02,1.0, b2Vec2(2.60f,-1.5f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.0;
      fd3->restitution = 0.0f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.02,2.0, b2Vec2(-3.0f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);
	}
///---------------------------------------------------------------------------------------------------------------------------------------------------\n
	/// Water
	/// Water has been replicated by balls of small radius
	{

{
      b2Body* spherebody;
  
      b2CircleShape circle; /** $Shape : Circle */
      circle.m_radius = 0.2f;/** $Radius : 0.2 */
  
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 2.0f; /** $Density : 2.0 */
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.5f; /** Restitution : 0.5 */
  
      for (int i = 0; i < 70; ++i)
  {
    b2BodyDef ballbd;
    ballbd.type = b2_dynamicBody;
    ballbd.position.Set(3.0f + i*0.005, 19.0f+ i*0.05f);
    spherebody = m_world->CreateBody(&ballbd);
    spherebody->CreateFixture(&ballfd);
  }
    }

	}
	//the  closing slid
 {
 	///Container 1 has a closing lid .
 	/// Its Dimension are perfectly matched with space between the two boxes to close it without leakage
          b2PolygonShape line1s;
      line1s.SetAsBox(1.2f, 0.1f);
      
      b2BodyDef line1bd;
      line1bd.type = b2_dynamicBody;
line1bd.position.Set(3.3,25.0);
    line1b1 = m_world->CreateBody(&line1bd);
      
      b2FixtureDef *line1fd = new b2FixtureDef;
      line1fd->shape = new b2PolygonShape;
      line1fd->shape = &line1s;
      line1fd->restitution=0.0f;
      
      line1b1->CreateFixture(line1fd);
    }
    
//! **************************************************************************************************************************************\n

    //! Rotating Fans(5 Gears peeler) ---------------------------------------------------------------------------------------------------------------------------------\n
//----------------------------------------------------------
//the rotating wings with cover
    ///It basically takes ball(fruit) from one side of the wing,sends it to bottom and returns it back
    /// to top through rotations.\n
    {
    	///Description for side covers,three lines which cover up the fans.
        {
            b2PolygonShape funnelsr;
            funnelsr.SetAsBox(0.0f, 7.0f); /** $Shape: Box */
            b2BodyDef funnelbdr;
            funnelbdr.position.Set(-30.2f,20.5f); /** $Position: -30.2 , 20.5 */
            funnelbdr.type= b2_staticBody; /** BODY Type: Static */
            b2Body* funnelbr = m_world->CreateBody(&funnelbdr);
            b2FixtureDef *funnelfr = new b2FixtureDef;
            funnelfr->density = 1.0f; 
            funnelfr->restitution = 0.0f;
            funnelfr->shape = new b2PolygonShape;
            funnelfr->shape = &funnelsr;
            funnelfr->filter.groupIndex=-1;
            funnelbr->CreateFixture(funnelfr);
        }
        {
            b2PolygonShape funnelsr;
            funnelsr.SetAsBox(0.0f, 7.0f);
            b2BodyDef funnelbdr;
            funnelbdr.position.Set(-26.8f,20.5f);
            funnelbdr.type= b2_staticBody;

            b2Body* funnelbr = m_world->CreateBody(&funnelbdr);
            b2FixtureDef *funnelfr = new b2FixtureDef;
            funnelfr->density = 1.0f;
             funnelfr->restitution = 0.0f;
            funnelfr->shape = new b2PolygonShape;
            funnelfr->shape = &funnelsr;
            funnelfr->filter.groupIndex=-1;
            funnelbr->CreateFixture(funnelfr);
        }
        {
            b2PolygonShape funnelsr;
            funnelsr.SetAsBox(1.8f, 0.0f);
            b2BodyDef funnelbdr;
            funnelbdr.position.Set(-28.5f,13.9f);
            funnelbdr.type= b2_staticBody;

            b2Body* funnelbr = m_world->CreateBody(&funnelbdr);
            b2FixtureDef *funnelfr = new b2FixtureDef;
            funnelfr->density = 1.0f;
            funnelfr->shape = new b2PolygonShape;
             funnelfr->restitution = 0.0f;
            funnelfr->shape = &funnelsr;
            funnelfr->filter.groupIndex=-1;
            funnelbr->CreateFixture(funnelfr);
        }




        {
            //group index -1 here,the wont collide with themslevs
            //change groupindex of other objects
            //this is the new rotating wind mill
///--------------------------------------------------------------------------------------------------------------------------------------------------\n
        	///Rotating fans\n
        	///It basically consisits of two rectangular bodies attached to each other.\n
        	///revolute joint has been used here,for continuous rotation of fans.\n
            b2PolygonShape shape,shape3;
            shape.SetAsBox(1.6f, 0.2f); /** $Shape: Box with Dimension 1.6*0.2 */
            shape3.SetAsBox(0.2f,1.6f);
            for(int i=0; i<5; i++)
            {
                b2BodyDef bd;
                bd.position.Set(-28.5f, 25.7f-2.5f*i); /** $Postion : Set with a relative gap of 2.5 units between each fan */
                if(i%2==0)
                {
                    bd.angle=3.14/2-0.614;
                }
                else
                {
                    bd.angle=3.14/2+0.614;
                }
                bd.type=b2_kinematicBody; /** $BODY TYPE = Kinematic Body */
                b2Body* body = m_world->CreateBody(&bd);
                b2FixtureDef *fd = new b2FixtureDef;
                fd->density = 1.f;
                fd->restitution = 0.0f;
                fd->filter.groupIndex = -1;
                fd->shape = new b2PolygonShape;
                fd->shape = &shape;
              
                b2FixtureDef *fd1 = new b2FixtureDef;
                fd1->density = 1.f;
                  fd1->restitution = 1.0f;
                fd1->filter.groupIndex = -1;
                fd1->shape = new b2PolygonShape;
                fd1->shape = &shape3;
                body->CreateFixture(fd);
                body->CreateFixture(fd1);
                body->SetAngularVelocity(3);



                //useless body
                b2PolygonShape shape2;
                shape2.SetAsBox(0.2f, 1.5f);
                b2BodyDef bd2;
                bd2.position.Set(-28.5f, 25.7f-2.5f*i);
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
//! **************************************************************************************************************************************\n

    //! The Bullet Shoot ---------------------------------------------------------------------------------------------------------------------------------\n
    	///The blast which takes place at the start,throws a small ball.
    	///from left right to left.
    	///Ray technique has been used for the blast simulation.\n
  {
          int numRays=1;
          float blastPower=23.0f;
        for (int i = 0; i < numRays; i++) {
          float angle = (-0.25 / (float)numRays) * 2*3.14;
          b2Vec2 rayDir( sinf(angle), cosf(angle) );
      
          b2BodyDef bd;
          bd.type = b2_dynamicBody;
          bd.fixedRotation = true; // rotation not necessary
          bd.bullet = true; // prevent tunneling at high speed
          bd.linearDamping = 0; // drag due to moving through air
          bd.gravityScale = 0; // ignore gravity
          //bd.position = center; // start at blast center
          bd.position.Set(45.0f,30.3f);
          bd.linearVelocity = blastPower * rayDir;
          b2Body* body = m_world->CreateBody( &bd );
      
          b2CircleShape circleShape;
          circleShape.m_radius = 0.1; // very small
      
          b2FixtureDef fd;
          fd.shape = &circleShape;
          fd.density = 3550.0f; // very high - shared across all particles
          fd.friction = 0; // friction not necessary
          fd.restitution = 0.0f; // high restitution to reflect off obstacles
          fd.filter.groupIndex = -1; // particles should not collide with each other -1 already used
          body->CreateFixture( &fd );
      }



}
// teleporter
//! **************************************************************************************************************************************\n

    //! Teleporter ---------------------------------------------------------------------------------------------------------------------------------\n
///At many places in the simulation,objects suddenly diappear from one place and appear somewhere else
///Following is the code for the teleporter,which is being used at multiple places.
///Specific objects which get in contact with teleporter,have sudden change in their positions.
/// There are three teleporters in the world
{ 
  float shift = -22.5;
  float shifty = 1.5;
///Following is the code for teleporter 1 
			{

				b2Vec2 vs[5];     /** $Shape : Oval */

			vs[0].Set(2.0f+ shift, 10.6f+shifty); 

			vs[1].Set(1.0f+shift, 10.25f+shifty);

			vs[2].Set(0.0f+shift, 10.0f+shifty);

			vs[3].Set(-1.0f+shift, 10.25f+shifty);
			vs[4].Set(-2.0f+shift, 10.6f+shifty);

			b2ChainShape chain;
			chain.CreateChain(vs, 5);
			b2BodyDef bd;

			bodyc = m_world->CreateBody(&bd);
			bodyc->SetUserData(this);
			b2FixtureDef fd;
			fd.shape = &chain;
			bodyc->CreateFixture(&fd);


			}

			{

				b2Vec2 vs[5];

			vs[0].Set(2.0f + shift, 10.6f+shifty);

			vs[1].Set(1.0f + shift, 10.95f+shifty);

			vs[2].Set(0.0f + shift, 11.2f+shifty);

			vs[3].Set(-1.0f + shift, 10.95f+shifty);
			vs[4].Set(-2.0f+ shift, 10.6f+shifty);

			b2ChainShape chain;
			chain.CreateChain(vs, 5);
			b2BodyDef bd;

			bodyc = m_world->CreateBody(&bd);
			bodyc->SetUserData(this); /** $Set User data specified to ddetect collision */
			b2FixtureDef fd;
			fd.shape = &chain;

			bodyc->CreateFixture(&fd);


			}
}

{ 
  float shift = 22.0;
  float shifty = 5.0;
      {

        b2Vec2 vs[5];

      vs[0].Set(2.0f+ shift, 10.6f+shifty);

      vs[1].Set(1.0f+shift, 10.25f+shifty);

      vs[2].Set(0.0f+shift, 10.0f+shifty);

      vs[3].Set(-1.0f+shift, 10.25f+shifty);
      vs[4].Set(-2.0f+shift, 10.6f+shifty);

      b2ChainShape chain;
      chain.CreateChain(vs, 5);
      b2BodyDef bd;
      
      bodyc = m_world->CreateBody(&bd);
      bodyc->SetUserData(this);
      b2FixtureDef fd;
      fd.shape = &chain;
      fd.filter.groupIndex = -1;
      bodyc->CreateFixture(&fd);



      }

      {

        b2Vec2 vs[5];

      vs[0].Set(2.0f + shift, 10.6f+shifty);

      vs[1].Set(1.0f + shift, 10.95f+shifty);

      vs[2].Set(0.0f + shift, 11.2f+shifty);

      vs[3].Set(-1.0f + shift, 10.95f+shifty);
      vs[4].Set(-2.0f+ shift, 10.6f+shifty);

      b2ChainShape chain;
      chain.CreateChain(vs, 5);
      b2BodyDef bd;
 
      bodyc = m_world->CreateBody(&bd);
      bodyc->SetUserData(this);
      b2FixtureDef fd;
      fd.shape = &chain;
      fd.friction= 0.0f;
      fd.filter.groupIndex = -1;
      bodyc->CreateFixture(&fd);
          


      }
}

//teleporter 3


{ 
  float shift = 8.0;
  float shifty = 30.0;
      {

        b2Vec2 vs[5];

      vs[0].Set(2.0f+ shift, 10.6f+shifty);

      vs[1].Set(1.0f+shift, 10.25f+shifty);

      vs[2].Set(0.0f+shift, 10.0f+shifty);

      vs[3].Set(-1.0f+shift, 10.25f+shifty);
      vs[4].Set(-2.0f+shift, 10.6f+shifty);

      b2ChainShape chain;
      chain.CreateChain(vs, 5);
      b2BodyDef bd;
      
      bodycl = m_world->CreateBody(&bd);
      bodycl->SetUserData(this);
      b2FixtureDef fd;
      fd.shape = &chain;
      fd.filter.groupIndex = -1;
      bodycl->CreateFixture(&fd);



      }

      {

        b2Vec2 vs[5];

      vs[0].Set(2.0f + shift, 10.6f+shifty);

      vs[1].Set(1.0f + shift, 10.95f+shifty);

      vs[2].Set(0.0f + shift, 11.2f+shifty);

      vs[3].Set(-1.0f + shift, 10.95f+shifty);
      vs[4].Set(-2.0f+ shift, 10.6f+shifty);

      b2ChainShape chain;
      chain.CreateChain(vs, 5);
      b2BodyDef bd;
 
      bodyc = m_world->CreateBody(&bd);
      bodyc->SetUserData(this);
      b2FixtureDef fd;
      fd.shape = &chain;
      fd.friction= 0.0f;
      fd.filter.groupIndex = -1;
      bodyc->CreateFixture(&fd);
          


      }
}


//! **************************************************************************************************************************************\n

    //! Curved Path ---------------------------------------------------------------------------------------------------------------------------------\n
///There is a curverd path on the left,which consisits of arcs and curves.
///There are in total 2 Curved paths and 2 skating platform having more or less same properties

// the curved path

   {
        float x0 = -35.0f, y0 = 23.0, r= 8,init=0.00;

        b2Body* b1; 
        b2EdgeShape shape; /**$Shape : Circular arc */
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
        float x=x0 - r*init ;
        float y=y0 + r * sqrt(1 - init*init);
        for(float i=init; i <=0.9; i+=0.05){ /** Body Type : Static path */
          float tempx = x0 - r * i;
          float tempy = y0 + r * sqrt(1 - i*i);
          shape.Set(b2Vec2(tempx,tempy), b2Vec2(x, y));
          b2FixtureDef fd;
          fd.shape = &shape;
          fd.friction = 10.0f;
          b1->CreateFixture(&fd); 
          x=tempx;
          y=tempy;
        }

    }


       {
        float x0 = -36.0f, y0 = 24.0, r= 8,init=0.09;

        b2Body* b1; 
        b2EdgeShape shape; 
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
        float x=x0 - r*init ;
        float y=y0 + r * sqrt(1 - init*init);
        float i;
        b2FixtureDef fd; 
        for(i=init; i <=1.0; i+=0.05){
          float tempx = x0 - r * i;
          float tempy = y0 + r * sqrt(1 - i*i);
          shape.Set(b2Vec2(tempx,tempy), b2Vec2(x, y));
          
          fd.shape = &shape;
          fd.friction = 100.0f;
          b1->CreateFixture(&fd); 
          x=tempx;
          y=tempy;
        }
           float tempx = x0-0.9;
          float tempy = y0 -0.2;
          shape.Set(b2Vec2(tempx,tempy), b2Vec2(x, y));

          fd.shape = &shape;
          fd.friction = 100.0f;
          b1->CreateFixture(&fd);
         

    }
//! **************************************************************************************************************************************\n

    //! Container ---------------------------------------------------------------------------------------------------------------------------------\n
// th
///There's a glass which contains both the liquid(water) and circles(fruit juice),finally.
    ///It appears at almost the end of simulation.\n
	///Its code is similar to open box with properties changed\n 

    {

    	 //The open container
  {

  		b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-52.0,1200.0); /** $Initial Position -52 , 1200 */
      bd->fixedRotation = true;

      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 11.0; /** $Density : 11 */
      fd1->friction = 110.0; /** $Friction : 110 */
      fd1->restitution = 0.0f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2.0,0.02, b2Vec2(0.f,-2.0f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 11.0;
      fd2->friction = 111.0;
      fd2->restitution = 0.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.02,2.0, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 11.0;
      fd3->friction = 11011.0;
      fd3->restitution = 0.0f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.02,2.0, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);
	}
    }

    
    {

    	///The platform
    	{
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(18.0f, 35.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(18.0f, 37.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }

    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.01f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(18.0f, 39.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }


    }
//! **************************************************************************************************************************************\n

    //! The pendulum ---------------------------------------------------------------------------------------------------------------------------------\n
// the pendulum

    ///Description of the code for the pendulum present on right top which knocks off the the dominos.

    {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.5f);
	  
	b2BodyDef bd;
	bd.position.Set(20.5f, 28.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);
	  
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(20.5f, 33.0f);
	b4 = m_world->CreateBody(&bd);
  b2FixtureDef fd;
  fd.shape = &shape;
  fd.density = 2.0f;
	b4->CreateFixture(&fd);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(21.0f, 40.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }
//! **************************************************************************************************************************************\n

    //! The Flying Sphere ---------------------------------------------------------------------------------------------------------------------------------\n
    /// Its Normally a sphere ball with gravity set to negative
   {
   	///Description for sphere which flies away during simulation.\n
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 0.8f; /** $Radius : 0.8 */
	  
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 5.0f; /** $Density : 5.0 */
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(35.5f, 34.0f); /** $Position : 35.5 , 34 */
      sbody = m_world->CreateBody(&ballbd);
       sbody->SetGravityScale(-1); /** $Gravity Scale : -1 */
      sbody->CreateFixture(&ballfd);
    }
    	{
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(35.5f, 35.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(35.5f, 35.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }

	}

	{

	// the curved path
		//Following is the code for the two curved arcs present at the right bottom.
   {
        float x0 = -41.0f, y0 = 26.0, r= 8,init=0.00;

        b2Body* b1; 
        b2EdgeShape shape; 
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
        float x=x0 + r*init ;
        float y=y0 - r * sqrt(1 - init*init);
        for(float i=init; i <=1.0; i+=0.05){
          float tempx = x0 + r * i;
          float tempy = y0 - r * sqrt(1 - i*i);
          shape.Set(b2Vec2(tempx,tempy), b2Vec2(x, y));
          b1->CreateFixture(&shape, 0.0f); 
          x=tempx;
          y=tempy;
        }

    }

       {
        float x0 = -40.0f, y0 = 10.0f, r= 8,init=0.09;

        b2Body* b1; 
        b2EdgeShape shape; 
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
        float x=x0 - r*init ;
        float y=y0 +r * sqrt(1 - init*init);
        float i;
        for(i=init; i <=0.9; i+=0.05){
          float tempx = x0 - r * i;
          float tempy = y0 + r * sqrt(1 - i*i);
          shape.Set(b2Vec2(tempx,tempy), b2Vec2(x, y));
          b1->CreateFixture(&shape, 0.0f); 
          x=tempx;
          y=tempy;
        }

    }
    	{
        float x0 = -40.0f, y0 = 10.0f, r= 9.5,init=0.0;

        b2Body* b1; 
        b2EdgeShape shape; 
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
        float x=x0 - r*init ;
        float y=y0 +r * sqrt(1 - init*init);
        float i,tempx,tempy;
        for(i=init; i <=1.0; i+=0.05){
           tempx = x0 - r * i;
           tempy = y0 + r * sqrt(1 - i*i);
          shape.Set(b2Vec2(tempx,tempy), b2Vec2(x, y));
          b1->CreateFixture(&shape, 0.0f); 
          x=tempx;
          y=tempy;
        }
         shape.Set(b2Vec2(tempx,tempy-1.1), b2Vec2(x, y));
          b1->CreateFixture(&shape, 0.0f); 

    }




          
    		
	}
//! **************************************************************************************************************************************\n

    //! Tick Tock ---------------------------------------------------------------------------------------------------------------------------------\n
   //  the tick tock
	///Following is the code for multiplatform lever design.Its consists of for platforms each having
	///ball and rotating lever at the end.\n
	///Its consists of four platforms each having a lever attached to it at alternate ends. \n
	{

		{  ///------------------------------------------------------------------------------------------------------------------------------\n
///The platforms:\n
			b2PolygonShape shape;
			shape.SetAsBox(2.5f, 0.25f); /** $Shape : Box with Dimension 2.5 , 0.25 */
			for(int i=0;i<4;i++){
				b2BodyDef bd;
				bd.position.Set(43.0f, 39.0f - i*4.0f); /** $Postion: 43.0 , 39.0 - i*4.0 where i varies from 0 to 3 */
				b2Body* ground = m_world->CreateBody(&bd);
				ground->CreateFixture(&shape, 0.0f);

				b2PolygonShape shape;
				shape.SetAsBox(0.2f, 1.2f);
				/// The vertical rotatble bars:\n
				float pos =39.5f, a=-3.5f;  /** $Shape : Box with dimension 0.2 , 1.2 */
				if(i%2==0){ pos=46.5f; a =3.5f;}
				b2BodyDef bd1;
				bd1.position.Set(pos, 35.0f - i*4.0f);
				bd1.type = b2_dynamicBody;
				b2Body* body = m_world->CreateBody(&bd1);
				b2FixtureDef *fd = new b2FixtureDef;
				fd->density = 20.0f;
				fd->shape = new b2PolygonShape;
				fd->shape = &shape;
				body->CreateFixture(fd);

				b2RevoluteJointDef jointDef; /** $Platform and bars are attached using revolute joint */
				jointDef.bodyA = body;
				jointDef.bodyB = ground;
				jointDef.localAnchorA.Set(0,-1.6f);
				jointDef.localAnchorB.Set(a,0);
				jointDef.collideConnected = false;
				m_world->CreateJoint(&jointDef);
			}
	  }
	  	///The  spheres on the platform of tick tock have same basic feature of balls
	  ///Following is the code for the  the balls present on the tick-tock platform.
    {
     
      b2CircleShape circle;
      circle.m_radius = 0.8; /** $Radius : 0.8 */
	for(int i =0;i<3;i++)
	{
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f; /** Density : 1.0 */
      ballfd.friction = 0.01f; /** Friction: 0.01 */
      ballfd.restitution = 0.1f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      float pos;
      if(i%2==0)pos = 45.0f;
      else pos = 41.5f;
      ballbd.position.Set(pos, 37.0f - 4.0*i);
      this->sbody = m_world->CreateBody(&ballbd);
      this->sbody->CreateFixture(&ballfd);
  }
    }



	}


// the platform and sphere at right 
	
		 {
    	{
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(42.0f, 40.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(42.0f, 42.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }

    //The heavy sphere on the platform
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 5.0f;
      ballfd.friction = 0.5f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(42.0f, 44.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }


  }

  // the right skate platform

  {
  		   {
        float x0 = 45.0f, y0 = 24.0, r= 8,init=0.00;

        b2Body* b1; 
        b2EdgeShape shape; 
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
        float x=x0 + r*init ;
        float y=y0 - r * sqrt(1 - init*init);
        for(float i=init; i <=1.0; i+=0.05){
          float tempx = x0 + r * i;
          float tempy = y0 - r * sqrt(1 - i*i);
          shape.Set(b2Vec2(tempx,tempy), b2Vec2(x, y));
          b1->CreateFixture(&shape, 0.0f); 
          x=tempx;
          y=tempy;
        }

    }

       {
        float x0 = 45.0f, y0 = 24.0, r= 8,init=0.00;

        b2Body* b1; 
        b2EdgeShape shape; 
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
        float x=x0 - r*init ;
        float y=y0 - r * sqrt(1 - init*init);
        for(float i=init; i <=0.9; i+=0.05){
          float tempx = x0 - r * i;
          float tempy = y0 - r * sqrt(1 - i*i);
          shape.Set(b2Vec2(tempx,tempy), b2Vec2(x, y));
          b1->CreateFixture(&shape, 0.0f); 
          x=tempx;
          y=tempy;
        }

    }

    {
        float x0 = 29.5f, y0 = 23.0, r= 8,init=0.0;

        b2Body* b1; 
        b2EdgeShape shape; 
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
        float x=x0 + r*init ;
        float y=y0 - r * sqrt(1 - init*init);
        for(float i=init; i <=0.95; i+=0.05){
          float tempx = x0 + r * i;
          float tempy = y0 - r * sqrt(1 - i*i);
          shape.Set(b2Vec2(tempx,tempy), b2Vec2(x, y));
          b1->CreateFixture(&shape, 0.0f); 
          x=tempx;
          y=tempy;
        }

    }

       {
        float x0 = 29.5f, y0 = 23.0, r= 8,init=0.00;

        b2Body* b1; 
        b2EdgeShape shape; 
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
        float x=x0 - r*init ;
        float y=y0 - r * sqrt(1 - init*init);
        for(float i=init; i <=0.8; i+=0.05){
          float tempx = x0 - r * i;
          float tempy = y0 - r * sqrt(1 - i*i);
          shape.Set(b2Vec2(tempx,tempy), b2Vec2(x, y));
          b1->CreateFixture(&shape, 0.0f); 
          x=tempx;
          y=tempy;
        }

    }

  }

  

//! **************************************************************************************************************************************\n

    //! The Conveyor Belt ---------------------------------------------------------------------------------------------------------------------------------\n
    ///The idea is to make a rectangular chain
    ///Then, we insert rotating discs "in" the chain
    ///We keep a very high friction between these discs and the chain
    ///We give the discs a constant angular velocity, due to which the conveyor belt rotates
  
    {
      //overall shift parameters for the entire conveyor belt system
      double conveyorleft = -51.0;
       double conveyorup = 2.0;
       //The number of links in the conveyor belt system either above or below.
       //The total number of links will be 2*links + 4 (2 links also on either side)
       int links = 45; //This number must be divisible by 4 and > 20 so that 5 well spaced disks which rotate and drive the pulley can be layed out
       float PI = 3.14;
       double speed = -PI/8 ; //This is the speed at which the conveyor belts' driving wheels rotate
       
       //The links are made as follows.
       //First, we make the lower chain.
       //Then, we make the upper chain
       //Then, we make the two links which join these two chains on the left hand side
       //Then, we make the two links which join these two chains on the right hand side	
       b2Body* leftup; //this is the body which is the top left link
       b2Body* rightup; //this is the body which is the top right link
       b2Body* leftdown; //this is the body which is the bottom left link
       b2Body* rightdown; //this is the body which is the bottom right link
       //Note that these four special links are required because they are used specifically to connect the upper and lower chains
       
       //here we initialize them to default initial values
       leftup = NULL;
       rightup = NULL;
       leftdown = NULL;
       rightdown = NULL;
       //Making the upper part of the conveyor belt
       
       {
	 //defining each piece of the chain, along with the shape, size, fixture and relevant properties
	 b2BodyDef bd;
	 //The body is a dynamic body
	 bd.type = b2_dynamicBody;
	 bd.position.Set(10, 10);
	 b2Body* ground = m_world->CreateBody(&bd);
	 b2PolygonShape chainpiece;
	 //Dimensions
	 chainpiece.SetAsBox(1, 0.2); 
	 b2FixtureDef chainfix;
	 //Properties
	 chainfix.density = 50;
	 chainfix.friction = 0.5;
	 chainfix.restitution = 0.0;
	 chainfix.shape = &chainpiece;
	 //Defining a revolute joint for our purpose
	 b2Body *lastLink = ground;
	 b2RevoluteJointDef chainJoint;
	 //If the bodies are connected in the Box 2D world, then they should not collide. This is why this line is needed.
	 chainJoint.collideConnected = false;
    
	 for(int32 i = 0; i < links; i++)
	   {
	     double cons = 2.0;//This is the difference in length in between two anchors
	     b2BodyDef bd;
	     bd.type = b2_dynamicBody;
	     bd.position.Set(i*cons + conveyorleft, 0.0 + conveyorup);//setting up a new dynamic chain link
	     b2Body* nextLink = m_world->CreateBody(&bd);
	     nextLink->CreateFixture(&chainfix);
	     b2Vec2 anchor(i*cons - 1.0  + conveyorleft, 0.0 + conveyorup);//The anchor is set exactly at the meeting point of two chain links
	     if(i > 0)//We should not define a link for the first piece alone. We need atleast two pieces to begin linking
	       {
		 chainJoint.Initialize(lastLink, nextLink, anchor);
		 m_world->CreateJoint(&chainJoint);
	       }
	     //Updating the current link
	     lastLink = nextLink;
	     //assignment to the last link. The last link will essentially be rightdown, once we exit this loop
	     rightdown = nextLink;
	     //for the first link, we need to set leftdown, which is supposed to point to that link (body)
	     if(i == 0)
	       {
		 leftdown = nextLink;
	       }
	   }
       }

       //here we make the lower part of the conveyor belt chain
       {
	 double shiftup = 4.0;//this denotes the height to which we shift up the upper part of the chain
	 // The rest of the process is identical to the first part except that everything is shifted up
	 b2BodyDef bd;
	 bd.type = b2_dynamicBody;
	 bd.position.Set(conveyorleft, conveyorup + shiftup);
	 b2Body* ground = m_world->CreateBody(&bd);
	 b2PolygonShape chainpiece;
	 chainpiece.SetAsBox(1, 0.2);
	 b2FixtureDef chainfix;
	 chainfix.density = 50;
	 chainfix.friction = 0.5;
       chainfix.restitution = 0.0;
       chainfix.shape = &chainpiece;
       b2Body *lastLink = ground;
       b2RevoluteJointDef chainJoint;
       chainJoint.collideConnected = false;
       
       for(int32 i = 0; i < links; i++)
	 {
	   double cons = 2.0;
	   b2BodyDef bd;
	   bd.type = b2_dynamicBody;
	   bd.position.Set(i*cons + conveyorleft, shiftup + conveyorup);
	   b2Body* nextLink = m_world->CreateBody(&bd);
	   nextLink->CreateFixture(&chainfix);
	   b2Vec2 anchor(i*cons - 1.0 + conveyorleft, shiftup + conveyorup);
	   if(i > 0)
	     {
	       chainJoint.Initialize(lastLink, nextLink, anchor);
	       m_world->CreateJoint(&chainJoint);
	     }
	   lastLink = nextLink;
	   rightup = nextLink;//"rightup" is updated to the right most link in the top part of the chain in every iteration
	   if(i == 0)
	     {
	       leftup = nextLink;//"leftup" stores information about the topleft link
	       //More Precisely, it stores a link to the body of the upper left link
	     }
       }
       }
       
       //Now, we make the two links on the left hand side which connect the upper and lower halves of the chain
       {
	 b2BodyDef bd;
	 bd.type = b2_dynamicBody;//Note that this is a dynamic body
	 //Position
	 bd.position.Set(-1.0 + conveyorleft, conveyorup + 1.0);
	 b2Body* ground = m_world->CreateBody(&bd);
	 b2PolygonShape chainpiece;
	 //Dimensions. Note that the dimensions are reversed, so that this is a vertical link, while the earlier one was a horizontal link
	 chainpiece.SetAsBox(0.2, 1, b2Vec2(0.0,0.0),0.0);
	 
	 //Again defining a fixture for the chain
	 b2FixtureDef chainfix;
	 //Chain Properties
	 chainfix.density = 50;
	 chainfix.friction = 0.5;
	 chainfix.restitution = 0.0;
	 chainfix.shape = &chainpiece;
	 //Creating the first vertical link on the left
	 ground->CreateFixture(&chainfix);
	 
	 bd.position.Set(-1.0 + conveyorleft, conveyorup + 3.0);
	 //Creating the second vertical link on the left
	 b2Body*groundnew = m_world->CreateBody(&bd);
	 groundnew->CreateFixture(&chainfix);
	 
	 //defining the revolute joint to connect links up
	 b2RevoluteJointDef chainJoint;
	 chainJoint.collideConnected = false;
	 b2Vec2 anchor( conveyorleft - 1.0, conveyorup);
	 //Connecting the lower left horizontal link and the lower vertical link
	 chainJoint.Initialize(leftdown, ground, anchor);
	 m_world->CreateJoint(&chainJoint);
	 
	 b2RevoluteJointDef chainJoint2;
	 chainJoint2.collideConnected = false;
	 b2Vec2 anchor2( conveyorleft - 1.0, 2.0 + conveyorup);
	 //Connecting the lower and upper vertical links
	 chainJoint2.Initialize(ground, groundnew, anchor2);
	 m_world->CreateJoint(&chainJoint2);
	 
	 b2RevoluteJointDef chainJoint3;
	 chainJoint3.collideConnected = false;
	 b2Vec2 anchor3( conveyorleft - 1.0, 4.0 + conveyorup);
	 //Connecting the upper vertical link and the top left horizontal link
	 chainJoint3.Initialize(groundnew, leftup, anchor3);
	 m_world->CreateJoint(&chainJoint3);
       }
       
       //Now again we connect the top and bottom portions of the conveyor belt, this time at the right end.
       {
       b2BodyDef bd;
       bd.type = b2_dynamicBody;
       //Note that the position is shifted towards the rights by a number proportional to the number of links in the chain
       bd.position.Set(2.0 * links - 1.0+ conveyorleft, conveyorup + 1.0);
       b2Body* ground = m_world->CreateBody(&bd);
       b2PolygonShape chainpiece;
       chainpiece.SetAsBox(0.2, 1, b2Vec2(0.0,0.0),0.0);
       //Defining the links and setting properties
       b2FixtureDef chainfix;
       chainfix.density = 50;
       chainfix.friction = 0.5;
       chainfix.restitution = 0.0;
       chainfix.shape = &chainpiece;
       //Making the lower vertical RHS link
       ground->CreateFixture(&chainfix);
     //Making the upper vertical RHS link
       bd.position.Set(2.0* links - 1.0+ conveyorleft, conveyorup + 3.0);
       b2Body*groundnew = m_world->CreateBody(&bd);
       groundnew->CreateFixture(&chainfix);
	//Defining the revolute joint
       b2RevoluteJointDef chainJoint;
       chainJoint.collideConnected = false;
       b2Vec2 anchor( 2.0 * links - 1.0 + conveyorleft, 0.0 + conveyorup);
       //Joining the right most link of the lower chain and the lower vertical link on the right
       chainJoint.Initialize(rightdown, ground, anchor);
       m_world->CreateJoint(&chainJoint);
       
       b2RevoluteJointDef chainJoint2;
       chainJoint2.collideConnected = false;
       b2Vec2 anchor2( 2.0 * links - 1.0+ conveyorleft, 2.0 + conveyorup);
       //Joining the lower and upper right vertical links
       chainJoint2.Initialize(ground, groundnew, anchor2);
       m_world->CreateJoint(&chainJoint2);
       
       b2RevoluteJointDef chainJoint3;
       chainJoint3.collideConnected = false;
       b2Vec2 anchor3( 2.0 * links - 1.0+ conveyorleft, 4.0 + conveyorup);
       //joining the upper right vertical link with the top right horizontal link
       chainJoint3.Initialize(groundnew, rightup, anchor3);
       m_world->CreateJoint(&chainJoint3);
       }
       //Now, we create the rotating high friction discs
       //
       //The parameters are similar for all the discs, except that they are all translated along the x axis
//
//
//
//The left most Disc:
       {
	 b2BodyDef bd;
   bd.type = b2_kinematicBody;//This disc is a kinematic body
   bd.position.Set(conveyorleft + 0.0, conveyorup + 2.0);//Position
   bd.angle = 0;//starts rotating at angle 0
  bodydiscl = m_world->CreateBody(&bd);
   b2CircleShape circle;//shape of body = circular
   circle.m_radius = 2.0;//radius of disc used (same as the rectangular length of the boxes)
   
   b2FixtureDef fd;
   //properties of the disc
   fd.shape = &circle;
   fd.density = 100;
   fd.friction = 1;//Note the friction. It is set as 1!
   bodydiscl->CreateFixture(&fd);
   bodydiscl->SetAngularVelocity(speed);
   
       }
       
       //The last disc (right most disc)
       {
	 b2BodyDef bd;
	 bd.type = b2_kinematicBody;
	 //Note the shift in position from the earlier disc
	 bd.position.Set(links * 2.0 + conveyorleft + -2.0, conveyorup + 2.0);
	 bd.angle = 0;
	 bodydiscr = m_world->CreateBody(&bd);
	 b2CircleShape circle;
	 circle.m_radius = 2.0;
	 
	 b2FixtureDef fd;
	 fd.shape = &circle;
	 fd.density = 100;
	 fd.friction = 1;
	 bodydiscr->CreateFixture(&fd);
	 bodydiscr->SetAngularVelocity(speed);
       }
       
       //The second disc from the left hand side
       {
	 b2BodyDef bd;
	 bd.type = b2_kinematicBody;
	  bd.position.Set(links/4 * 2.0 + 1.0 + conveyorleft + -2.0, conveyorup + 2.0);
	 bd.angle = 0;
	 body1 = m_world->CreateBody(&bd);
	 b2CircleShape circle;
	 circle.m_radius = 2.0;
	 
	 b2FixtureDef fd;
	 fd.shape = &circle;
	 fd.density = 100;
	 fd.friction = 1;
	 body1->CreateFixture(&fd);
	 body1->SetAngularVelocity(speed);
       }
       
       //The third disc from the left hand side
       {
	 b2BodyDef bd;
	 bd.type = b2_kinematicBody;
	 //Note the position
	 bd.position.Set(links/2 * 2.0 + 1.0 + conveyorleft + -2.0, conveyorup + 2.0);
	 bd.angle = 0;
	 body2 = m_world->CreateBody(&bd);
	 b2CircleShape circle;
	 circle.m_radius = 2.0;
	 
	 b2FixtureDef fd;
	 fd.shape = &circle;
	 fd.density = 100;
	 fd.friction = 1;
	 body2->CreateFixture(&fd);
	 body2->SetAngularVelocity(speed);
       }
       
       //The fourth disc from the left hand side
{
  b2BodyDef bd;
  bd.type = b2_kinematicBody;
  //Note the position
  bd.position.Set(3 * links/4 * 2.0 + 1.0 + conveyorleft + -2.0, conveyorup + 2.0);
  bd.angle = 0;
  body3 = m_world->CreateBody(&bd);
  b2CircleShape circle;
  circle.m_radius = 2.0;
  
  b2FixtureDef fd;
  fd.shape = &circle;
  fd.density = 100;
  fd.friction = 1;
  body3->CreateFixture(&fd);
  body3->SetAngularVelocity(speed);
 }


  }
      // modification to hydraulics:
  {


       {
        

        b2Body* b1; 
        b2EdgeShape shape; 
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
       
          shape.Set(b2Vec2(6.3,23.0), b2Vec2(9.0, 23.0));
          b1->CreateFixture(&shape, 0.0f); 
      
        }
  }

  // the modification to hydraulics

{


       {
        

        b2Body* b1; 
        b2EdgeShape shape; 
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
       
          shape.Set(b2Vec2(9.0,23.0), b2Vec2(9.0, 17.0));
          b1->CreateFixture(&shape, 0.0f); 
      
        }
  }
// dominos near hydraulics
  {

  {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
  
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
    
      for (int i = 0; i < 12; ++i)
  {
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(7.0f + 1.0f * i, 30.0f);
    b2Body* body = m_world->CreateBody(&bd);
    body->CreateFixture(&fd);
  }
    }

       {
      b2PolygonShape shape;
      shape.SetAsBox(8.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);
  
      b2BodyDef bd;
      bd.position.Set(33.0f, 4.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
// the sphere after dominos
    {
      
      b2CircleShape circle;
      circle.m_radius = 1.0;
  
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.01f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(5.0f, 30.0f);
      sbody1 = m_world->CreateBody(&ballbd);
      sbody1->CreateFixture(&ballfd);
      sbody1->SetUserData(this);
    }
  }
  }
}

}
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
