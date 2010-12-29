#ifndef _TEST_APP
#define _TEST_APP

#include "ofMain.h"

#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxVectorMath.h"
#include "ofxBox2d.h"

class Ball : public ofxBox2dCircle {
	
public:
	Ball() {
	}
	ofColor colour;
	void draw() {
		float radius = getRadius();
		
		glPushMatrix();
		glTranslatef(getPosition().x, getPosition().y, 0);
		
		ofSetColor(colour.r, colour.g, colour.b);
		ofFill();
		ofCircle(0, 0, radius);	
		
		glPopMatrix();
		
	}
};

class Wall : public ofxBox2dLine {
	
public:
	Wall() {
	}
	ofColor colour;
	void draw() {
		ofBeginShape();
		ofSetColor(colour.r,colour.g,colour.b);
		ofNoFill();
		for(int i=0; i<points.size(); i++) {
			ofVertex(points[i].x, points[i].y);
		}
		ofEndShape();
	}
};

class Paddle : public ofxBox2dRect {
	
public:
	Paddle() {
	}
	ofColor colour;
	void draw() {
		
		b2Shape* s = body->GetShapeList();
		const b2XForm& xf = body->GetXForm();
		b2PolygonShape* poly = (b2PolygonShape*)s;
		int count = poly->GetVertexCount();
		const b2Vec2* localVertices = poly->GetVertices();
		b2Assert(count <= b2_maxPolygonVertices);
		b2Vec2 verts[b2_maxPolygonVertices];
		for(int32 i=0; i<count; ++i) {
			verts[i] = b2Mul(xf, localVertices[i]);
		}
		
		ofSetColor(colour.r, colour.g, colour.b);
		ofFill();
		ofBeginShape();
		for (int32 i = 0; i <count; i++) {
			ofVertex(verts[i].x*OFX_BOX2D_SCALE, verts[i].y*OFX_BOX2D_SCALE);
		}
		ofEndShape();
		ofEndShape(true);
	}
};

class pongDemo : public ofBaseApp
{
	public:

		void setup();
		void update();
		void draw();
		void exit();
	
		void keyPressed(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
	
		void newBall();
	
		bool containsFingers(ofxCvBlob contour, int k=35, float angleThreshold=40.0);

		ofxKinect kinect;

		ofxCvGrayscaleImage 	depthImage;

		ofxCvContourFinder 	contourFinder;

		int 				nearThreshold;
		int					farThreshold;

		int					angle;
		
		ofxBox2d						box2d;
		
		Ball ball;
	
		Wall					leftWallUpper;
		Wall					leftWallLower;
		Wall					rightWallUpper;
		Wall					rightWallLower;
		Wall					topWall;
		Wall					bottomWall;
	
		ofxBox2dLine					leftGoal;
		ofxBox2dLine					rightGoal;
			
		Paddle puck1;
		Paddle puck2;
	
		int player1score;
		int player2score;
	
		float p1y;
		float p2y;
	
		char* strLeftGoal;
		char* strRightGoal;
	
};

class pongContactListener : public ofxBox2dContactListener
{
public:
	pongDemo* game;
	pongContactListener(pongDemo* obj){
		game = obj;
	}
	
	void Add(const b2ContactPoint* point) {
		
		b2Vec2 p = point->position;
		p *= OFX_BOX2D_SCALE;
		
		b2Shape* shape1 = point->shape1;
		b2Shape* shape2 = point->shape2;
		
		//if the right goal was involved in a collision, player 1 got a goal
		if(shape1->GetBody()->GetUserData() == game->strRightGoal || shape2->GetBody()->GetUserData() == game->strRightGoal)
		{
			game->player1score++;
			game->newBall();
		}
		//if the left goal was involved in a collision it was player 2
		else if (shape1->GetBody()->GetUserData() == game->strLeftGoal || shape2->GetBody()->GetUserData() == game->strLeftGoal)
		{
			game->player2score++;
			game->newBall();
		}
		
	}	
};

#endif
