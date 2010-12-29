#include "pongDemo.h"

//--------------------------------------------------------------
void pongDemo::setup()
{
	kinect.init();
	kinect.open();

	depthImage.allocate(kinect.width, kinect.height);

	nearThreshold = 50;
	farThreshold  = 180;
	
	ofSetFrameRate(60);

	angle = 15;
	kinect.setCameraTiltAngle(angle);
	
	//setup physics engine
	box2d.init();
	box2d.setGravity(0, 0);
	box2d.checkBounds(true);
	box2d.setFPS(60);
	ofxBox2dContactListener* listener = new pongContactListener(this);
	box2d.setContactListener(listener);
	
	//walls
	ofPoint topLeft(50, 50);
	ofPoint bottomLeft(50, 700);
	ofPoint topRight(950, 50);
	ofPoint bottomRight(950, 700);
	
	topWall.setWorld(box2d.getWorld());
	topWall.clear();
	topWall.addPoint(topRight);
	topWall.addPoint(topLeft);
	topWall.createShape();
	
	bottomWall.setWorld(box2d.getWorld());
	bottomWall.clear();
	bottomWall.addPoint(bottomLeft);
	bottomWall.addPoint(bottomRight);
	bottomWall.createShape();
	
	leftWallUpper.setWorld(box2d.getWorld());
	leftWallUpper.clear();
	leftWallUpper.addPoint(topLeft);
	leftWallUpper.addPoint(topLeft.x, (bottomLeft.y - topLeft.y)/2 - 100);
	leftWallUpper.createShape();
	
	leftWallLower.setWorld(box2d.getWorld());
	leftWallLower.clear();
	leftWallLower.addPoint(topLeft.x, (bottomLeft.y - topLeft.y)/2 + 100);
	leftWallLower.addPoint(bottomLeft);
	leftWallLower.createShape();
	
	rightWallUpper.setWorld(box2d.getWorld());
	rightWallUpper.clear();
	rightWallUpper.addPoint(topRight.x, (bottomRight.y - topRight.y)/2 - 100);
	rightWallUpper.addPoint(topRight);
	rightWallUpper.createShape();
	
	rightWallLower.setWorld(box2d.getWorld());
	rightWallLower.clear();
	rightWallLower.addPoint(bottomRight);
	rightWallLower.addPoint(topRight.x, (bottomRight.y - topRight.y)/2 + 100);
	rightWallLower.createShape();
	
	//strings for goal object IDs
	strLeftGoal = "leftGoal";
	strRightGoal = "rightGoal";
	
	//goals
	rightGoal.setWorld(box2d.getWorld());
	rightGoal.clear();
	rightGoal.addPoint(topRight.x, (bottomRight.y - topRight.y)/2 + 100);
	rightGoal.addPoint(topRight.x, (bottomRight.y - topRight.y)/2 - 100);
	rightGoal.createShape();
	rightGoal.body->SetUserData(strRightGoal);
	
	leftGoal.setWorld(box2d.getWorld());
	leftGoal.clear();
	leftGoal.addPoint(topLeft.x, (bottomLeft.y - topLeft.y)/2 - 100);
	leftGoal.addPoint(topLeft.x, (bottomLeft.y - topLeft.y)/2 + 100);
	leftGoal.createShape();
	leftGoal.body->SetUserData(strLeftGoal);
	
	//pucks
	puck1.setPhysics(1000.0, 1.0, 1.0);
	puck1.setup(box2d.getWorld(), 100, 375, 10,75,false);
	
	puck2.setPhysics(1000.0, 1.0, 1.0);
	puck2.setup(box2d.getWorld(), 850, 375, 10,75,false);
	
	//game setup
	ball.setPhysics(1.0, 1.0, 0.0);
	ball.setup(box2d.getWorld(), 500, 375, 20);
	
	player1score = 0;
	player2score = 0;
	
	newBall();
	
}

//--------------------------------------------------------------
void pongDemo::update()
{
	ofBackground(100, 100, 100);
	kinect.update();

	depthImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		
	unsigned char * pix = depthImage.getPixels();
	int numPixels = depthImage.getWidth() * depthImage.getHeight();

	//threshold between near and far
	for(int i = 0; i < numPixels; i++){
		if( pix[i] > nearThreshold && pix[i] < farThreshold )
		{
			pix[i] = 255;
		}
		else
		{
			pix[i] = 0;
		}
	}

	depthImage.flagImageChanged();

	//find the 4 largest contours in the image between 1000px and 20000px in area
    contourFinder.findContours(depthImage, 1000, 20000, 4, false);
	
	vector<ofxCvBlob> blobs = contourFinder.blobs;
	
	//search through the found contours and test if any look like they contain fingers
	for(int i=0;i<blobs.size();i++)
	{
		if(containsFingers(blobs[i]))
		{
			if(blobs[i].centroid.x > kinect.width/2)
			{
				//player 1 is facing the kinect on the left hand side so the x coord is on the right hand side of the kinect image
				p1y = (blobs[i].centroid.y)/kinect.height * 1500;
			}
			else
			{
				p2y = (blobs[i].centroid.y)/kinect.height * 1500;
			}

		}
	}
		
	//update physics engine state
	box2d.update();	
	
	//move the pucks to the location of the player hands
	puck1.moveTo(100, p1y);
	puck2.moveTo(850, p2y);
	
}

//--------------------------------------------------------------
void pongDemo::draw()
{
	//draw contour blobs
	contourFinder.draw(250, 320, 400, 300);

	ofSetColor(200, 200, 200);
	
	char reportStr[1024];
	sprintf(reportStr, "\nset near threshold %i (press: + -)\nset far threshold %i (press: < >) blobs found %i", nearThreshold, farThreshold, contourFinder.nBlobs);
	ofDrawBitmapString(reportStr, 20, 700);
	
	//draw our scene
	bottomWall.draw();
	topWall.draw();
	leftWallLower.draw();
	leftWallUpper.draw();
	rightWallLower.draw();
	rightWallUpper.draw();
	puck1.draw();
	puck2.draw();
	ball.draw();
	
	//draw scores
	ofDrawBitmapString("Player 1 Score: " + ofToString(player1score),50,30);
	ofDrawBitmapString("Player 2 Score: " + ofToString(player2score),800,30);
	
	box2d.draw();
}

//--------------------------------------------------------------
void pongDemo::exit(){
	kinect.close();
}

//--------------------------------------------------------------
void pongDemo::keyPressed (int key)
{
	switch (key)
	{
		//thresholding control from the ofxKinect sample	
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
		case '<':		
		case ',':		
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
		case '-':		
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;

		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case 'n':
			newBall();
			break;
	}
}

void pongDemo::newBall()
{
	//update state of engine
	box2d.update();
	
	//pick random compass direction for new ball start
	float sgn = ofRandom(-1, 1);
	float vx = copysign(10,sgn);
	sgn = ofRandom(-1, 1);
	float vy = copysign(10,sgn);
	
	//start new ball in the middle
	ball.moveTo(500,375);
	ball.setVelocity(vx, vy);
}

//--------------------------------------------------------------
void pongDemo::mouseMoved(int x, int y)
{}

//--------------------------------------------------------------
void pongDemo::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void pongDemo::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void pongDemo::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void pongDemo::windowResized(int w, int h)
{}

bool pongDemo::containsFingers(ofxCvBlob contour, int k, float angleThreshold)
{
	ofxVec2f	v, u;
	ofxVec3f	v1, u1;	
	
	for(int i=k; i<contour.nPts-k; i++)
	{
		//calculate k-curvature to find peaks and valleys in the contour (likely fingers)
		//calculating angle between vectors with one end at point i and the other end k points infront and behind
		v.set(contour.pts[i].x - contour.pts[i-k].x, contour.pts[i].y - contour.pts[i-k].y);
		u.set(contour.pts[i].x - contour.pts[i+k].x, contour.pts[i].y - contour.pts[i+k].y);
		
		//fill 3-vectors so we can calculate cross product
		v1.set(contour.pts[i].x - contour.pts[i-k].x, contour.pts[i].y - contour.pts[i-k].y, 0.0);
		u1.set(contour.pts[i].x - contour.pts[i+k].x, contour.pts[i].y - contour.pts[i+k].y, 0.0);
		
		v.normalize(); u.normalize();
		
		//if the angle is nice and acute, this extrema might be a fingertip
		if(fabs(v.angle(u)) < angleThreshold)
		{	
			//if the cross product is positive this is a peak (fingertip), not a valley between fingers.			
			if(v1.cross(u1).z > 0){
				return true;
			}
		}
	}

	return false;	
}
