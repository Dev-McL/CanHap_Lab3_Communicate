/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 


/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/


/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 

PVector           fContact                            = new PVector(0, 0);
PVector           fDamping                            = new PVector(0, 0);
/* end effector radius in meters */
float             rEE                                 = 0.006;
float             rEEContact                          = 0.006;




/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 25.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;
float             edgeBottomLeftY                    = worldHeight; 

float             gravityAcceleration                 = 15;//980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;


/* define world shapes & attributes */
FCircle           b1, b2, b3; //word changing buttons
//FCircle           c1, c2, c3, c4, c5, c6, c7, c8, c9; // solid circles
FBox              word_bx, left, right, bottom, top1 , top2;
FBody[] particles = new FBody[16];
FBody[] solids    = new FBody[9];

/* define game start */
boolean           gameStart                           = false;

/* text font */
PFont             f;
int rotations = 0;

PShape pGraph, joint, endEffector;
PShape ball, ball2, ball3, ball4, ball5, ball6, ball7, ball8, ball9, leftWall, bottomWall, rightWall;

/* end elements definition *********************************************************************************************/  

/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 1000);
  
  /* set font type and size */
  f                   = createFont("Arial", 16, true);

  /* device setup */
  
  haplyBoard          = new Board(this, Serial.list()[1], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  /* State 1 button */
  b1                  = new FCircle(2.0); // diameter is 2
  b1.setPosition(edgeTopLeftX+2.5, edgeTopLeftY+worldHeight/3);
  b1.setFill(0, 255, 0);
  b1.setStaticBody(true);
  world.add(b1);
  
  /* State 2 button */
  b2                  = new FCircle(2.0);
  b2.setPosition(edgeTopLeftX+2.5, edgeTopLeftY+worldHeight/2.0);
  b2.setFill(200,0,0);
  b2.setStaticBody(true);
  //b2.setSensor(true);
  world.add(b2);
 
   /* State 3 button */
  b3                  = new FCircle(2.0);
  b3.setPosition(edgeTopLeftX+2.5, edgeBottomRightY-worldHeight/3);
  b3.setFill(120,0,100);
  b3.setStaticBody(true);
  //b3.setSensor(true);
  world.add(b3);
  
  /* Box to contain the 'words' */
  
  word_bx = new FBox(15,15);
  word_bx.setNoFill();
  word_bx.setStroke(0,0,0);
  word_bx.setStrokeWeight(2);
  word_bx.setStaticBody(true);
  word_bx.setPosition(worldWidth/2, worldHeight/2);
  //world.add(word_bx);
  
  
  
  
 FLine top = new FLine(7.5, 7.5, 17.5, 7.5);
 //world.add(top);
 
  left = new FBox(0.25,9);
  left.setFill(0,0,0);
  left.setStaticBody(true);
  
 
  right = new FBox(0.25,9);
  right.setFill(0,0,0);
  right.setStaticBody(true);

  
  bottom = new FBox(8,0.25);
  bottom.setFill(0,0,0);
  bottom.setStaticBody(true);
  
    right.setPosition(16.8, 10.9);
  left.setPosition(8.75, 10.9);
  bottom.setPosition(12.75, 16.3);
  top1 = new FBox(4,0.25);
  top1.setFill(0,0,0);
  top1.setStaticBody(true);
  top1.setPosition(10, 6);
  
  top2 = new FBox(4,0.25);
  top2.setFill(0,0,0);
  top2.setStaticBody(true);
  top2.setPosition(15.2, 6);

  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(0,0,200); 
  s.h_avatar.setSensor(true);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
 
  world.draw();
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/


/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    textFont(f, 22);
 
    if(gameStart){
      text("Can you guess the state of matter? Touch the coloured circles to change the word", width/2, 60);
      text("State 1", 105, 285, 90);
      text("State 2", 105, 450, 90);
      text("State 3", 105, 620, 90);

    }
    else{
      fill(128, 128, 128);
      textAlign(CENTER);
      text("Can you guess the state of matter? Touch the coloured circles to change the word", width/2, 60);
      text("State 1", 105, 285, 90);
      text("State 2", 105, 450, 90);
      text("State 3", 105, 620, 90);
    
    }
  
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/


/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();
 
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
 

    
    if (s.h_avatar.isTouchingBody(b1)){
      gameStart = true;
      world.remove(b1);
      s.h_avatar.setSensor(false);
      drawSolid();
    }
    
    if (isTouchingWater()) {
      s.h_avatar.setSensor(false);
      s.h_avatar.setDamping(500);
    }
    else {
      s.h_avatar.setDamping(100);
      
    }
      
    if(s.h_avatar.isTouchingBody(b2)){
      world.remove(b2);
      removeLattice(solids);
      s.h_avatar.setSensor(false);
      drawWater(); 
    }
    
    if (s.h_avatar.isTouchingBody(b3)) {
      world.remove(b3);
      removeLattice(particles);
      
              /* forces due to damping */
        fDamping.set(posEE.x*-10, posEE.y*-10);
        /* end forces due to damping*/
        if (rotations < 2000) {
          if (posEE.x > 0) {
            rEEContact = rEE;
            fContact.set(-2, 0);
            fEE = fContact.copy().add(fDamping);
          }
          if (posEE.x > 0.05) {
            rEEContact = rEE;
            fContact.set(-2, 2);
            fEE = fContact.copy();
          }
          if (posEE.x > 0 && posEE.y > 0.1) {
            rEEContact = rEE;
            fContact.set(2, 2);
            fEE = fContact.copy();
          }
          if (posEE.x < 0.05 && posEE.y > 0.08) {
            // update rotations
            rotations = rotations + 1;
            rEEContact = rEE;
            fContact.set(-1, -1);
            fEE = fContact.copy();
          }
          if (posEE.x < 0 || posEE.y < 0.03) {
            rEEContact = rEE;
            fContact.set(0, 0);
            fEE = fContact.copy().add(fDamping);
          }
        }
        else if (rotations < 9000) {
          //println(posEE);
          if (posEE.x > -0.05 && posEE.y > 0.04) {
            rEEContact = rEE;
            fContact.set(4, 0);
            fEE = fContact.copy();
          }
          if (posEE.x < -0.05 || posEE.y < 0.03) {
            rEEContact = rEE;
            fContact.set(0, 0);
            fEE = fContact.copy().add(fDamping);
            // reset
            rotations = 0;
          }
        }
        else {
          // reset
          rotations = 0;
        }
       
      }
      
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}

void drawWater() {
  // this function uses concepts from fisica example: anchors found here: http://www.ricardmarxer.com/fisica/examples/Anchors/applet/index.html
  
    for (int i=0; i < particles.length ; i++) {
      particles[i] = new FCircle(1.7);
      particles[i].setNoStroke();
      particles[i].setFriction(5);
      particles[i].setHaptic(true);
      particles[i].setFill(120, 200, 190);
      //particles[i].setStatic(true);
      world.add(particles[i]);   
    }
    
   particles[0].setPosition(9.5, 6.5); 
   particles[1].setPosition(11, 6.5);
   particles[2].setPosition(14, 6.5);
   particles[3].setPosition(16, 6.5);
   
   particles[4].setPosition(9.5, 9); 
   particles[5].setPosition(11,9);
   particles[6].setPosition(14, 9);
   particles[7].setPosition(16, 9);
   
   
   particles[8].setPosition(9.5, 11); 
   particles[9].setPosition(11, 11);
   particles[10].setPosition(14,11);
   particles[11].setPosition(16, 11);
   
   particles[12].setPosition(9.5, 16); 
   particles[13].setPosition(11, 16);
   particles[14].setPosition(14, 16);
   particles[15].setPosition(16, 16);
   
     for (int i = 1; i < 4; i++) {
       createJoint(i-1, i, particles);
       createJoint(i-1, i+3, particles); 
     }
     
     for (int i = 5; i < 8; i++) {
       createJoint(i-1, i, particles);
       createJoint(i-1, i+3, particles); 
     }
      
    for (int i = 9; i < 12; i++) {
       createJoint(i-1, i, particles);
       createJoint(i-1, i+3, particles);  
  
     }
     
    for (int i = 13; i < 16; i++) {
       createJoint(i-1, i, particles);
     }
   
   createJoint(3, 7, particles);
   createJoint(7, 11, particles);
   createJoint(11, 15, particles);
     right.setPosition(16.8, 10.9);
  left.setPosition(8.75, 10.9);
  bottom.setPosition(12.75, 16.3);
   world.add(left);
   world.add(right);
   world.add(bottom);
   world.add(top1);
   world.add(top2);

  }
  
  void createJoint(int i, int j, FBody[] particles) {
    
     FDistanceJoint jt = new FDistanceJoint(particles[i], particles[j]);
     jt.setFrequency(2);
     jt.setDamping(24);
     jt.setFill(0);
     jt.setLength(4);
     //jt.calculateLength();
     world.add(jt);
    
  }
  
  
  boolean isTouchingWater(){
    
    for (int i = 0; i < particles.length; i++) {
      if (s.h_avatar.isTouchingBody(particles[i])){
        return true; 
      }
    }
    return false;
       
  }
  
void drawSolid() {
  // this function uses concepts from fisica example: anchors found here: http://www.ricardmarxer.com/fisica/examples/Anchors/applet/index.html
  
    for (int i=0; i < solids.length ; i++) {
      solids[i] = new FCircle(2);
      solids[i].setNoStroke();
      solids[i].setFriction(5);
      solids[i].setHaptic(true);
      solids[i].setFill(120, 120, 120);
      solids[i].setStatic(true);
      world.add(solids[i]);   
    }
    
   solids[0].setPosition(11, 8); 
   solids[1].setPosition(12.5, 8);
   solids[2].setPosition(14, 8);
   
   solids[3].setPosition(11, 10.5);
   solids[4].setPosition(12.5, 10.5); 
   solids[5].setPosition(14, 10.5);
   
   solids[6].setPosition(11, 12.5);
   solids[7].setPosition(12.5, 12.5);
   solids[8].setPosition(14, 12.5); 
   
   right.setPosition(15.3, 10.9);
  left.setPosition(9.25, 10.9);
  bottom.setPosition(12.4, 16);
   
   world.add(left);
   world.add(right);
   world.add(bottom);
   world.add(top1);
   world.add(top2);

  }
  
  void removeLattice(FBody[] l) {
    
    for (int i = 0; i < l.length; i++) {
      world.remove(l[i]);
    }
    
    
  }
 
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
/* end simulation section **********************************************************************************************/

/* ********************************************************************************************************************************************************************************/
/* ********************************************************************************************************************************************************************************/
/* ********************************************************************************************************************************************************************************/
/* ********************************************************************************************************************************************************************************/
/* ********************************************************************************************************************************************************************************/
/* ********************************************************************************************************************************************************************************/
/* ********************************************************************************************************************************************************************************/
/* ********************************************************************************************************************************************************************************/
/* ********************************************************************************************************************************************************************************/
/* ********************************************************************************************************************************************************************************/

///**
// **********************************************************************************************************************
// * @file       sketch_3_Hello_Ball.pde
// * @author     Steve Ding, Colin Gallacher
// * @version    V1.0.0
// * @date       08-January-2021
// * @brief      Ball haptic example with programmed physics for a haptic ball enclosed in a box
// **********************************************************************************************************************
// * @attention
// *
// *
// **********************************************************************************************************************
// */
 
//  /* library imports *****************************************************************************************************/ 
//import processing.serial.*;
//import static java.util.concurrent.TimeUnit.*;
//import java.util.concurrent.*;
///* end library imports *************************************************************************************************/  


///* scheduler definition ************************************************************************************************/ 
//private final ScheduledExecutorService scheduler1      = Executors.newScheduledThreadPool(1);
///* end scheduler definition ********************************************************************************************/ 



///* device block definitions ********************************************************************************************/
//Board             haplyBoard1;
//Device            widgetOne1;
//Mechanisms        pantograph1;

//byte              widgetOneID1                         = 5;
//int               CW1                                 = 0;
//int               CCW1                                 = 1;
//boolean           renderingForce1                     = false;
///* end device block definition *****************************************************************************************/



///* framerate definition ************************************************************************************************/
//long              baseFrameRate1                       = 120;
///* end framerate definition ********************************************************************************************/ 



///* elements definition *************************************************************************************************/

///* Screen and world setup parameters */
//float             pixelsPerMeter                      = 4000.0;
//float             radsPerDegree                       = 0.01745;

///* pantagraph link parameters in meters */
//float             l                                   = 0.07; // m
//float             L                                   = 0.09; // m


///* end effector radius in meters */
//float             rEE                                 = 0.006;
//float             rEEContact                          = 0.006;


///* virtual ball parameters  */
//float             rBall                               = 0.015;

//float             mBall                               = 0.15;  // kg
//float             kBall                               = 600;  // N/m
//float             bBall                               = 3.7;
//float             penBall                             = 0.0;  // m
//float             bAir                                = 0.0;  // kg/s
//PVector           fGravity                            = new PVector(0, 9.8*mBall);
//float             dt                                  = 1/1000.0;

//PVector           posBall                             = new PVector(0, 0.05);  
//PVector           posBall2                            = new PVector(0, 0.075);
//PVector           posBall3                            = new PVector(0, 0.1);
//PVector           posBall4                            = new PVector(0.025, 0.05);
//PVector           posBall5                            = new PVector(0.025, 0.075);
//PVector           posBall6                            = new PVector(0.025, 0.1);
//PVector           posBall7                            = new PVector(0.05, 0.05);
//PVector           posBall8                            = new PVector(0.05, 0.075);
//PVector           posBall9                            = new PVector(0.05, 0.1);

//PVector           velBall                             = new PVector(0, 0); 

//PVector[]         solidBallPos                        = {posBall, posBall2, posBall3, posBall4, posBall5, posBall6, posBall7, posBall8, posBall9};

//PVector           fBall                               = new PVector(0 ,0);    
//PVector           fContact                            = new PVector(0, 0);
//PVector           fDamping                            = new PVector(0, 0);

//PVector           posEEToBall;
//float             posEEToBallMagnitude;

//PVector           velEEToBall;
//float             velEEToBallMagnitude;


///* virtual wall parameters */
//PVector           fWall                               = new PVector(0, 0);
//float             kWall                               = 800; // N/m
//float             bWall                               = 2; // kg/s
//PVector           penWall                             = new PVector(0, 0);

//PVector           posWallLeft                         = new PVector(-0.07, 0.03);
//PVector           posWallRight                        = new PVector(0.07, 0.03);
//PVector           posWallBottom                       = new PVector(0.0, 0.1);


///* generic data for a 2DOF device */
///* joint space */
//PVector           angles1                              = new PVector(0, 0);
//PVector           torques1                             = new PVector(0, 0);

///* task space */
//PVector           posEE1                               = new PVector(0, 0);
//PVector           posEELast                           = new PVector(0, 0);
//PVector           velEE                               = new PVector(0, 0);

//PVector           fEE1                                 = new PVector(0, 0); 

///* device graphical position */
//PVector           deviceOrigin                        = new PVector(0, 0);

///* World boundaries reference */
//final int         worldPixelWidth                     = 1000;
//final int         worldPixelHeight                    = 650;


///* graphical elements */
//PShape pGraph, joint, endEffector;
//PShape ball, ball2, ball3, ball4, ball5, ball6, ball7, ball8, ball9, leftWall, bottomWall, rightWall;


//// for gas
//int rotations = 0;

//String curState = "solid";
///* end elements definition *********************************************************************************************/ 


///* setup section *******************************************************************************************************/
//void setup1(){
//  /* put setup code here, run once: */
  
//  /* screen size definition */
//  //size(1000, 650);
  
//  /* device setup */
  
//  /**  
//   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
//   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
//   * to explicitly state the serial port will look like the following for different OS:
//   *
//   *      windows:      haplyBoard = new Board(this, "COM10", 0);
//   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
//   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
//   */ 
//  //haplyBoard1          = new Board(this, Serial.list()[1], 0);
//  //widgetOne1           = new Device(widgetOneID, haplyBoard);
//  //pantograph1          = new Pantograph();
  
//  //widgetOne1.set_mechanism(pantograph1);
  
//  //widgetOne1.add_actuator(1, CCW1, 2);
//  //widgetOne1.add_actuator(2, CW1, 1);
 
//  //widgetOne1.add_encoder(1, CCW1, 241, 10752, 2);
//  //widgetOne1.add_encoder(2, CW1, -61, 10752, 1);
  
//  //widgetOne1.device_set_parameters();
    
  
//  /* visual elements setup */
//  background(0);
//  deviceOrigin.add(worldPixelWidth/2, 0);
  
//  /* create pantagraph graphics */
//  create_pantagraph();
  
  
//  /* create balls for solid */
//  ball = create_ball(rBall);
//  ball.setStroke(color(0));
  
//  ball2 = create_ball(rBall);
//  ball2.setStroke(color(0));
  
//  ball3 = create_ball(rBall);
//  ball3.setStroke(color(0));
  
//  ball4 = create_ball(rBall);
//  ball4.setStroke(color(0));
  
//  ball5 = create_ball(rBall);
//  ball5.setStroke(color(0));
  
//  ball6 = create_ball(rBall);
//  ball6.setStroke(color(0));
  
//  ball7 = create_ball(rBall);
//  ball7.setStroke(color(0));
  
//  ball8 = create_ball(rBall);
//  ball8.setStroke(color(0));
  
//  ball9 = create_ball(rBall);
//  ball9.setStroke(color(0));
  

  
//  /* setup framerate speed */
//  frameRate(baseFrameRate1);
  
  
//  /* setup simulation thread to run at 1kHz */ 
//  SimulationThread1 st1 = new SimulationThread1();
//  scheduler.scheduleAtFixedRate(st1, 1, 1, MILLISECONDS);
//}
///* end setup section ***************************************************************************************************/




///* draw section ********************************************************************************************************/
//void draw1(){
//  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
//  if(renderingForce1 == false){
//    background(255);  
//    update_animation(angles1.x*radsPerDegree, angles1.y*radsPerDegree, posEE1.x, posEE1.y);
//  }
//}
///* end draw section ****************************************************************************************************/




///* simulation section **************************************************************************************************/
//class SimulationThread1 implements Runnable{
  
//  public void run(){
//    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
//    renderingForce1 = true;
//    println(curState);
    
//    if(haplyBoard1.data_available()){
//      /* GET END-EFFECTOR STATE (TASK SPACE) */
//      widgetOne1.device_read_data();
    
//      angles1.set(widgetOne1.get_device_angles()); 
//      posEE1.set(widgetOne1.get_device_position(angles1.array()));
//      posEE1.set(device_to_graphics(posEE1)); 
      
//      velEE.set((posEE1.copy().sub(posEELast)).div(dt));
//      posEELast = posEE1;
     
      
//      /* haptic physics force calculation */
//      PVector[] ballForces = new PVector[solidBallPos.length];
      
//      for (int i=0; i < solidBallPos.length; i++) {
//        PVector curBallPos = solidBallPos[i];
//              /* ball and end-effector contact forces */
//        posEEToBall = (curBallPos.copy()).sub(posEE1);
//        posEEToBallMagnitude = posEEToBall.mag();
      
//        penBall = posEEToBallMagnitude - (rBall + rEE);
//        /* end ball and end-effector contact forces */
      
//        /* ball forces */
//        if(penBall < 0){
//          rEEContact = rEE + penBall;
//          fContact = posEEToBall.normalize();
//          velEEToBall = velBall.copy().sub(velEE);
//          velEEToBall = fContact.copy().mult(velEEToBall.dot(fContact));
//          velEEToBallMagnitude = velEEToBall.mag();
//        }
//        else{
//          rEEContact = rEE;
//          fContact.set(0, 0);
//        }
//        /* since penBall is negative kBall must be negative to ensure the force acts along the end-effector to the ball */
//        fContact = fContact.mult((-kBall * penBall) - (bBall * velEEToBallMagnitude));
//        fEE1 = (fContact.copy()).mult(-1);
//        ballForces[i] = fEE1;
//        /* end ball forces */     
//      }
      
//      if (curState.equals("solid")) {
//              // make sure all ball forces are applied, even when touching multiple balls
//        PVector allForces = ballForces[0];
//        for (int i=1; i < ballForces.length; i++) {
//          allForces.add(ballForces[i]);
//        }
//        fEE1 = allForces.copy();
//        fEE1.set(graphics_to_device(fEE1));
//      }
//      else if (curState.equals("liquid")){
//        rEEContact = rEE;
//        fContact.set(0, 0);
//        fEE1 = fContact.copy();
//      }
//      else {
//        /* forces due to damping */
//        fDamping.set(posEE1.x*-10, posEE1.y*-10);
//        /* end forces due to damping*/
//        if (rotations < 2000) {
//          if (posEE1.x > 0) {
//            rEEContact = rEE;
//            fContact.set(-2, 0);
//            fEE1 = fContact.copy().add(fDamping);
//          }
//          if (posEE1.x > 0.05) {
//            rEEContact = rEE;
//            fContact.set(-2, 2);
//            fEE1 = fContact.copy();
//          }
//          if (posEE1.x > 0 && posEE1.y > 0.1) {
//            rEEContact = rEE;
//            fContact.set(2, 2);
//            fEE1 = fContact.copy();
//          }
//          if (posEE1.x < 0.05 && posEE1.y > 0.08) {
//            // update rotations
//            rotations = rotations + 1;
//            rEEContact = rEE;
//            fContact.set(-1, -1);
//            fEE1 = fContact.copy();
//          }
//          if (posEE1.x < 0 || posEE1.y < 0.03) {
//            rEEContact = rEE;
//            fContact.set(0, 0);
//            fEE1 = fContact.copy().add(fDamping);
//          }
//        }
//        else if (rotations < 9000) {
//          //println(posEE);
//          if (posEE1.x > -0.05 && posEE1.y > 0.04) {
//            rEEContact = rEE;
//            fContact.set(4, 0);
//            fEE1 = fContact.copy();
//          }
//          if (posEE1.x < -0.05 || posEE1.y < 0.03) {
//            rEEContact = rEE;
//            fContact.set(0, 0);
//            fEE1 = fContact.copy().add(fDamping);
//            // reset
//            rotations = 0;
//          }
//        }
//        else {
//          // reset
//          rotations = 0;
//        }
       
//      }


//        /* end haptic physics force calculation */
//    }
    
//    torques1.set(widgetOne1.set_device_torques(fEE1.array()));
//    widgetOne1.device_write_torques();
    
     
//    renderingForce1 = false;
//  }
//}
///* end simulation section **********************************************************************************************/




///* helper functions section, place helper functions here ***************************************************************/
//void create_pantagraph(){
//  float lAni = pixelsPerMeter * l;
//  float LAni = pixelsPerMeter * L;
//  float rEEAni = pixelsPerMeter * rEE;
  
//  pGraph = createShape();
//  pGraph.beginShape();
//  pGraph.fill(255);
//  pGraph.stroke(0);
//  pGraph.strokeWeight(2);
  
//  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
//  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
//  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
//  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
//  pGraph.endShape(CLOSE);
  
//  joint = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, rEEAni, rEEAni);
//  joint.setStroke(color(0));
  
//  endEffector = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, 2*rEEAni, 2*rEEAni);
//  endEffector.setStroke(color(0));
//  strokeWeight(5);
  
//}


//PShape create_wall(float x1, float y1, float x2, float y2){
//  x1 = pixelsPerMeter * x1;
//  y1 = pixelsPerMeter * y1;
//  x2 = pixelsPerMeter * x2;
//  y2 = pixelsPerMeter * y2;
  
//  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
//}


//PShape create_ball(float rBall){
//  rBall = pixelsPerMeter * rBall;
  
//  return createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, 2*rBall, 2*rBall);
//}


//void update_animation(float th1, float th2, float xE, float yE){
//  background(255);
  
//  float lAni = pixelsPerMeter * l;
//  float LAni = pixelsPerMeter * L;
  
//  xE = pixelsPerMeter * xE;
//  yE = pixelsPerMeter * yE;
  
//  th1 = 3.14 - th1;
//  th2 = 3.14 - th2;
  
//  pGraph.setVertex(1, deviceOrigin.x + lAni*cos(th1), deviceOrigin.y + lAni*sin(th1));
//  pGraph.setVertex(3, deviceOrigin.x + lAni*cos(th2), deviceOrigin.y + lAni*sin(th2));
//  pGraph.setVertex(2, deviceOrigin.x + xE, deviceOrigin.y + yE);
  
//    // text
//  // instructions
//    String s = "Can you identify the states of matter? Press any key to switch to a new state!";
//    fill(0);
//    textSize(20);
//    text(s, 40, 40, 280, 320);
//    if (curState.equals("solid")) {
//      String state = "State 1";
//      fill(0);
//      textSize(16);
//      text(state, 40, 150, 100, 100);
//    }
//    else if (curState.equals("liquid")) {
//      String state = "State 2";
//      fill(0);
//      textSize(16);
//      text(state, 40, 150, 100, 100);
//    }
//    else {
//      String state = "State 3";
//      fill(0);
//      textSize(16);
//      text(state, 40, 150, 100, 100);
//    }
  
//  //shape(pGraph);
//  //shape(joint);
  
//  shape(ball, posBall.x * pixelsPerMeter, posBall.y * pixelsPerMeter);
//  shape(ball2, posBall2.x * pixelsPerMeter, posBall2.y * pixelsPerMeter);
//  shape(ball3, posBall3.x * pixelsPerMeter, posBall3.y * pixelsPerMeter);
//  shape(ball4, posBall4.x * pixelsPerMeter, posBall4.y * pixelsPerMeter);
//  shape(ball5, posBall5.x * pixelsPerMeter, posBall5.y * pixelsPerMeter);
//  shape(ball6, posBall6.x * pixelsPerMeter, posBall6.y * pixelsPerMeter);
//  shape(ball7, posBall7.x * pixelsPerMeter, posBall7.y * pixelsPerMeter);
//  shape(ball8, posBall8.x * pixelsPerMeter, posBall8.y * pixelsPerMeter);
//  shape(ball9, posBall9.x * pixelsPerMeter, posBall9.y * pixelsPerMeter);
//  stroke(0);
  
  
//  translate(xE, yE);
//  shape(endEffector);
//}


//PVector device_to_graphics(PVector deviceFrame){
//  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
//}


//PVector graphics_to_device(PVector graphicsFrame){
//  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
//}

//// change state when any key pressed
//void keyPressed() {
//  if (curState.equals("solid")) {
//    curState = "liquid";
//  }
//  else if (curState.equals("liquid")) {
//    curState = "gas";
//  }
//  else {
//    curState = "solid";
//  }
//}
///* end helper function section *******************************************/ 
