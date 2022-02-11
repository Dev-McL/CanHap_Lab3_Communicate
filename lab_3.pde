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


Gas gas;
Water water;
Solid solid;


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
float             worldWidth                          = 35.0;
float             worldHeight                         = 25.0;

float             edgeTopLeftX                        = 0.0;
float             edgeTopLeftY                        = 0.0;
float             edgeBottomRightX                    = worldWidth;
float             edgeBottomRightY                    = worldHeight;
float             edgeBottomLeftY                    = worldHeight;

float             gravityAcceleration                 = 15;//980; //cm/s2

String curState = "n"; // 0 = solid, 1 = liquid, 2 = gas
//int curStateLast;
/* Initialization of virtual tool */
HVirtualCoupling  s;


/* define world shapes & attributes */
FCircle           b1, b2, b3; //word changing buttons
//FCircle           c1, c2, c3, c4, c5, c6, c7, c8, c9; // solid circles
FBody gCorners[] = new FBody[4];
FBox              left, right, bottom, top1, top2;
FBox              l, r, b, t1, t2;
FBody[] particles = new FBody[16];
FBody[] solids    = new FBody[9];

/* define game start */
boolean           gameStart                           = true;

/* text font */
PFont             f;

/* end elements definition *********************************************************************************************/

/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */

  /* screen size definition */
  size(1400, 1000);

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

  ///* State 1 button */
  //b1                  = new FCircle(2.0); // diameter is 2
  //b1.setPosition(edgeTopLeftX+2.5, edgeTopLeftY+worldHeight/3);
  //b1.setFill(0, 255, 0);
  //b1.setStaticBody(true);
  //world.add(b1);


  left = new FBox(0.25, 9);
  left.setFill(0, 0, 0);
  left.setStaticBody(true);


  right = new FBox(0.25, 9);
  right.setFill(0, 0, 0);
  right.setStaticBody(true);


  bottom = new FBox(8, 0.25);
  bottom.setFill(0, 0, 0);
  bottom.setStaticBody(true);

  right.setPosition(16.8, 10.9);
  left.setPosition(8.75, 10.9);
  bottom.setPosition(12.75, 16.3);
  top1 = new FBox(4, 0.25);
  top1.setFill(0, 0, 0);
  top1.setStaticBody(true);
  top1.setPosition(10, 6);

  top2 = new FBox(4, 0.25);
  top2.setFill(0, 0, 0);
  top2.setStaticBody(true);
  top2.setPosition(15.2, 6);
  
  
  
  l = new FBox(0.25, 9);
  l.setFill(0, 0, 0);
  l.setStaticBody(true);


  r = new FBox(0.25, 9);
  r.setFill(0, 0, 0);
  r.setStaticBody(true);


  b = new FBox(8, 0.25);
  b.setFill(0, 0, 0);
  b.setStaticBody(true);

  t1 = new FBox(4, 0.25);
  t1.setFill(0, 0, 0);
  t1.setStaticBody(true);

  t2 = new FBox(4, 0.25);
  t2.setFill(0, 0, 0);
  t2.setStaticBody(true);

  
  water = new Water();
  solid = new Solid();
  gas = new Gas();
  gas.drawGas();
  solid.drawSolid();
  water.drawWater();
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75));
  s.h_avatar.setDensity(4);
  s.h_avatar.setFill(0, 0, 200);
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
void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if (renderingForce == false) {
    background(255,204,153);
    textFont(f, 22);

    if (gameStart) {
      textAlign(CENTER);
      text("Can you guess the state of matter?", width/2, 60);// Touch the coloured circles to change the word", width/2, 60);
      text("State 1", 250, 700, 90);
      text("State 2", 600, 700, 90);
      text("State 3", 1100, 700, 90);

    } else {

      //fill(128, 128, 128);
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
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

    renderingForce = true;
    println(curState);

    if (haplyBoard.data_available()) {
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


    if (isTouchingMatter(particles)) {
      s.h_avatar.setSensor(false);
      s.h_avatar.setDamping(500);
    } else {
      s.h_avatar.setDamping(100);
    }
    
     if (isTouchingMatter(solids)) {
      s.h_avatar.setSensor(false);
    } 

      if ((s.getAvatarPositionX() > 25) && (s.getAvatarPositionY() < 7)) {
        fEE.set(-2, 0, 0);
      }
      if (s.getAvatarPositionX() > 30) {
        fEE.set(-2, 2, 0);
      }
      if ((s.getAvatarPositionX() > 25) && (s.getAvatarPositionY() > 10)) {
        fEE.set(2, 2, 0);
      }

      if ((s.getAvatarPositionX() < 25) && (s.getAvatarPositionY() > 9.5)) {
        fEE.set(-1, -1, 0);
      }

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();

    world.step(1.0f/1000.0f);

    renderingForce = false;
  }
}

class Water {
  void drawWater() {
    // this function uses concepts from fisica example: anchors found here: http://www.ricardmarxer.com/fisica/examples/Anchors/applet/index.html

    for (int i=0; i < particles.length; i++) {
      particles[i] = new FCircle(1.7);
      particles[i].setNoStroke();
      particles[i].setFriction(5);
      particles[i].setHaptic(true);
      particles[i].setFill(120, 200, 190);
      //particles[i].setStatic(true);
      world.add(particles[i]);
    }

    particles[0].setPosition(12, 6.5);
    particles[1].setPosition(13.5, 6.5);
    particles[2].setPosition(16.5, 6.5);
    particles[3].setPosition(18.5, 6.5);

    particles[4].setPosition(12, 9);
    particles[5].setPosition(13.5, 9);
    particles[6].setPosition(16.5, 9);
    particles[7].setPosition(18.5, 9);


    particles[8].setPosition(12, 11);
    particles[9].setPosition(13.5, 11);
    particles[10].setPosition(16.5, 11);
    particles[11].setPosition(18.5, 11);

    particles[12].setPosition(12, 15);
    particles[13].setPosition(13.5, 15);
    particles[14].setPosition(16.5, 15);
    particles[15].setPosition(18.5, 15);

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
    right.setPosition(19.7, 10.9);
    left.setPosition(11.5, 10.9);
    bottom.setPosition(15.7, 15.6);
    top1.setPosition(13.2, 6);
    top2.setPosition(18.1, 6);
    world.add(left);
    world.add(right);
    world.add(bottom);
    world.add(top1);
    world.add(top2);
  }
}

void createJoint(int i, int j, FBody[] particles) {

  FDistanceJoint jt = new FDistanceJoint(particles[i], particles[j]);
  jt.setFrequency(2);
  jt.setDamping(24);
  jt.setFill(0);
  jt.setLength(4);
  world.add(jt);
}


boolean isTouchingMatter(FBody[] m) {

  for (int i = 0; i < m.length; i++) {
    if (s.h_avatar.isTouchingBody(m[i])) {
      return true;
    }
  }
  return false;
}

boolean isTouchingWater() {

  for (int i = 0; i < particles.length; i++) {
    if (s.h_avatar.isTouchingBody(particles[i])) {
      return true;
    }
  }
  return false;
}



class Solid {

  void drawSolid() {
    // this function uses concepts from fisica example: anchors found here: http://www.ricardmarxer.com/fisica/examples/Anchors/applet/index.html

    for (int i=0; i < solids.length; i++) {
      solids[i] = new FCircle(2);
      solids[i].setNoStroke();
      solids[i].setFriction(5);
      solids[i].setHaptic(true);
      solids[i].setFill(120, 120, 120);
      solids[i].setStatic(true);
      solids[i].setDensity(0.2);
      world.add(solids[i]);
    }

    solids[0].setPosition(5, 8);
    solids[1].setPosition(6.5, 8);
    solids[2].setPosition(8, 8);

    solids[3].setPosition(5, 10.5);
    solids[4].setPosition(6.5, 10.5);
    solids[5].setPosition(8, 10.5);

    solids[6].setPosition(5, 12.5);
    solids[7].setPosition(6.5, 12.5);
    solids[8].setPosition(8, 12.5);

    r.setPosition(9.9, 10.9);
    l.setPosition(3.1, 10.9);
    b.setPosition(6.5, 15.6);
    t1.setPosition(5,3);
    t2.setPosition(7,3);

    world.add(l);
    world.add(r);
    world.add(b);

  }
}


class Gas {

  void drawGas() {

    for (int i = 0; i < gCorners.length; i++) {
      gCorners[i] = new FCircle(2);
      gCorners[i].setNoStroke();
      //solids[i].setFriction(5);
      //solids[i].setHaptic(true);
      gCorners[i].setFill(120, 120, 120);
      gCorners[i].setStatic(true);
      //solids[i].setDensity(0.2);
      world.add(gCorners[i]);
    }
    gCorners[0].setPosition(24, 7);
    gCorners[1].setPosition(30, 7);
    gCorners[2].setPosition(24, 14);
    gCorners[3].setPosition(30, 14);

  }
}
