/**
 **********************************************************************************************************************
 * @file       sketch_4_Wall_Physics.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V4.1.0
 * @date       08-January-2021
 * @brief      wall haptic example using 2D physics engine 
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
boolean           renderingForce                      = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 

/* text font */
PFont             f;

/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

float             kWall                               = 200;
PVector           fWall                               = new PVector(0, 0);
PVector           penWall                             = new PVector(0, 0);
PVector           posWall                             = new PVector(0.01, 0.10);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           posEELast                           = new PVector(0, 0);
PVector           fEE                                = new PVector(0, 0); 

float             rEE                                 = 0.006;
/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 35.0;  
float             worldHeight                         = 22.5; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of wall */
FBox              wall, wallPortal12, wallPortal11, wallHidePortal1, wallPortal5, wall4, wallPortal4, wallPortal3, wallHidePortal5, wallPortal2, wall9,wallPortal120; 
FCircle           circle1,circle2;
FBlob             blob1;


/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar,pac2;

/* end elements definition *********************************************************************************************/ 

float threshold = 20;



/* Timer variables */
long currentMillis = millis();
long previousMillis = 0;
float interval = 8000;



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1400, 900);
  
  /* device setup */
  f                   = createFont("Arial", 30, true);
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */ 
  haplyBoard          = new Board(this, "COM3", 0);
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
  
  
  //angry
  wallPortal11                   = new FBox(2, worldHeight/2.5);
  wallPortal11.setPosition(1, 6);
  wallPortal11.setStatic(true);
  wallPortal11.setSensor(true);
  wallPortal11.setFill(0,0,0);
  wallPortal11.setNoStroke();
  world.add(wallPortal11);
  
  
  wallPortal12                   = new FBox(2, worldHeight/2.5);
  wallPortal12.setPosition(6, 6);
  wallPortal12.setStatic(true);
  wallPortal12.setSensor(true);
  wallPortal12.setFill(0,0,0);
  wallPortal12.setNoStroke();
  world.add(wallPortal12);
  
  //hide wall for angry  
  //wallHidePortal1                   = new FBox(14, worldHeight/2);
  //wallHidePortal1.setPosition(1, 6);
  //wallHidePortal1.setStatic(true);
  //wallHidePortal1.setSensor(true);
  //wallHidePortal1.setFill(0,0,0);
  //world.add(wallHidePortal1);
  
  //excited
  wallPortal5                   = new FBox(15, 6);
  wallPortal5.setPosition(18, 22);
  wallPortal5.setStatic(true);
  //wallPortal5.setSensor(true);
  wallPortal5.setDensity(10000);
  wallPortal5.setSensor(true);
  wallPortal5.setFill(0,0,0);
  wallPortal5.setNoStroke();
  world.add(wallPortal5);
  
  //stuck/scared/dead
  wallPortal2                   = new FBox(5, worldHeight/3);
  wallPortal2.setPosition(2, 18);
  wallPortal2.setStatic(true);
  wallPortal2.setSensor(true);
  wallPortal2.setFriction(100);
  wallPortal2.setFill(0,0,0);
  world.add(wallPortal2);


  //wall4                   = new FBox(15, worldHeight/6);
  //wall4.setPosition(16, 22);
  //wall4.setStatic(true);
  //wall4.setSensor(true);
  ////wall4.setFill(0,134,255);
  //wall4.setNoStroke();
  //world.add(wall4);
 
  //hides bottom wall
  //wallHidePortal5                   = new FBox(15, 10);
  //wallHidePortal5.setPosition(18, 22);
  //wallHidePortal5.setStatic(true);
  //wallHidePortal5.setSensor(true);
  //wallHidePortal5.setFill(0,0,0);
  //world.add(wallHidePortal5);
  
  //sluggish
  wallPortal4                  = new FBox(7, worldHeight/2);
  wallPortal4.setPosition(31, 10);
  wallPortal4.setFill(0,0,0);
  //wallPortal4.setDensity(800);
  wallPortal4.setNoStroke();
  wallPortal4.setSensor(true);
  wallPortal4.setStatic(true);
  world.add(wallPortal4); 
  
  
  //moved, one way 
  wallPortal3                  = new FBox(4,8);
  wallPortal3.setPosition(17, 8);
  wallPortal3.setFill(0,0,0);
  //wallPortal4.setDensity(800);
  wallPortal3.setNoStroke();
  wallPortal3.setSensor(true);
  wallPortal3.setStatic(true);
  world.add(wallPortal3); 
  
  
  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4);  
  s.h_avatar.setSensor(false);
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
 
  
  /* If you are developing on a Mac users must update the path below 
   * from "../img/Haply_avatar.png" to "./img/Haply_avatar.png" 
   */
  haplyAvatar = loadImage("../img/space.png"); 
  haplyAvatar.resize((int)(hAPI_Fisica.worldToScreen(2)), (int)(hAPI_Fisica.worldToScreen(2)));
  s.h_avatar.attachImage(haplyAvatar); 

  //pac2 = loadImage("../img/pac2.png"); 
  //pac2.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1)));
  //wallPortal4.attachImage(pac2); 

  /* world conditions setup */
  world.setGravity((0.0), (0.0)); //1000 cm/(s^2)
  //world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
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
    background(0);
    fill(255);
    world.draw();
    
    textAlign(CENTER);
    text("It's Dina's first time in space", width/2, 50);    
    textAlign(CENTER);
    text("Move her around, there are different portals to alternate universes", width/2, 70);
    text("Could you use one word to describe Dina's travel experience going through each portal?", width/2, 90);
    textFont(f, 16);
    
    
    text("**Portal 1**", 100, 200);
    text("**Portal 2**", 100, 700);
    text("**Portal 3**", 680, 300);
    text("**Portal 4**", 1200, 300);
    text("**Portal 5**", 700, 800);

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
    
    //if (s.h_avatar.isTouchingBody(wallPortal11) || s.h_avatar.isTouchingBody(wallPortal12)){
    //  if(s.h_avatar.isTouchingBody(wallPortal11)) {
    //    s.h_avatar.adjustPosition(3,0);
    //  }
    //  else
    //  {
    //    s.h_avatar.adjustPosition(-3,0);
    //  }
    //}
    
    //angry
    if (s.h_avatar.isTouchingBody(wallPortal11)){   
      s.h_avatar.adjustPosition(4,0);  
    }

    if (s.h_avatar.isTouchingBody(wallPortal12)){   
      s.h_avatar.adjustPosition(-4,0);    
    }
    
    //excited?
    //if (s.h_avatar.isTouchingBody(wallPortal5)){   
    //  s.h_avatar.setVelocity(20,0);
    //  s.h_avatar.setDamping(500);
    //}
    
    //nervous/excited
    if (s.h_avatar.isTouchingBody(wallPortal5)){   
      //s.h_avatar.setVelocity(20,0);
      //s.h_avatar.setDamping(900);
      //s.h_avatar.setDensity(40);
         angles.set(widgetOne.get_device_angles()); 
         posEE.set(widgetOne.get_device_position(angles.array()));
         
        /* haptic wall force calculation */
        fWall.set(0, 0);

        penWall.set(0, (posWall.y - (posEE.y + rEE)));

        if (penWall.y < 0) {
          fWall = fWall.add(penWall.mult(-kWall));
        }

        fEE = (fWall.copy()).mult(-1);
        fEE.set(graphics_to_device(fEE));
        /* end haptic wall force calculation */
        posEE.set(posEE.copy().mult(175));
    }
        
    
    //sluggish
    if(s.h_avatar.isTouchingBody(wallPortal4))
    {
       s.h_avatar.setDamping(800);
      
    }
    
     //moved 
    if(s.h_avatar.isTouchingBody(wallPortal3))
    {
       s.h_avatar.setVelocity(0,70);
      
    }

    
    if(s.h_avatar.isTouchingBody(wallPortal2))
    {
       s.h_avatar.setStatic(true);
        currentMillis = millis();
        if (currentMillis - previousMillis > interval) {
          s.h_avatar.setStatic(false);
          s.h_avatar.setPosition(15,14);
          previousMillis=millis();
        }     
    }
    
    
    if(s.h_avatar.isTouchingBody(wallHidePortal1) || s.h_avatar.isTouchingBody(wallHidePortal5) || s.h_avatar.isTouchingBody(wallPortal4) || s.h_avatar.isTouchingBody(wallPortal2) || s.h_avatar.isTouchingBody(wallPortal3)) {
      //s.h_avatar.dettachImage();
      //s.h_avatar.setNoFill();
    }
    else
    {
      s.h_avatar.attachImage(haplyAvatar); 
    }

    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/
PVector device_to_graphics(PVector deviceFrame) {
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}

PVector graphics_to_device(PVector graphicsFrame) {
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}
/* end helper functions section ****************************************************************************************/
