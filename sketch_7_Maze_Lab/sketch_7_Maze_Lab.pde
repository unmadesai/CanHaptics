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



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                = new PVector(0, 0); 

/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 30.0;  
float             worldHeight                         = 18.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;
float             edgeBottomLeftX                    = 0.0; 
float             edgeBottomLeftY                    = worldHeight;

/* Initialization of wall */
FBox              wall;


/* define maze blocks */
FBox              b1;
FBox              b2;
FBox              b3;
FBox              b4;
FBox              b5;
FBox              b6;
FBox              b7;
FBox              b8;
FBox              b9;
FBox              b10;
FBox              b11;
FBox              b12;
FBox              b13;
FBox              b14;
FBox              b15;
FBox              b16;
FBox              b17;
FBox              b18;
FBox              b19;
FBox              b20;

FPoly              cr1;
FPoly              cr2;
FPoly              cr3;
FPoly              cr4;
FPoly              cr5;
FPoly              cr6;
FPoly              cr7;

FPoly              f1;
FBox              t1;
FBox              t2;
FBox              t3;
FBox              t4;
FBox              t5;
FBox              t6;

FPoly             tt1;
FPoly             tt2;
FPoly             tt3;
FPoly             tt4;
FPoly             tt5;
FPoly             tt6;


FPoly             d1;
FPoly             d1e1;
FBox              d1b;
FBox              d1t;

FPoly             d2;
FPoly             d2e1;
FBox              d2b;
FBox              d2t;

FCircle           o;
FPoly             ob;
FBox              ol;


/* define game start */
boolean           gameStart                           = true;

/* text font */
PFont             f;

/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar;

/* end elements definition *********************************************************************************************/ 



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1200, 720);
  
  /* device setup */
  /* set font type and size */
  f                   = createFont("Cambria", 20, true);
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
  
  
  /* creation of wall 
  wall                   = new FBox(10.0, 0.5);
  wall.setPosition(edgeTopLeftX+worldWidth/2.0, edgeTopLeftY+2*worldHeight/3.0);
  wall.setStatic(true);
  wall.setFill(0, 0, 0);
  world.add(wall); */

 /* Set maze barriers */
  b1                  = new FBox(1.5, 2.0);
  b1.setPosition(edgeTopLeftX+worldWidth/4.0-5, edgeTopLeftY+worldHeight/2-4); 
  b1.setStatic(true);
  b1.setFill(255,0,102);
  world.add(b1);
  
  b2                  = new FBox(1.5, 2.0);
  b2.setPosition(edgeTopLeftX+worldWidth/4.0-5, edgeTopLeftY+worldHeight/2-2); 
  b2.setStatic(true);
  b2.setFill(255,153,153);
  world.add(b2);
  
  b3                  = new FBox(1.5, 2.0);
  b3.setPosition(edgeTopLeftX+worldWidth/4.0-5, edgeTopLeftY+worldHeight/2); 
  b3.setStatic(true);
  b3.setFill(204,0,82);
  world.add(b3);
  
  b4                  = new FBox(1.5, 2.0);
  b4.setPosition(edgeTopLeftX+worldWidth/4.0-5, edgeTopLeftY+worldHeight/2+2); 
  b4.setStatic(true);
  b4.setFill(255,204,204);
  world.add(b4);


 
  /* creation of Tree 1 */
  tt1                   = new FPoly(); 
  tt1.vertex(-0.5, 1.0);
  tt1.vertex( 0.5, 1.0);
  tt1.vertex( 0, 0);
  tt1.setPosition(edgeTopLeftX+worldWidth/4.0-2, edgeTopLeftY+worldHeight/2-3); 
  tt1.setStatic(true);
  tt1.setFill(0,179,0);
  world.add(tt1); 

  t1                  = new FBox(0.4, 0.8);
  t1.setPosition(edgeTopLeftX+worldWidth/4.0-2, edgeTopLeftY+worldHeight/2-1.6); 
  t1.setStatic(true);
  t1.setFill(153,102,51);
  world.add(t1); 
 
 
  /* creation of Tree 2*/
  tt2                   = new FPoly(); 
  tt2.vertex(-0.5, 1.0);
  tt2.vertex( 0.5, 1.0);
  tt2.vertex( 0, 0);
  tt2.setPosition(edgeTopLeftX+worldWidth/4.0, edgeTopLeftY+worldHeight/2); 
  tt2.setStatic(true);
  tt2.setFill(0, 204, 102);
  world.add(tt2); 

  t2                  = new FBox(0.4, 0.8);
  t2.setPosition(edgeTopLeftX+worldWidth/4.0, edgeTopLeftY+worldHeight/2+1.4); 
  t2.setStatic(true);
  t2.setFill(102,68,0);
  world.add(t2);  
  
  
    /* creation of Tree 4*/
  tt4                   = new FPoly(); 
  tt4.vertex(-0.5, 1.0);
  tt4.vertex( 0.5, 1.0);
  tt4.vertex( 0, 0);
  tt4.setPosition(edgeTopLeftX+worldWidth/4.0, edgeTopLeftY+worldHeight/2-6); 
  tt4.setStatic(true);
  tt4.setFill(0, 204, 102);
  world.add(tt4); 

  t4                  = new FBox(0.4, 0.8);
  t4.setPosition(edgeTopLeftX+worldWidth/4.0, edgeTopLeftY+worldHeight/2-4.6); 
  t4.setStatic(true);
  t4.setFill(102,68,0);
  world.add(t4);  
  
  
  
  /* dog */
  d1e1                   = new FPoly(); 
  d1e1.vertex(-0.5, 1.0);
  d1e1.vertex( 0.5, 1.0);
  d1e1.vertex( 0, 0);
  d1e1.setPosition(edgeTopLeftX+worldWidth/4.0, edgeTopLeftY+worldHeight/2+6); 
  d1e1.setStatic(true);
  d1e1.setFill(0,179,0);
  world.add(d1e1); 

  d1                   = new FPoly(); 
  d1.vertex(-0.4, 1.0);
  d1.vertex( 0.4, 1.0);
  d1.vertex( 0, 0);
  d1.setPosition(edgeTopLeftX+worldWidth/4.0, edgeTopLeftY+worldHeight/2+5.6); 
  d1.setStatic(true);
  d1.setFill(0,179,0);
  world.add(d1);

  d1b                  = new FBox(0.6, 0.88);
  d1b.setPosition(edgeTopLeftX+worldWidth/4.0, edgeTopLeftY+worldHeight/2+7); 
  d1b.setStatic(true);
  d1b.setFill(153,102,51);
  world.add(d1b); 
  
  d1t                  = new FBox(0.2, 0.3);
  d1t.setPosition(edgeTopLeftX+worldWidth/4.0, edgeTopLeftY+worldHeight/2+7.6); 
  d1t.setStatic(true);
  d1t.setFill(153,102,51);
  world.add(d1t); 


  /* Building T*/
  b9                  = new FBox(2.0, 1.0);
  b9.setPosition(edgeTopLeftX+worldWidth/4.0+3, edgeTopLeftY+worldHeight/2+6); 
  b9.setStatic(true);
  b9.setFill(0, 136, 204);
  world.add(b9);

  b10                  = new FBox(2.0, 1.0);
  b10.setPosition(edgeTopLeftX+worldWidth/4.0+5, edgeTopLeftY+worldHeight/2+6); 
  b10.setStatic(true);
  b10.setFill(179, 230, 255);
  world.add(b10);
  
  b11                  = new FBox(1.0, 1.8);
  b11.setPosition(edgeTopLeftX+worldWidth/4.0+5.5, edgeTopLeftY+worldHeight/2+7.4); 
  b11.setStatic(true);
  b11.setFill(0, 51, 77);
  world.add(b11); 
  


  /* Building 2*/
  b5                  = new FBox(1.0, 2.0);
  b5.setPosition(edgeTopLeftX+worldWidth/4.0+3, edgeTopLeftY+worldHeight/2-6); 
  b5.setStatic(true);
  b5.setFill(102, 204, 255);
  world.add(b5);
  
  b6                  = new FBox(1.0, 2.0);
  b6.setPosition(edgeTopLeftX+worldWidth/4.0+3, edgeTopLeftY+worldHeight/2-4); 
  b6.setStatic(true);
  b6.setFill(0, 136, 204);
  world.add(b6);

  b7                  = new FBox(1.0, 2.0);
  b7.setPosition(edgeTopLeftX+worldWidth/4.0+3, edgeTopLeftY+worldHeight/2); 
  b7.setStatic(true);
  b7.setFill(179, 230, 255);
  world.add(b7);
  
  b8                  = new FBox(1.0, 2.0);
  b8.setPosition(edgeTopLeftX+worldWidth/4.0+3, edgeTopLeftY+worldHeight/2+2); 
  b8.setStatic(true);
  b8.setFill(0, 51, 77);
  world.add(b8);  
    
  /*cr1                  = new FBox(2.0, 1.0);
  cr1.setPosition(edgeTopLeftX+worldWidth/4.0+5, edgeTopLeftY+worldHeight/2-4); 
  cr1.setStatic(true);
  cr1.setFill(255, 153, 51);
  world.add(cr1);  
  
  /* creation of Car 1*/
  cr1                   = new FPoly(); 
  cr1.vertex(-1, 0.25);
  cr1.vertex(-0.5, 0.5);
  cr1.vertex( 1.0, 0.5);
  cr1.vertex( 1.0, -0.5);
  cr1.vertex(-0.5, -0.5);
  cr1.vertex(-1, -0.25);
  cr1.setPosition(edgeTopLeftX+worldWidth/4.0+7, edgeTopLeftY+worldHeight/2-4); 
  cr1.setStatic(true);
  cr1.setFill(255, 153, 51);
  world.add(cr1);   
    
    
  /* creation of Car 2*/
  cr2                   = new FPoly(); 
  cr2.vertex(-1, 0.25);
  cr2.vertex(-0.5, 0.5);
  cr2.vertex( 1.0, 0.5);
  cr2.vertex( 1.0, -0.5);
  cr2.vertex(-0.5, -0.5);
  cr2.vertex(-1, -0.25);
  cr2.setPosition(edgeTopLeftX+worldWidth/4.0+7, edgeTopLeftY+worldHeight/2-2.5); 
  cr2.setStatic(true);
  cr2.setFill(0, 153, 153);
  world.add(cr2);   
  
  /* creation of Car 3*/
  cr3                   = new FPoly(); 
  cr3.vertex(-1, 0.25);
  cr3.vertex(-0.5, 0.5);
  cr3.vertex( 1.0, 0.5);
  cr3.vertex( 1.0, -0.5);
  cr3.vertex(-0.5, -0.5);
  cr3.vertex(-1, -0.25);
  cr3.setPosition(edgeTopLeftX+worldWidth/4.0+11, edgeTopLeftY+worldHeight/2-1); 
  cr3.setStatic(true);
  cr3.setFill(204, 153, 255);
  world.add(cr3);   
  
  /* creation of Car 4*/
  cr4                   = new FPoly(); 
  cr4.vertex(-1, 0.25);
  cr4.vertex(-0.5, 0.5);
  cr4.vertex( 1.0, 0.5);
  cr4.vertex( 1.0, -0.5);
  cr4.vertex(-0.5, -0.5);
  cr4.vertex(-1, -0.25);
  cr4.setPosition(edgeTopLeftX+worldWidth/4.0+7, edgeTopLeftY+worldHeight/2+0.5); 
  cr4.setStatic(true);
  cr4.setFill(102, 153, 153);
  world.add(cr4);   

 /* creation of Fire Hydrant */
  f1                   = new FPoly(); 
  f1.vertex(-0.75, 0.5);
  f1.vertex( 0.75, 0.5);
  f1.vertex( 0, -0.5);
  f1.setPosition(edgeTopLeftX+worldWidth/4.0+10, edgeTopLeftY+worldHeight/2+5); 
  f1.setStatic(true);
  f1.setFill(255, 71, 26);
  world.add(f1);
  

  /* creation of Tree 3 */
  tt3                   = new FPoly(); 
  tt3.vertex(-0.5, 1.0);
  tt3.vertex( 0.5, 1.0);
  tt3.vertex( 0, 0);
  tt3.setPosition(edgeTopLeftX+worldWidth/4.0+11.8, edgeTopLeftY+worldHeight/2+4); 
  tt3.setStatic(true);
  tt3.setFill(96, 128, 0);
  world.add(tt3); 

  t3                  = new FBox(0.4, 0.8);
  t3.setPosition(edgeTopLeftX+worldWidth/4.0+11.8, edgeTopLeftY+worldHeight/2+5.4); 
  t3.setStatic(true);
  t3.setFill(204, 163, 0);
  world.add(t3);   
  
   /* creation of Tree 5*/
  tt5                   = new FPoly(); 
  tt5.vertex(-0.5, 1.0);
  tt5.vertex( 0.5, 1.0);
  tt5.vertex( 0, 0);
  tt5.setPosition(edgeTopLeftX+worldWidth/4.0+13, edgeTopLeftY+worldHeight/2+5); 
  tt5.setStatic(true);
  tt5.setFill(0, 204, 102);
  world.add(tt5); 

  t5                  = new FBox(0.4, 0.8);
  t5.setPosition(edgeTopLeftX+worldWidth/4.0+13, edgeTopLeftY+worldHeight/2+6.4); 
  t5.setStatic(true);
  t5.setFill(102,68,0);
  world.add(t5); 
  
   /* creation of Tree 6*/
  tt6                   = new FPoly(); 
  tt6.vertex(-0.5, 1.0);
  tt6.vertex( 0.5, 1.0);
  tt6.vertex( 0, 0);
  tt6.setPosition(edgeTopLeftX+worldWidth/4.0+11, edgeTopLeftY+worldHeight/2+6); 
  tt6.setStatic(true);
  tt6.setFill(0, 204, 102);
  world.add(tt6); 

  t6                  = new FBox(0.4, 0.8);
  t6.setPosition(edgeTopLeftX+worldWidth/4.0+11, edgeTopLeftY+worldHeight/2+7.4); 
  t6.setStatic(true);
  t6.setFill(102,68,0);
  world.add(t6); 
  
  /* creation of Car 5*/
  cr5                   = new FPoly(); 
  cr5.vertex(-1, 0.25);
  cr5.vertex(-0.5, 0.5);
  cr5.vertex( 1.0, 0.5);
  cr5.vertex( 1.0, -0.5);
  cr5.vertex(-0.5, -0.5);
  cr5.vertex(-1, -0.25);
  cr5.setPosition(edgeTopLeftX+worldWidth/4.0+10, edgeTopLeftY+worldHeight/2-6); 
  cr5.setStatic(true);
  cr5.setFill(255, 77, 148);
  world.add(cr5);   
  
  /* creation of Car 6*/
  cr6                   = new FPoly(); 
  cr6.vertex(-1, 0.25);
  cr6.vertex(-0.5, 0.5);
  cr6.vertex( 1.0, 0.5);
  cr6.vertex( 1.0, -0.5);
  cr6.vertex(-0.5, -0.5);
  cr6.vertex(-1, -0.25);
  cr6.setPosition(edgeTopLeftX+worldWidth/4.0+12.5, edgeTopLeftY+worldHeight/2-6); 
  cr6.setStatic(true);
  cr6.setFill(230, 230, 0);
  world.add(cr6);     

  /* creation of Car 7*/
  cr7                   = new FPoly(); 
  cr7.vertex(-1, 0.25);
  cr7.vertex(-0.5, 0.5);
  cr7.vertex( 1.0, 0.5);
  cr7.vertex( 1.0, -0.5);
  cr7.vertex(-0.5, -0.5);
  cr7.vertex(-1, -0.25);
  cr7.setPosition(edgeTopLeftX+worldWidth/4.0+15, edgeTopLeftY+worldHeight/2-6); 
  cr7.setStatic(true);
  cr7.setFill(153, 230, 255);
  world.add(cr7);    
  
  
  /* Building 3 */
  b12                  = new FBox(1.5, 2.0);
  b12.setPosition(edgeTopLeftX+worldWidth/4.0+16, edgeTopLeftY+worldHeight/2-2); 
  b12.setStatic(true);
  b12.setFill(255, 153, 102);
  world.add(b12);
  
  b13                  = new FBox(1.5, 2.0);
  b13.setPosition(edgeTopLeftX+worldWidth/4.0+16, edgeTopLeftY+worldHeight/2); 
  b13.setStatic(true);
  b13.setFill(255, 195, 77);
  world.add(b13);
  
  b14                  = new FBox(1.5, 2.0);
  b14.setPosition(edgeTopLeftX+worldWidth/4.0+16, edgeTopLeftY+worldHeight/2+2); 
  b14.setStatic(true);
  b14.setFill(255, 112, 77);
  world.add(b14);
  
  b15                  = new FBox(1.5, 2.0);
  b15.setPosition(edgeTopLeftX+worldWidth/4.0+16, edgeTopLeftY+worldHeight/2+4); 
  b15.setStatic(true);
  b15.setFill(204, 82, 0);
  world.add(b15);
  
  b20                  = new FBox(1.5, 2.0);
  b20.setPosition(edgeTopLeftX+worldWidth/4.0+16, edgeTopLeftY+worldHeight/2+6); 
  b20.setStatic(true);
  b20.setFill(204, 82, 0);
  world.add(b20);
  
  
  /* Building 4 */
  b16                  = new FBox(1.5, 2.0);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0+20, edgeTopLeftY+worldHeight/2-2); 
  b16.setStatic(true);
  b16.setFill(255, 153, 102);
  world.add(b16);
  
  b17                  = new FBox(1.5, 2.0);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0+20, edgeTopLeftY+worldHeight/2); 
  b17.setStatic(true);
  b17.setFill(255, 195, 77);
  world.add(b17);
  
  b18                  = new FBox(1.5, 2.0);
  b18.setPosition(edgeTopLeftX+worldWidth/4.0+20, edgeTopLeftY+worldHeight/2+2); 
  b18.setStatic(true);
  b18.setFill(255, 112, 77);
  world.add(b18);
  
  b19                  = new FBox(1.5, 2.0);
  b19.setPosition(edgeTopLeftX+worldWidth/4.0+20, edgeTopLeftY+worldHeight/2+4); 
  b19.setStatic(true);
  b19.setFill(204, 82, 0);
  world.add(b19);
  
  
    
  /* dog */
  d2e1                   = new FPoly(); 
  d2e1.vertex(-0.8, 0);
  d2e1.vertex( 0, 0.6);
  d2e1.vertex( 0, -0.6);
  d2e1.setPosition(edgeTopLeftX+worldWidth/4.0+19.8, edgeTopLeftY+worldHeight/2-7); 
  d2e1.setStatic(true);
  d2e1.setFill(0,179,0);
  world.add(d2e1); 

  d2                   = new FPoly(); 
  d2.vertex(-0.8, 0);
  d2.vertex( 0, 0.5);
  d2.vertex( 0, -0.5);
  d2.setPosition(edgeTopLeftX+worldWidth/4.0+19.4, edgeTopLeftY+worldHeight/2-7); 
  d2.setStatic(true);
  d2.setFill(0,179,0);
  world.add(d2);

  d2b                  = new FBox(1.2, 0.8);
  d2b.setPosition(edgeTopLeftX+worldWidth/4.0+20, edgeTopLeftY+worldHeight/2-7); 
  d2b.setStatic(true);
  d2b.setFill(153,102,51);
  world.add(d2b); 
  
  d2t                  = new FBox(0.5, 0.3);
  d2t.setPosition(edgeTopLeftX+worldWidth/4.0+20.8, edgeTopLeftY+worldHeight/2-7); 
  d2t.setStatic(true);
  d2t.setFill(153,102,51);
  world.add(d2t); 
  
    /* creation of Owner*/
  o                   = new FCircle(1.0);
  o.setPosition(edgeTopLeftX+worldWidth/4.0+18.2, edgeTopLeftY+worldHeight/2+5.5); 
  o.setStatic(true);
  o.setFill(0, 204, 102);
  world.add(o);
  
  ob                   = new FPoly(); 
  ob.vertex(-0.5, 1.0);
  ob.vertex( 0.5, 1.0);
  ob.vertex( 0, 0);
  ob.setPosition(edgeTopLeftX+worldWidth/4.0+18.2, edgeTopLeftY+worldHeight/2+6); 
  ob.setStatic(true);
  ob.setFill(0, 204, 102);
  world.add(ob); 

  ol                  = new FBox(0.4, 0.8);
  ol.setPosition(edgeTopLeftX+worldWidth/4.0+18.2, edgeTopLeftY+worldHeight/2+7.4); 
  ol.setStatic(true);
  ol.setFill(102,68,0);
  world.add(ol); 
    
    
    
  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4);
  s.h_avatar.setRotation(0);
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
 
  
  /* If you are developing on a Mac users must update the path below 
   * from "../img/Haply_avatar.png" to "./img/Haply_avatar.png" 
   */
  haplyAvatar = loadImage("../img/dog.png"); 
  haplyAvatar.resize((int)(hAPI_Fisica.worldToScreen(1)), (int)(hAPI_Fisica.worldToScreen(1)));
  s.h_avatar.attachImage(haplyAvatar); 


  /* world conditions setup */
  world.setGravity((0.0), (1000.0)); //1000 cm/(s^2)
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
    textFont(f, 20);
    
     if(gameStart){
      fill(26, 26, 0);
      textAlign(LEFT);
      text("Buddy is lost, get him to his owner!", (width/2)-550, (height/2)+180);
      text("Note: Buddy is scared of other dogs,", (width/2)-550, (height/2)+200);
      text("and be careful of the cars!", (width/2)-550, (height/2)+220);
     }
    
    else if(gameStart == false){
      b1.setDrawable(false);
      b2.setDrawable(false);
      b3.setDrawable(false);
      b4.setDrawable(false);
      b5.setDrawable(false);
      b6.setDrawable(false);
      b7.setDrawable(false);
      b8.setDrawable(false);
      b9.setDrawable(false);
      b10.setDrawable(false);
      b11.setDrawable(false);
      b12.setDrawable(false);
      b13.setDrawable(false);
      b14.setDrawable(false);
      b15.setDrawable(false);
      b16.setDrawable(false);
      b17.setDrawable(false);
      b18.setDrawable(false);
      b19.setDrawable(false);
      b20.setDrawable(false);      
      cr1.setDrawable(false);
      cr2.setDrawable(false);
      cr3.setDrawable(false);
      cr4.setDrawable(false);
      cr5.setDrawable(false);
      cr6.setDrawable(false);
      cr7.setDrawable(false);
      f1.setDrawable(false);
      t1.setDrawable(false);
      t2.setDrawable(false);
      t3.setDrawable(false);
      t4.setDrawable(false);
      t5.setDrawable(false);
      t6.setDrawable(false);
      tt1.setDrawable(false);
      tt2.setDrawable(false);
      tt3.setDrawable(false);
      tt4.setDrawable(false);
      tt5.setDrawable(false);
      tt6.setDrawable(false);
      d1.setDrawable(false);
      d1e1.setDrawable(false);
      d1b.setDrawable(false);
      d1t.setDrawable(false);
      d2.setDrawable(false);
      d2e1.setDrawable(false);
      d2b.setDrawable(false);
      d2t.setDrawable(false);
      textAlign(CENTER);
      textFont(f, 36);
      fill(255, 51, 51);
      text("Congrats!", width/2, height/2);
      textFont(f, 24);
      fill(64, 0, 128);
      text("Buddy made it home!", width/2, (height/2)+100);
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
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
  if(s.h_avatar.isTouchingBody(o)){
      gameStart = false;
      s.h_avatar.setSensor(true);
    }
  
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
