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
float             worldWidth                          = 30.0f;  
float             worldHeight                         = 18.0f; 

float             edgeTopLeftX                        = 0.0f; 
float             edgeTopLeftY                        = 0.0f; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;
float             edgeBottomLeftX                    = 0.0f; 
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
  
  
 /* Create obstacles */
  b1                  = new FBox(1.5f, 2.0f);
  b1.setPosition(edgeTopLeftX+worldWidth/4.0f-5, edgeTopLeftY+worldHeight/2-4); 
  b1.setStatic(true);
  b1.setFill(92, 92, 138);
  b1.setNoStroke();
  world.add(b1);
  
  b2                  = new FBox(1.5f, 2.0f);
  b2.setPosition(edgeTopLeftX+worldWidth/4.0f-5, edgeTopLeftY+worldHeight/2-2); 
  b2.setStatic(true);
  b2.setFill(133, 133, 173);
  b2.setNoStroke();
  world.add(b2);
  
  b3                  = new FBox(1.5f, 2.0f);
  b3.setPosition(edgeTopLeftX+worldWidth/4.0f-5, edgeTopLeftY+worldHeight/2); 
  b3.setStatic(true);
  b3.setFill(51, 51, 77);
  b3.setNoStroke();
  world.add(b3);
  
  b4                  = new FBox(1.5f, 2.0f);
  b4.setPosition(edgeTopLeftX+worldWidth/4.0f-5, edgeTopLeftY+worldHeight/2+2); 
  b4.setStatic(true);
  b4.setFill(179, 179, 203);
  b4.setNoStroke();
  world.add(b4);


 
  /* creation of Tree 1 */
  tt1                   = new FPoly(); 
  tt1.vertex(-0.5f, 1.0f);
  tt1.vertex( 0.5f, 1.0f);
  tt1.vertex( 0, 0);
  tt1.setPosition(edgeTopLeftX+worldWidth/4.0f-2, edgeTopLeftY+worldHeight/2-3); 
  tt1.setStatic(true);
  tt1.setFill(0,179,0);
  tt1.setNoStroke();
  world.add(tt1); 

  t1                  = new FBox(0.4f, 0.8f);
  t1.setPosition(edgeTopLeftX+worldWidth/4.0f-2, edgeTopLeftY+worldHeight/2-1.6f); 
  t1.setStatic(true);
  t1.setFill(153,102,51);
  t1.setNoStroke();
  world.add(t1); 
 
 
  /* creation of Tree 2*/
  tt2                   = new FPoly(); 
  tt2.vertex(-0.5f, 1.0f);
  tt2.vertex( 0.5f, 1.0f);
  tt2.vertex( 0, 0);
  tt2.setPosition(edgeTopLeftX+worldWidth/4.0f, edgeTopLeftY+worldHeight/2); 
  tt2.setStatic(true);
  tt2.setFill(0, 204, 102);
  tt2.setNoStroke();
  world.add(tt2); 

  t2                  = new FBox(0.4f, 0.8f);
  t2.setPosition(edgeTopLeftX+worldWidth/4.0f, edgeTopLeftY+worldHeight/2+1.4f); 
  t2.setStatic(true);
  t2.setFill(102,68,0);
  t2.setNoStroke();
  world.add(t2);  
  
  
    /* creation of Tree 4*/
  tt4                   = new FPoly(); 
  tt4.vertex(-0.5f, 1.0f);
  tt4.vertex( 0.5f, 1.0f);
  tt4.vertex( 0, 0);
  tt4.setPosition(edgeTopLeftX+worldWidth/4.0f, edgeTopLeftY+worldHeight/2-6); 
  tt4.setStatic(true);
  tt4.setFill(0, 204, 102);
  tt4.setNoStroke();
  world.add(tt4); 

  t4                  = new FBox(0.4f, 0.8f);
  t4.setPosition(edgeTopLeftX+worldWidth/4.0f, edgeTopLeftY+worldHeight/2-4.6f); 
  t4.setStatic(true);
  t4.setFill(102,68,0);
  t4.setNoStroke();
  world.add(t4);  
  
  
  
  /* dog */
  d1e1                   = new FPoly(); 
  d1e1.vertex(-0.5f, 1.0f);
  d1e1.vertex( 0.5f, 1.0f);
  d1e1.vertex( 0, 0);
  d1e1.setPosition(edgeTopLeftX+worldWidth/4.0f, edgeTopLeftY+worldHeight/2+6); 
  d1e1.setStatic(true);
  d1e1.setFill(128, 128, 128);
  d1e1.setNoStroke();
  world.add(d1e1); 

  d1                   = new FPoly(); 
  d1.vertex(-0.4f, 1.0f);
  d1.vertex( 0.4f, 1.0f);
  d1.vertex( 0, 0);
  d1.setPosition(edgeTopLeftX+worldWidth/4.0f, edgeTopLeftY+worldHeight/2+5.6f); 
  d1.setStatic(true);
  d1.setFill(140, 140, 140);
  d1.setNoStroke();
  world.add(d1);

  d1b                  = new FBox(0.6f, 0.88f);
  d1b.setPosition(edgeTopLeftX+worldWidth/4.0f, edgeTopLeftY+worldHeight/2+7); 
  d1b.setStatic(true);
  d1b.setFill(89, 89, 89);
  d1b.setNoStroke();
  world.add(d1b); 
  
  d1t                  = new FBox(0.2f, 0.3f);
  d1t.setPosition(edgeTopLeftX+worldWidth/4.0f, edgeTopLeftY+worldHeight/2+7.6f); 
  d1t.setStatic(true);
  d1t.setFill(128, 128, 128);
  d1t.setNoStroke();
  world.add(d1t); 


  /* Building T*/
  b9                  = new FBox(2.0f, 1.0f);
  b9.setPosition(edgeTopLeftX+worldWidth/4.0f+3, edgeTopLeftY+worldHeight/2+6); 
  b9.setStatic(true);
  b9.setFill(0, 136, 204);
  b9.setNoStroke();
  world.add(b9);

  b10                  = new FBox(2.0f, 1.0f);
  b10.setPosition(edgeTopLeftX+worldWidth/4.0f+5, edgeTopLeftY+worldHeight/2+6); 
  b10.setStatic(true);
  b10.setFill(179, 230, 255);
  b10.setNoStroke();
  world.add(b10);
  
  b11                  = new FBox(1.0f, 1.8f);
  b11.setPosition(edgeTopLeftX+worldWidth/4.0f+5.5f, edgeTopLeftY+worldHeight/2+7.4f); 
  b11.setStatic(true);
  b11.setFill(0, 51, 77);
  b11.setNoStroke();
  world.add(b11); 
  


  /* Building 2*/
  b5                  = new FBox(1.0f, 2.0f);
  b5.setPosition(edgeTopLeftX+worldWidth/4.0f+3, edgeTopLeftY+worldHeight/2-6); 
  b5.setStatic(true);
  b5.setFill(102, 204, 255);
  b5.setNoStroke();
  world.add(b5);
  
  b6                  = new FBox(1.0f, 2.0f);
  b6.setPosition(edgeTopLeftX+worldWidth/4.0f+3, edgeTopLeftY+worldHeight/2-4); 
  b6.setStatic(true);
  b6.setFill(0, 136, 204);
  b6.setNoStroke();
  world.add(b6);

  b7                  = new FBox(1.0f, 2.0f);
  b7.setPosition(edgeTopLeftX+worldWidth/4.0f+3, edgeTopLeftY+worldHeight/2); 
  b7.setStatic(true);
  b7.setFill(179, 230, 255);
  b7.setNoStroke();
  world.add(b7);
  
  b8                  = new FBox(1.0f, 2.0f);
  b8.setPosition(edgeTopLeftX+worldWidth/4.0f+3, edgeTopLeftY+worldHeight/2+2); 
  b8.setStatic(true);
  b8.setFill(0, 51, 77);
  b8.setNoStroke();
  world.add(b8);  
    
  
  /* creation of Car 1*/ 
  cr1                   = new FPoly(); 
  cr1.vertex(-1, 0.25f);
  cr1.vertex(-0.5f, 0.5f);
  cr1.vertex( 1.0f, 0.5f);
  cr1.vertex( 1.0f, -0.5f);
  cr1.vertex(-0.5f, -0.5f);
  cr1.vertex(-1, -0.25f);
  cr1.setPosition(edgeTopLeftX+worldWidth/4.0f+7, edgeTopLeftY+worldHeight/2-4); 
  cr1.setStatic(true);
  cr1.setFill(255, 153, 51);
  cr1.setNoStroke();
  world.add(cr1);   

    
  /* creation of Car 2*/
  cr2                   = new FPoly(); 
  cr2.vertex(-1, 0.25f);
  cr2.vertex(-0.5f, 0.5f);
  cr2.vertex( 1.0f, 0.5f);
  cr2.vertex( 1.0f, -0.5f);
  cr2.vertex(-0.5f, -0.5f);
  cr2.vertex(-1, -0.25f);
  cr2.setPosition(edgeTopLeftX+worldWidth/4.0f+7, edgeTopLeftY+worldHeight/2-2.5f); 
  cr2.setStatic(true);
  cr2.setFill(0, 153, 153);
  cr2.setNoStroke();
  world.add(cr2);   
  
  /* creation of Car 3*/
  cr3                   = new FPoly(); 
  cr3.vertex(-1, 0.25f);
  cr3.vertex(-0.5f, 0.5f);
  cr3.vertex( 1.0f, 0.5f);
  cr3.vertex( 1.0f, -0.5f);
  cr3.vertex(-0.5f, -0.5f);
  cr3.vertex(-1, -0.25f);
  cr3.setPosition(edgeTopLeftX+worldWidth/4.0f+11, edgeTopLeftY+worldHeight/2-1); 
  cr3.setStatic(true);
  cr3.setFill(204, 153, 255);
  cr3.setNoStroke();
  world.add(cr3);   
  
  /* creation of Car 4*/
  cr4                   = new FPoly(); 
  cr4.vertex(-1, 0.25f);
  cr4.vertex(-0.5f, 0.5f);
  cr4.vertex( 1.0f, 0.5f);
  cr4.vertex( 1.0f, -0.5f);
  cr4.vertex(-0.5f, -0.5f);
  cr4.vertex(-1, -0.25f);
  cr4.setPosition(edgeTopLeftX+worldWidth/4.0f+7, edgeTopLeftY+worldHeight/2+0.5f); 
  cr4.setStatic(true);
  cr4.setFill(153, 0, 102);
  cr4.setNoStroke();
  world.add(cr4);   

 /* creation of Fire Hydrant */
  f1                   = new FPoly(); 
  f1.vertex(-0.75f, 0.5f);
  f1.vertex( 0.75f, 0.5f);
  f1.vertex( 0, -0.5f);
  f1.setPosition(edgeTopLeftX+worldWidth/4.0f+10, edgeTopLeftY+worldHeight/2+5); 
  f1.setStatic(true);
  f1.setFill(255, 71, 26);
  f1.setNoStroke();
  world.add(f1);
  

  /* creation of Tree 3 */
  tt3                   = new FPoly(); 
  tt3.vertex(-0.5f, 1.0f);
  tt3.vertex( 0.5f, 1.0f);
  tt3.vertex( 0, 0);
  tt3.setPosition(edgeTopLeftX+worldWidth/4.0f+11.8f, edgeTopLeftY+worldHeight/2+4); 
  tt3.setStatic(true);
  tt3.setFill(96, 128, 0);
  tt3.setNoStroke();
  world.add(tt3); 

  t3                  = new FBox(0.4f, 0.8f);
  t3.setPosition(edgeTopLeftX+worldWidth/4.0f+11.8f, edgeTopLeftY+worldHeight/2+5.4f); 
  t3.setStatic(true);
  t3.setFill(204, 163, 0);
  t3.setNoStroke();
  world.add(t3);   
  
   /* creation of Tree 5*/
  tt5                   = new FPoly(); 
  tt5.vertex(-0.5f, 1.0f);
  tt5.vertex( 0.5f, 1.0f);
  tt5.vertex( 0, 0);
  tt5.setPosition(edgeTopLeftX+worldWidth/4.0f+13, edgeTopLeftY+worldHeight/2+5); 
  tt5.setStatic(true);
  tt5.setFill(0, 204, 102);
  tt5.setNoStroke();
  world.add(tt5); 

  t5                  = new FBox(0.4f, 0.8f);
  t5.setPosition(edgeTopLeftX+worldWidth/4.0f+13, edgeTopLeftY+worldHeight/2+6.4f); 
  t5.setStatic(true);
  t5.setFill(102,68,0);
  t5.setNoStroke();
  world.add(t5); 
  
   /* creation of Tree 6*/
  tt6                   = new FPoly(); 
  tt6.vertex(-0.5f, 1.0f);
  tt6.vertex( 0.5f, 1.0f);
  tt6.vertex( 0, 0);
  tt6.setPosition(edgeTopLeftX+worldWidth/4.0f+11, edgeTopLeftY+worldHeight/2+6); 
  tt6.setStatic(true);
  tt6.setFill(0, 204, 102);
  tt6.setNoStroke();
  world.add(tt6); 

  t6                  = new FBox(0.4f, 0.8f);
  t6.setPosition(edgeTopLeftX+worldWidth/4.0f+11, edgeTopLeftY+worldHeight/2+7.4f); 
  t6.setStatic(true);
  t6.setFill(102,68,0);
  t6.setNoStroke();
  world.add(t6); 
  
  /* creation of Car 5*/
  cr5                   = new FPoly(); 
  cr5.vertex(-1, 0.25f);
  cr5.vertex(-0.5f, 0.5f);
  cr5.vertex( 1.0f, 0.5f);
  cr5.vertex( 1.0f, -0.5f);
  cr5.vertex(-0.5f, -0.5f);
  cr5.vertex(-1, -0.25f);
  cr5.setPosition(edgeTopLeftX+worldWidth/4.0f+10, edgeTopLeftY+worldHeight/2-6); 
  cr5.setStatic(true);
  cr5.setFill(230, 230, 0);
  cr5.setNoStroke();
  world.add(cr5);   
  
  /* creation of Car 6*/
  cr6                   = new FPoly(); 
  cr6.vertex(-1, 0.25f);
  cr6.vertex(-0.5f, 0.5f);
  cr6.vertex( 1.0f, 0.5f);
  cr6.vertex( 1.0f, -0.5f);
  cr6.vertex(-0.5f, -0.5f);
  cr6.vertex(-1, -0.25f);
  cr6.setPosition(edgeTopLeftX+worldWidth/4.0f+12.5f, edgeTopLeftY+worldHeight/2-6); 
  cr6.setStatic(true);
  cr6.setFill(255, 102, 102);
  cr6.setNoStroke();
  world.add(cr6);     

  /* creation of Car 7*/
  cr7                   = new FPoly(); 
  cr7.vertex(-1, 0.25f);
  cr7.vertex(-0.5f, 0.5f);
  cr7.vertex( 1.0f, 0.5f);
  cr7.vertex( 1.0f, -0.5f);
  cr7.vertex(-0.5f, -0.5f);
  cr7.vertex(-1, -0.25f);
  cr7.setPosition(edgeTopLeftX+worldWidth/4.0f+15, edgeTopLeftY+worldHeight/2-6); 
  cr7.setStatic(true);
  cr7.setFill(255, 214, 51);
  cr7.setNoStroke();
  world.add(cr7);    
  
  
  /* Building 3 */
  b12                  = new FBox(1.5f, 2.0f);
  b12.setPosition(edgeTopLeftX+worldWidth/4.0f+16, edgeTopLeftY+worldHeight/2-2); 
  b12.setStatic(true);
  b12.setFill(102, 102, 204);
  b12.setNoStroke();
  world.add(b12);
  
  b13                  = new FBox(1.5f, 2.0f);
  b13.setPosition(edgeTopLeftX+worldWidth/4.0f+16, edgeTopLeftY+worldHeight/2); 
  b13.setStatic(true);
  b13.setFill(179, 179, 230);
  b13.setNoStroke();
  world.add(b13);
  
  b14                  = new FBox(1.5f, 2.0f);
  b14.setPosition(edgeTopLeftX+worldWidth/4.0f+16, edgeTopLeftY+worldHeight/2+2); 
  b14.setStatic(true);
  b14.setFill(92, 92, 138);
  b14.setNoStroke();
  world.add(b14);
  
  b15                  = new FBox(1.5f, 2.0f);
  b15.setPosition(edgeTopLeftX+worldWidth/4.0f+16, edgeTopLeftY+worldHeight/2+4); 
  b15.setStatic(true);
  b15.setFill(194, 194, 214);
  b15.setNoStroke();
  world.add(b15);
  
  b20                  = new FBox(1.5f, 2.0f);
  b20.setPosition(edgeTopLeftX+worldWidth/4.0f+16, edgeTopLeftY+worldHeight/2+6); 
  b20.setStatic(true);
  b20.setFill(61, 61, 92);
  b20.setNoStroke();
  world.add(b20);
  
  
  /* Building 4 */
  b16                  = new FBox(1.5f, 2.0f);
  b16.setPosition(edgeTopLeftX+worldWidth/4.0f+20, edgeTopLeftY+worldHeight/2-2); 
  b16.setStatic(true);
  b16.setFill(102, 163, 255);
  b16.setNoStroke();
  world.add(b16);
  
  b17                  = new FBox(1.5f, 2.0f);
  b17.setPosition(edgeTopLeftX+worldWidth/4.0f+20, edgeTopLeftY+worldHeight/2); 
  b17.setStatic(true);
  b17.setFill(117, 117, 163);
  b17.setNoStroke();
  world.add(b17);
  
  b18                  = new FBox(1.5f, 2.0f);
  b18.setPosition(edgeTopLeftX+worldWidth/4.0f+20, edgeTopLeftY+worldHeight/2+2); 
  b18.setStatic(true);
  b18.setFill(152, 178, 230);
  b18.setNoStroke();
  world.add(b18);
  
  b19                  = new FBox(1.5f, 2.0f);
  b19.setPosition(edgeTopLeftX+worldWidth/4.0f+20, edgeTopLeftY+worldHeight/2+4); 
  b19.setStatic(true);
  b19.setFill(50, 102, 205);
  b19.setNoStroke();
  world.add(b19);
  
  
    
  /* dog */
  d2e1                   = new FPoly(); 
  d2e1.vertex(-0.8f, 0);
  d2e1.vertex( 0, 0.6f);
  d2e1.vertex( 0, -0.6f);
  d2e1.setPosition(edgeTopLeftX+worldWidth/4.0f+19.8f, edgeTopLeftY+worldHeight/2-7); 
  d2e1.setStatic(true);
  d2e1.setFill(153, 115, 0);
  d2e1.setNoStroke();
  world.add(d2e1); 

  d2                   = new FPoly(); 
  d2.vertex(-0.8f, 0);
  d2.vertex( 0, 0.5f);
  d2.vertex( 0, -0.5f);
  d2.setPosition(edgeTopLeftX+worldWidth/4.0f+19.4f, edgeTopLeftY+worldHeight/2-7); 
  d2.setStatic(true);
  d2.setFill(204, 153, 0);
  d2.setNoStroke();
  world.add(d2);

  d2b                  = new FBox(1.2f, 0.8f);
  d2b.setPosition(edgeTopLeftX+worldWidth/4.0f+20, edgeTopLeftY+worldHeight/2-7); 
  d2b.setStatic(true);
  d2b.setFill(102, 77, 0);
  d2b.setNoStroke();
  world.add(d2b); 
  
  d2t                  = new FBox(0.5f, 0.3f);
  d2t.setPosition(edgeTopLeftX+worldWidth/4.0f+20.8f, edgeTopLeftY+worldHeight/2-7); 
  d2t.setStatic(true);
  d2t.setFill(153, 115, 0);
  d2t.setNoStroke();
  world.add(d2t); 
  
    /* creation of Owner*/
  o                   = new FCircle(1.0f);
  o.setPosition(edgeTopLeftX+worldWidth/4.0f+18.2f, edgeTopLeftY+worldHeight/2+5.5f); 
  o.setStatic(true);
  o.setFill(179, 107, 0);
  o.setNoStroke();
  world.add(o);
  
  ob                   = new FPoly(); 
  ob.vertex(-0.5f, 1.0f);
  ob.vertex( 0.5f, 1.0f);
  ob.vertex( 0, 0);
  ob.setPosition(edgeTopLeftX+worldWidth/4.0f+18.2f, edgeTopLeftY+worldHeight/2+6); 
  ob.setStatic(true);
  ob.setFill(0, 230, 172);
  ob.setNoStroke();
  world.add(ob); 

  ol                  = new FBox(0.4f, 0.8f);
  ol.setPosition(edgeTopLeftX+worldWidth/4.0f+18.2f, edgeTopLeftY+worldHeight/2+7.4f); 
  ol.setStatic(true);
  ol.setFill(179, 107, 0);
  ol.setNoStroke();
  world.add(ol); 
    

    
  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4);  
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
public void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    textFont(f, 20);
    
     if(gameStart){
      fill(92, 0, 153);
      textAlign(LEFT);
      text("Buddy is lost, help him reach his owner!", (width/2)-550, (height/2)+180);
      text("Note: Buddy is scared of other dogs,", (width/2)-550, (height/2)+200);
      text("and be careful of the cars!", (width/2)-550, (height/2)+220);
      
      /*labels*/
      textFont(f, 12);
      fill(0, 0, 0);
      textAlign(CENTER);
      text("Walter", (width/2)-500, (height/2)-222);
      text("Gage", (width/2)-500, (height/2)-212);
      text("Residence", (width/2)-500, (height/2)-202);
      
      textFont(f, 16);
      text("<-- Guide him to Owner", (width/2)+100, (height/2)-300);
       
      textFont(f, 12); 
      text("UBC Admin Office", (width/2)-150, (height/2)+300);
      
      text("Car", (width/2)-10, (height/2)-185);
      text("Car", (width/2)-10, (height/2)-60);
      text("Car", (width/2)-10, (height/2)+60);
      text("Car", (width/2)+150, (height/2)+0);
      text("Car", (width/2)+110, (height/2)-200);
      text("Car", (width/2)+210, (height/2)-200);
      text("Car", (width/2)+310, (height/2)-200);
      
      textFont(f, 20);
      text("Owner", (width/2)+480, (height/2)+300);
      
      textFont(f, 12);
      text("Tree", (width/2)-300, (height/2)-155);
      text("Tree", (width/2)-380, (height/2)-35);
      text("Tree", (width/2)-300, (height/2)+85);
      text("Trees", (width/2)+180, (height/2)+300);
      
      text("Marine", (width/2)-130, (height/2)-270);
      text("Drive", (width/2)-130, (height/2)-260);
      
      text("Brock", (width/2)-180, (height/2)+140);
      text("Commons", (width/2)-180, (height/2)+150);
      
      text("Fire", (width/2)+100, (height/2)+235);
      text("Hydrant", (width/2)+100, (height/2)+245);
     
      text("Salish", (width/2)+340, (height/2)+300);
      text("Drive", (width/2)+340, (height/2)+310);
 
      text("Camosun", (width/2)+500, (height/2)+220);
      text("Street", (width/2)+500, (height/2)+230);
      
      text("Dog", (width/2)-340, (height/2)+300);
      text("Dog", (width/2)+500, (height/2)-245);
 
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
      textFont(f, 72);
      fill(255, 77, 77);
      text("YAYYY!", width/2, height/2);
      textFont(f, 48);
      fill(77, 0, 102);
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
