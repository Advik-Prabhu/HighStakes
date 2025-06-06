/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Advik Prabhu and Atharv Bagotra                           */
/*    Created:      2/3/2025, 9:07:19 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/


//Version            Description
//v2025-02-17-00     Initalizing versioning
//v2025-02-18-00     Created Move Selection
//v2025-02-18-01     Created PID Skills
//v2025-02-18-02     Created Left Right Alignment Button
//v2025-03-1-02      Added Wall Stakes
//v2025-03-4-00      Improved Wall Stakes and Finished PID backside
//v2025-03-14-00     ITS PI day !!! 3.1415926535 ...
//v2025-03-15-00     Added two new autons for positive
//v2025-04-5-00      Fixed Drivetrain 6 motors
//v2025-04-6-00      Added Motor Temp Readings
//v2025-04-29-00     Fixed Negative Side Autons
#include "vex.h"
#include <iostream>
using namespace vex;




// A global instance of competition
competition Competition;




// define your global instances of motors and other devices here
brain Brain;
controller Controller = controller(primary);

//const float PI=M_PI;
const float PI=3.142;

const float Wheeldiameter=88.9;
const float WheelCircumfrence=Wheeldiameter*PI;
const float wheelTravel=8.635;
const float trackwidth=355.6;
const float wheelbase=203.2;

int drivetrainspeed=100;
int slowdrivetrainspeed=70;
int fastdrivetrainspeed=100;

bool controllerMove = true;
bool doinkerOpen = false;
bool PID =true;

float pController,iController,integral,dController,derivitive,targetDistance,prevError,error,threshold,distanceTravel;

int tick,maxSpeed,speed;
int dT =20;

// define your global instances of motors and other devices here
motor Frontleftmotor = motor(PORT11,ratio6_1,true);//Updated
motor Frontrightmotor = motor(PORT15,ratio6_1);//Updated
motor Middleleftmotor= motor(PORT13,ratio6_1,true); //Updated
motor Middlerightmotor= motor(PORT17,ratio6_1);//Updated
motor Backleftmotor = motor(PORT16,ratio6_1,true);//Updated
motor Backrightmotor = motor(PORT18,ratio6_1);//Updated

motor_group LeftMotors {Frontleftmotor, Backleftmotor,Middleleftmotor };
motor_group RightMotors { Frontrightmotor, Backrightmotor,Middlerightmotor };
//motor_group LeftMotors {Frontleftmotor,Middleleftmotor };
//motor_group RightMotors { Frontrightmotor,Middlerightmotor };

inertial Inertial = inertial(PORT14,right);

smartdrive Drive = smartdrive(LeftMotors, RightMotors,Inertial,WheelCircumfrence,trackwidth,wheelbase, mm,1.0);

motor conveyer = motor(PORT19,false);//Updated
motor intake = motor(PORT12,ratio6_1); //Updated

pneumatics clamp = pneumatics(Brain.ThreeWirePort.H);
pneumatics doinker = pneumatics(Brain.ThreeWirePort.G);

// Construct a Rotation Sensor for the odometry
rotation Rotation = rotation(PORT20, true);

motor wallStake = motor(PORT2,ratio36_1);

//Type   Color/Side  Slot
//Type=0 Blue Right, Slot 1
//Type=1 Blue Left with Alliance Stake, Slot 2
//Type=2 Red Right with Alliance Stake, Slot 3
//Type=3 Red Left, Slot 4
//Type=4 Pid Skills, Slot 6
//Type=5 PID test, Slot 8
//Type=6 Blue Left without Alliance Stake, Slot 5
//Type=7 Red Right without Alliance Stake, Slot 7 
//Type=8 Red Right Goal Rush, Slot 8 
//Type=9 
//Type=10 
//Type=11
int type = 1;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/






void pre_auton(void) {

// All activities that occur before the competition starts
// Example: clearing encoders, setting servo positions, ...
Controller.Screen.print("V-25-04-29.00");
std::cout<< "Version 25-04-29.00"<<"\n";

conveyer.setVelocity(100,percent);
intake.setVelocity(100,percent);
wallStake.setVelocity(100,percent);
wallStake.setStopping(hold);

Drive.setStopping(coast);
// Set the drivetrain to drive at a velocity of 600 rpm.
Drive.setDriveVelocity(600,rpm);

clamp.close();
doinker.close();

// Start calibration.
Inertial.calibrate();
// Print that the Gyro is calibrating while waiting for it to
// finish calibrating.
while(Inertial.isCalibrating()){
 Brain.Screen.print("Gyro Calibrating");
 wait(50,msec);
}
Brain.Screen.newLine();
Inertial.setHeading(0,degrees);
Controller.Screen.print("Calibrated");
Brain.Screen.print("Calibrated");

}

void movePID(float distance, float accuracy, int topSpeed){

Rotation.resetPosition();

speed=integral=derivitive=distanceTravel=0;
prevError=error=targetDistance = distance;
threshold=accuracy;
maxSpeed=topSpeed;

//11,2,0.35

pController=11;
iController=2 * ((float)dT/1000);// after normalization it is 0.04
dController=0.35/ ((float)dT/1000);//used to be 0.4

std::cout<<"Debug Code \n";
std::cout<<"speed: "<<speed<<integral<<derivitive<<distanceTravel<<"\n";
std::cout<< "error/distance: "<<prevError<<error<<targetDistance<<distance<<"\n";
std::cout<<"threshold: "<<threshold<<"\n";
std::cout<<"max speed: "<<maxSpeed<<"\n";
std::cout<<"PID begin"<<"\n";

while (fabs(error)>threshold)
{

 //Calculate distance traveled
 double angle = Rotation.position(turns);
 distanceTravel = angle*wheelTravel;

 //Calculate Error
 prevError=error;
 error=targetDistance-distanceTravel;
  //Integral
 integral=integral+error;


  //If error is to large
 if (fabs(error)>6){
  integral=0;
 }
 //Prevent occilations
 if (error/fabs(error)==(-1*integral/fabs(integral))){
  integral=0;
 }
 //If error so large it is unusable
 if ((integral*iController)>100){
   integral = 100;
 }


 //Derivitive
 derivitive=(error-prevError);

 speed=(error*pController)+(integral*iController)+(derivitive*dController);


 //If speed is over max speed
  if(speed> maxSpeed){
   speed=maxSpeed;
 }
  if(speed<-maxSpeed){
   speed=-maxSpeed;
 }




 //Move Robot
 Drive.drive(forward,speed,rpm);


   if (tick%1==0)
   {
     std::cout << tick << ","<< speed << "," << distanceTravel << "," << (error*pController) <<"," << (integral*iController) << "," << (derivitive*dController) << "," << angle << "\n";
   }
  
 tick+=1;
wait(dT, msec); // Sleep the task for a short amount of time to
                // prevent wasted resources.
}
// Stop Drivetrain.
Drive.stop();
}


void move(float distance){
 if (PID)
 {
   movePID(distance,0.1,200);
 }
 else
 {
   Drive.driveFor(distance,inches);
 }
 }


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/




void autonomous(void) {

// ..........................................................................
// Insert autonomous user code here.
// ..........................................................................

if (type==0)
{

 //Sets Velocity
 Drive.setDriveVelocity(100,percent);//Og 60
 intake.setVelocity(100,percent);
 conveyer.setVelocity(100,percent);
Drive.setTurnVelocity(40,percent);//Og 40
Drive.setTurnConstant(0.5);
//Clamps the stake

 Drive.turnToHeading(330,degrees);//OG 35
movePID(-34,6,250);//Scores Preload

clamp.open();

 Drive.turnToHeading(265,degrees);//OG 90

 Drive.driveFor(forward,2,inches);
  

 conveyer.spin(forward);
 //Scores secondary ring

 Drive.setDriveVelocity(50,percent);//Og 60
 Drive.driveFor(forward,61,inches);
 conveyer.spin(forward);
 intake.spin(reverse);
 wait(0.5,seconds);
 Drive.driveFor(reverse,8,inches);

 Drive.turnToHeading(185,degrees);//OG 90
 Drive.setDriveVelocity(30,percent);//Og 60

 Drive.driveFor(forward,28,inches);//Og 17inches
 Drive.driveFor(reverse,8,inches);
 Drive.turnToHeading(130,degrees);//OG 90
 Drive.driveFor(forward,8,inches);//Og 4inches
 Drive.driveFor(reverse,3,inches);

 Drive.turnToHeading(80,degrees);

 Drive.setDriveVelocity(100,percent);

 Drive.driveFor(forward,50,inches);
 
}
if (type==1)
{
  Drive.setDriveVelocity(34,percent);
  intake.setVelocity(100,percent);
  conveyer.setVelocity(100,percent);
  Drive.setTurnVelocity(40,percent);
  Drive.setTurnConstant(0.8);
  clamp.close();
 
 
  Drive.turnToHeading(329,degrees); //OG 30
  movePID(5.5,0.05,250);
 
  wait(0.5,seconds);
 
  wallStake.spinToPosition(160,degrees);
  wallStake.spinToPosition(0,degrees);
  Drive.driveFor(reverse,4,inches);
 
  Drive.turnToHeading(335,degrees); //OG 25
 
 
  movePID(-39,0.1,500); //OG -35,0.1,250
  clamp.open();
 
  Drive.turnToHeading(90,degrees);
 
  intake.spin(reverse);
  conveyer.spin(forward);
 
  movePID(21,0.1,600);//og 16
  wait(1,sec);
  movePID(-10,5,600);//Not necassery if earlier move == OG
 
  Drive.turnToHeading(270,degrees);
  intake.stop();
  movePID(100,85,600);
  conveyer.stop();

}
if (type==2)
{


 Drive.setDriveVelocity(34,percent);
 intake.setVelocity(100,percent);
 conveyer.setVelocity(100,percent);
 Drive.setTurnVelocity(40,percent);
 Drive.setTurnConstant(0.8);
 clamp.close();


 Drive.turnToHeading(31,degrees); //OG 30
 movePID(5.5,0.05,250);

 wait(0.5,seconds);

 wallStake.spinToPosition(160,degrees);
 wallStake.spinToPosition(0,degrees);
 Drive.driveFor(reverse,4,inches);

 Drive.turnToHeading(25,degrees); //OG 25


 movePID(-39,0.1,500); //OG -35,0.1,250
 clamp.open();

 Drive.turnToHeading(270,degrees);

 intake.spin(reverse);
 conveyer.spin(forward);

 movePID(16,0.1,600);
 wait(0.5,sec);


 Drive.turnToHeading(90,degrees);
 intake.stop();
 movePID(100,85,600);
 conveyer.stop();

}
if (type==3)
{
 //Sets Velocity
 Drive.setDriveVelocity(100,percent);//Og 60
 intake.setVelocity(100,percent);
 conveyer.setVelocity(100,percent);
Drive.setTurnVelocity(40,percent);//Og 40
Drive.setTurnConstant(0.5);

//Clamps the stake
 Drive.turnToHeading(22,degrees);//OG 35
movePID(-34,6,200);//Scores Preload

clamp.open();


 Drive.turnToHeading(90,degrees);//OG 90

 Drive.driveFor(forward,2,inches);

 conveyer.spin(forward);
 //Scores secondary ring

 Drive.setDriveVelocity(50,percent);//Og 60



 Drive.driveFor(forward,61,inches);
 conveyer.spin(forward);
 intake.spin(reverse);
 wait(0.5,seconds);
 Drive.driveFor(reverse,8,inches);

 Drive.turnToHeading(167,degrees);//OG 90
 Drive.setDriveVelocity(30,percent);//Og 60

 Drive.driveFor(forward,33,inches);//Og 17inches
 Drive.driveFor(reverse,8,inches);

 Drive.turnToHeading(230,degrees);//OG 90
 Drive.driveFor(forward,12,inches);//Og 4inches

 Drive.turnToHeading(280,degrees);
 Drive.setDriveVelocity(100,percent);

 Drive.driveFor(forward,15,inches);
 Drive.turnToHeading(280,degrees);
 Drive.driveFor(forward,50,inches);
 

}

if (type==4)
{
 //wait for gyro initialization
 //wait(5000,msec);
 Inertial.setHeading(180,degrees);
 //setup
 Drive.setStopping(hold);
 Controller.Screen.print("Skills is Running");

 Drive.setDriveVelocity(50,percent);
 Drive.setTurnVelocity(40,percent);
 Drive.setTurnConstant(0.8);

 intake.setVelocity(100,percent);
 conveyer.setVelocity(100,percent);
 clamp.close();

 //score preload on alliance
 Drive.setDriveVelocity(30,percent);
 Drive.driveFor(reverse,3,inches);
 wallStake.spinToPosition(156,degrees);
 wallStake.spinToPosition(0,degrees);
 Drive.setDriveVelocity(50,percent);

 //Go to stake
 movePID(-5,0.1,200);
 Drive.turnToHeading(270,degrees);//OG 270
 movePID(-27,0.1,250);//OG 26

 //Clamp Stake
 clamp.open(); 
 movePID(9,0.1,200);//OG


 //Get ready to score
 Drive.turnToHeading(0,degrees);
 intake.spin(reverse);
 conveyer.spin(forward);

     //Score 4 rings
     Drive.turnToHeading(0,degrees);//OG 0 
     movePID(24,0.1,200);//OG 24

     Drive.turnToHeading(90,degrees);//OG 90
     movePID(23.5,0.1,200);
     
     Drive.turnToHeading(180,degrees); //OG 180
     movePID(26,0.1,200);

     wait(2000,msec);


     //Drop stake in corner
     Drive.turnToHeading(310,degrees);
     intake.stop();
     conveyer.stop();
     Drive.driveFor(forward,2,inches);
     clamp.close();
     movePID(-12,0.1,200);
     wait(500,msec);

     //Go back to line
     movePID(18,0.1,200);//OG 13

     //Long Stretch
     //heads 70 inches

       Drive.turnToHeading(89,degrees);//OG 88
      std::cout << Inertial.heading(degrees);

       movePID(-75,0.1,300);
       clamp.open();
       movePID(4,0.1,200);
       wait(200,msec); // helps the stake to settle down
     /*
      Drive.turnToHeading(89,degrees);//Should be 90 but has been tuned at 2/21
      std::cout << Inertial.heading(degrees);
      movePID(-23,0.1,150);
 
      Drive.turnToHeading(89,degrees);//Should be 90 but has been tuned at 2/21
      std::cout << Inertial.heading(degrees);
      movePID(-23,0.1,150);
       Drive.turnToHeading(90,degrees);//Should be 90 but has been tuned at 2/21
      std::cout << Inertial.heading(degrees);
      movePID(-24,0.1,150);
     */
     //Drive.setTurnThreshold(1.0);


     //Clamp Stake

     Drive.driveFor(forward,4,inches,true);




     //Get ready to score
     intake.spin(reverse);
     conveyer.spin(forward);


     //Score 4 rings
     Drive.turnToHeading(0,degrees);
     movePID(22,0.1,200); //used to be 24


     Drive.turnToHeading(270,degrees);
     movePID(21.5,0.1,200);//used to be 24


     Drive.turnToHeading(179,degrees);
     movePID(30,0.1,200);


     //Drop stake in corner
     Drive.turnToHeading(45,degrees); //45


     intake.stop();
     conveyer.stop();
     clamp.close();


     Drive.setTimeout(1, seconds);
     Drive.driveFor(reverse,15,inches);   
     Drive.setTimeout(99, seconds);
    
     
      Drive.driveFor(forward,8,inches);   


 Drive.turnToHeading(0,degrees);
  Drive.driveFor(forward,55,inches);


 Drive.turnToHeading(215,degrees);
  Drive.driveFor(reverse,72,inches);


 Drive.turnToHeading(120,degrees);


  Drive.setTimeout(3, seconds);
 Drive.driveFor(reverse,80,inches);
  Drive.setTimeout(99, seconds);


 Drive.driveFor(forward,40,inches);


 Drive.turnToHeading(260,degrees);
 Drive.setTimeout(3, seconds);
 Drive.driveFor(reverse,200,inches);
 Drive.driveFor(forward,1000,inches);


     //Initialize for next attempt
     Drive.setDriveVelocity(slowdrivetrainspeed,percent);
     Drive.setTurnVelocity(slowdrivetrainspeed,percent);
     clamp.close();

}
if (type==5)
{


   //setup
 Drive.setStopping(hold);
 Drive.setDriveVelocity(100,percent);
 Drive.setTurnVelocity(100,percent);
 Drive.setTurnConstant(1.0);
 Drive.setTurnThreshold(1.0);

 move(20);
 float curving=Inertial.heading();

 std::cout<<"\n"<<curving<<"\n";

}

if (type==6)
{


 //Sets Velocity
 Drive.setDriveVelocity(33,percent);
 intake.setVelocity(100,percent);
 conveyer.setVelocity(100,percent);
 Drive.setTurnVelocity(40,percent);
 Drive.setTurnConstant(0.8);
 clamp.close();
 //Clamps the stake
 Drive.driveFor(reverse,20,inches);
 
 //wait(2500,msec);
 
 Drive.turnToHeading(32,degrees);
 Drive.setDriveVelocity(20,percent);//OG 20 percent

 movePID(-19,1.1,300);
  clamp.open();
 
  //Scores Preload
  Drive.driveFor(forward,6,inches);
 
  wait(.5,seconds);
    Drive.setDriveVelocity(33,percent);
 
 conveyer.spin(forward);
 wait(1,seconds);
 conveyer.stop();
 
 Drive.driveFor(forward,12,inches);
 
 intake.spin(reverse);
 conveyer.spin(forward);
 
 
 Drive.setDriveVelocity(10,percent);
 Drive.turnToHeading(137,degrees);//OG 48
 movePID(16,0.1,600);
 
 wait(4,seconds);
 conveyer.stop();
 intake.stop();
 
 Drive.setDriveVelocity(100,percent);//OG 20 percent
}
if (type==7)
{

 //Sets Velocity
Drive.setDriveVelocity(33,percent);
intake.setVelocity(100,percent);
conveyer.setVelocity(100,percent);
Drive.setTurnVelocity(40,percent);
Drive.setTurnConstant(0.8);
clamp.close();
//Clamps the stake
Drive.driveFor(reverse,20,inches);

//wait(2500,msec);

Drive.turnToHeading(330,degrees);
Drive.setDriveVelocity(20,percent);//OG 20 percent

Drive.driveFor(reverse,26,inches);
 clamp.open();

 //Scores Preload
 Drive.driveFor(forward,6,inches);

 wait(.5,seconds);
   Drive.setDriveVelocity(33,percent);

conveyer.spin(forward);
wait(1,seconds);
conveyer.stop();

Drive.driveFor(forward,12,inches);

intake.spin(reverse);
conveyer.spin(forward);


Drive.setDriveVelocity(10,percent);
Drive.turnToHeading(220,degrees);//OG 48
Drive.driveFor(forward,25,inches);

wait(4,seconds);
conveyer.stop();
intake.stop();

Drive.setDriveVelocity(100,percent);//OG 20 percent
}
if(type==8){

}
}




/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/




void usercontrol(void) {
// User control code here, inside the loop

conveyer.setVelocity(100,percent);
intake.setVelocity(100,percent);
wallStake.setVelocity(100,percent);
wallStake.setStopping(hold);

Drive.setStopping(coast);
// Set the drivetrain to drive at a velocity of 600 rpm.
Drive.setDriveVelocity(600,rpm);

doinker.close();



while (1) {
  // This is the main execution loop for the user control program.
  // Each time through the loop your program should update motor + servo
  // values based on feedback from the joysticks.


  // ........................................................................
  // Insert user code here. This is where you use the joystick values to
  // update your motors, etc.
  // ........................................................................


  if(controllerMove){
    Drive.arcade(Controller.Axis3.value()*drivetrainspeed/100,Controller.Axis1.value()*drivetrainspeed/200);
  }


  wait(20, msec); // Sleep the task for a short amount of time to
                  // prevent wasted resources.
}
}

void ButtonR1Pressed (){
conveyer.spin(forward);
intake.spin(reverse);
}


void StopButtonR1(){
conveyer.stop();
intake.stop();
}


void ButtonR2Pressed (){
conveyer.spin(reverse);
intake.spin(forward);
}


void StopButtonR2(){
conveyer.stop();
intake.stop();
}


void ButtonL1Pressed(){
clamp.open();
}


void ButtonL2Pressed(){
clamp.close();
}


void ButtonUpPressed()
{
 if (doinkerOpen)
 {
   doinker.close();
   doinkerOpen=false;
   std::cout<<"close";


 }
 else
 {
   doinker.open();
   doinkerOpen=true;
   std::cout<<"open";


 }
}


void ButtonXPressed(){
 // Opens Motor completely
  wallStake.setVelocity(80,percent);
  conveyer.spinFor(reverse,15,degrees);
  wallStake.spinToPosition(156,degrees); // Might need to get tuned
  std::cout<<wallStake.temperature(celsius)<<"\n";
}


void ButtonYPressed(){
  //Opens Motor to Pick up ring
  wallStake.setVelocity(100,percent);
  wallStake.spinToPosition(34,degrees); // Might need to get tuned
}
void ButtonBPressed(){
  //Resets Wallstake
  wallStake.setVelocity(60,percent);
  wallStake.spinToPosition(0,degrees);
}
void ButtonAPressed(){
//Prints Motor Temps
Brain.Screen.print("Frontleftmotor : ");
Brain.Screen.print(Frontleftmotor.temperature(celsius));
Brain.Screen.newLine();

Brain.Screen.print("Frontrightmotor : ");
Brain.Screen.print(Frontrightmotor.temperature(celsius));
Brain.Screen.newLine();

Brain.Screen.print("Middleleftmotor : ");
Brain.Screen.print(Middleleftmotor.temperature(celsius));
Brain.Screen.newLine();

Brain.Screen.print("Middlerightmotor : ");
Brain.Screen.print(Middlerightmotor.temperature(celsius));
Brain.Screen.newLine();

Brain.Screen.print("Backleftmotor : ");
Brain.Screen.print(Backleftmotor.temperature(celsius));
Brain.Screen.newLine();

Brain.Screen.print("Backrightmotor : ");
Brain.Screen.print(Backrightmotor.temperature(celsius));
Brain.Screen.newLine();

}


//
// Main will set up the competition functions and callbacks.
//
int main() {
// Set up callbacks for autonomous and driver control periods.

Competition.autonomous(autonomous); //initializes Auton
Competition.drivercontrol(usercontrol); //initializes Driver Control

Controller.ButtonR1.pressed(ButtonR1Pressed);// Forward (Intake + Conveyer)
Controller.ButtonR1.released(StopButtonR1);//Stop (Intake + Conveyer)
Controller.ButtonR2.pressed(ButtonR2Pressed);// Reverse (Conveyer + Intake)
Controller.ButtonR2.released(StopButtonR2);//Stop (Conveyer + Intake)

Controller.ButtonUp.pressed(ButtonUpPressed); //Doinker Open/Close

Controller.ButtonL1.pressed(ButtonL1Pressed);//Clamp Close
Controller.ButtonL2.pressed(ButtonL2Pressed);// Clamp Open

Controller.ButtonA.pressed (ButtonAPressed); //Reads the Motot Temperature

Controller.ButtonX.pressed(ButtonXPressed); //Scores Wallstake
Controller.ButtonY.pressed(ButtonYPressed); //Get Ready to Score Wallstake
Controller.ButtonB.pressed(ButtonBPressed); //Resets Wallstake


// Run the pre-autonomous function.
pre_auton();


// Prevent main from exiting with an infinite loop.
while (true) {
  wait(100, msec);
}
}
