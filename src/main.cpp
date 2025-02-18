/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       advikprabhu                                               */
/*    Created:      2/3/2025, 9:07:19 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

//Version     Description
//v2025-02-17 Initalizing versioning 

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
int slowdrivetrainspeed=80;
int fastdrivetrainspeed=100;


bool controllerMove = true;
bool doinkerOpen = false;


float pController,iController,integral,dController,derivitive,targetDistance,prevError,error,threshold,distanceTravel;


int dT,tick,maxSpeed,speed;


// define your global instances of motors and other devices here
motor Frontleftmotor = motor(PORT11, false);//Updated
motor Frontrightmotor = motor(PORT15, true);//Updated
//motor Backleftmotor = motor(PORT16, false);//Updated
//motor Backrightmotor = motor(PORT14, true);//Updated
motor Middleleftmotor= motor(PORT13, false); //Updated
motor Middlerightmotor= motor(PORT20, true);//Updated


//motor_group LeftMotors {Frontleftmotor, Backleftmotor,Middleleftmotor };
//motor_group RightMotors { Frontrightmotor, Backrightmotor,Middlerightmotor };
motor_group LeftMotors {Frontleftmotor,Middleleftmotor };
motor_group RightMotors { Frontrightmotor,Middlerightmotor };


inertial Inertial = inertial(PORT10,right);


smartdrive Drive = smartdrive(LeftMotors, RightMotors,Inertial,WheelCircumfrence,trackwidth,wheelbase, mm,1.0);


motor conveyer = motor(PORT19,false);//Updated
motor intake = motor(PORT12,ratio6_1); //Updated


pneumatics clamp = pneumatics(Brain.ThreeWirePort.H);
pneumatics doinker = pneumatics(Brain.ThreeWirePort.G);


// Construct a Rotation Sensor "Rotation" with the
// rotation class.
rotation Rotation = rotation(PORT18, false);




//Type=0 Blue Right, Slot 1
//Type=1 Blue Left, Slot 2
//Type=2 Red Right, Slot 3
//Type=3 Red Left, Slot 4
//Type=4 Skills, Slot 6
//Type=5 PID Test, Slot 7


int type = 5;




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
 Controller.Screen.print("V-25-02-17.00");
 std::cout<< "Version 25-02-17.00"<<"\n";


conveyer.setVelocity(100,percent);
intake.setVelocity(100,percent);


clamp.open();
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

pController=7;
iController=2*((float)dT/1000);
dController=0.1/((float)dT/1000);


std::cout<<"Debug Code \n";
std::cout<<"speed: "<<speed<<integral<<derivitive<<distanceTravel<<"\n";
std::cout<< "error/distance: "<<prevError<<error<<targetDistance<<distance<<"\n";
std::cout<<"threshold: "<<threshold<<"\n";
std::cout<<"max speed: "<<maxSpeed<<"\n";

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
       std::cout << tick << ","<< speed <<","<<distanceTravel<< "," <<(error*pController)<<","<< (integral*iController)<<","<<(derivitive*dController)<<","<<angle<<"\n";
    }
    
  tick+=1;
 wait(dT, msec); // Sleep the task for a short amount of time to
                 // prevent wasted resources.
}
// Stop Drivetrain.
Drive.stop();
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
   //Sets Speed
  Drive.setDriveVelocity(60,percent);
  intake.setVelocity(100,percent);
  conveyer.setVelocity(100,percent);

  //go to stake
  Drive.driveFor(forward, 36, inches);
  Drive.turnToHeading(90,degrees);
  Drive.driveFor(reverse,24,inches);

  //clamp stake
  clamp.close();
  conveyer.spin(forward);
  //touch ladder
  Drive.turnToHeading(270,degrees);
  Drive.driveFor(forward,28,inches);


  
}
if (type==1)
{

 //Sets Speed
  Drive.setDriveVelocity(60,percent);
  intake.setVelocity(100,percent);
  conveyer.setVelocity(100,percent);

  //go to stake
  Drive.driveFor(forward, 36, inches);
  Drive.turnToHeading(270,degrees);
  Drive.driveFor(reverse,24,inches);

  //clamp stake
  clamp.close();
  conveyer.spin(forward);
  //touch ladder
  Drive.turnToHeading(90,degrees);
  Drive.driveFor(forward,28,inches);



}
if (type==2)
{
  //Sets Speed
  Drive.setDriveVelocity(60,percent);
  intake.setVelocity(100,percent);
  conveyer.setVelocity(100,percent);

  //go to stake
  Drive.driveFor(forward, 36, inches);
  Drive.turnToHeading(90,degrees);
  Drive.driveFor(reverse,24,inches);

  //clamp stake
  clamp.close();
  conveyer.spin(forward);
  
  //touch ladder
  Drive.turnToHeading(270,degrees);
  Drive.driveFor(forward,28,inches);

}
if (type==3)
{

 //Sets Speed
  Drive.setDriveVelocity(60,percent);
  intake.setVelocity(100,percent);
  conveyer.setVelocity(100,percent);

  //go to stake
  Drive.driveFor(forward, 36, inches);
  Drive.turnToHeading(270,degrees);
  Drive.driveFor(reverse,24,inches);

  //clamp stake
  clamp.close();
  conveyer.spin(forward);
  
  //touch ladder
  Drive.turnToHeading(90,degrees);
  Drive.driveFor(forward,28,inches);





}








if (type==4)
{
    //TODO fix scoring after long stretch



  //wait for gyro initialization




  //wait(5000,msec);




  //setup
  Drive.setStopping(hold);
  Controller.Screen.print("Skills is Running");
  Drive.setDriveVelocity(60,percent);
  Drive.setTurnVelocity(15,percent);
  intake.setVelocity(100,percent);
  conveyer.setVelocity(100,percent);
  clamp.open();
  //score preload on alliance




  conveyer.spin(forward);
  wait(1000,msec);
  conveyer.stop();




   //Go to stake
  Drive.driveFor(forward, 12.5, inches);
  Drive.turnToHeading(270,degrees);
  Drive.driveFor(reverse, 24, inches);




  clamp.close();




  //Get ready to score
  Drive.turnToHeading(0,degrees);
  intake.spin(reverse);
  conveyer.spin(forward);




      //Score 4 rings
      Drive.turnToHeading(0,degrees);
      Drive.driveFor(forward, 26, inches);

      Drive.turnToHeading(90,degrees);

      Drive.driveFor(forward, 26.5, inches);
      Drive.turnToHeading(180,degrees);

      Drive.driveFor(forward, 27, inches);
      wait(2000,msec);
      conveyer.stop();

      //Drop stake in corner
      Drive.turnToHeading(310,degrees);
      intake.stop();
      Drive.driveFor(reverse,14, inches);
      clamp.open();
      wait(500,msec);
      //Go back to line
      Drive.driveFor(forward,14,inches);




      //Go to other side
      Drive.setTurnVelocity(10,percent);
      Drive.setDriveVelocity(50,percent);

    
      //Long Stretch
      // heads 70 inches

      std::cout << Inertial.heading(degrees);
      Drive.turnToHeading(90,degrees);
      std::cout << Inertial.heading(degrees);

      Drive.driveFor(reverse,30,inches);


      Drive.turnToHeading(90,degrees);
      Drive.driveFor(reverse,40,inches);

      //Set speed
      Drive.setTurnVelocity(15,percent);
      Drive.setDriveVelocity(60,percent);



      //Clamp Stake
      clamp.close();



      //Get ready to score
      intake.spin(reverse);
      conveyer.spin(forward);




      //Score 4 rings
      Drive.turnToHeading(0,degrees);
      Drive.driveFor(forward,24,inches);



      Drive.turnToHeading(270,degrees);
      Drive.driveFor(forward,26,inches);


      Drive.turnToHeading(180,degrees);
      Drive.driveFor(forward,28,inches); //27



      //Drop stake in corner
      Drive.turnToHeading(50,degrees); //45




       Drive.drive(reverse);
       wait(1000,msec);
       Drive.stop();
         clamp.open();


       Drive.drive(forward);
       wait(750,msec);
       Drive.stop();




      //Initialize for next attempt
      Drive.setDriveVelocity(slowdrivetrainspeed,percent);
      Drive.setTurnVelocity(slowdrivetrainspeed,percent);
      clamp.open();
}
if (type==5)
{
 
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
 while (1) {
   // This is the main execution loop for the user control program.
   // Each time through the loop your program should update motor + servo
   // values based on feedback from the joysticks.


   // ........................................................................
   // Insert user code here. This is where you use the joystick values to
   // update your motors, etc.
   // ........................................................................


   if(controllerMove){
     Drive.arcade(Controller.Axis3.value()*drivetrainspeed/100,Controller.Axis1.value()*drivetrainspeed/100);
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
clamp.close();
}

void ButtonL2Pressed(){
clamp.open();
}

void ButtonUpPressed()
{
  if (doinkerOpen)
  {
    doinker.close();
    doinkerOpen=false;


  }
  else
  {
   doinker.open();
  doinkerOpen=true;
  }
}


void ButtonBPressed()
{
drivetrainspeed=slowdrivetrainspeed;
}


void ButtonXPressed()
{
drivetrainspeed=fastdrivetrainspeed;
}



//TODO add buttons to orient the robots left and right






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
 Controller.ButtonX.pressed (ButtonXPressed); //Speed Up Robot
 Controller.ButtonB.pressed (ButtonBPressed); //Slow Down Robot


 // Run the pre-autonomous function.
 pre_auton();


 // Prevent main from exiting with an infinite loop.
 while (true) {
   wait(100, msec);
 }
}



