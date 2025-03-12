/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Advik Prabhu and Atharv Bagotra                                               */
/*    Created:      2/3/2025, 9:07:19 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/


//Version            Description
//v2025-02-17-01     Initalizing versioning
//v2025-02-18-01     Created Move Selection
//v2025-02-18-01     Created PID Skills
//v2025-02-18-02     Created Left Right Alignment Button
//v2025-03-1-02      Added Wall Stakes
//v2025-03-4-00      Improved Wall Stakes and Finished PID backside


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




// Construct a Rotation Sensor for the odometry
rotation Rotation = rotation(PORT18, true);


motor wallStake = motor(PORT2,ratio36_1);




//Type=0 Blue Right, Slot 1
//Type=1 Blue Left, Slot 2
//Type=2 Red Right, Slot 3
//Type=3 Red Left, Slot 4
//Type=4 PID Skills, Slot 6
//Type=5 PID test, Slot 8


int type = 4;








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
Controller.Screen.print("V-25-03-03.09");
std::cout<< "Version 25-03-03.09"<<"\n";




conveyer.setVelocity(100,percent);
intake.setVelocity(100,percent);
wallStake.setVelocity(100,percent);
wallStake.setStopping(hold);


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





pController=11;
iController=2 * ((float)dT/1000);// after normalization it is 0.04
dController=0.3/ ((float)dT/1000);// after normalization it is 10




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
     std::cout << tick << ","<< speed << "," << distanceTravel << "," << (error*pController) <<"," << (integral*iController) << "," << (derivitive*dController) << "," << angle << "\n" ;
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
   movePID(distance,0.1,170);
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




Drive.setDriveVelocity(60,percent);//Og 60
intake.setVelocity(100,percent);
conveyer.setVelocity(100,percent);
Drive.setTurnVelocity(40,percent);//Og 40
Drive.setTurnConstant(1.2);
//Clamps the stake  




Drive.turnToHeading(333,degrees);//OG 35
Drive.driveFor(reverse,43,inches);
// Drive.driveFor(forward,2,inches);


 clamp.close();
 Drive.driveFor(forward,3,inches);




  //Scores Preload




conveyer.spin(forward);
//Scores secondary ring




 Drive.turnToHeading(280,degrees);//OG 260


 Drive.setDriveVelocity(80,percent);//Og 60




Drive.driveFor(forward,40,inches);
conveyer.spin(forward);
intake.spin(reverse);
 Drive.driveFor(reverse,8,inches);


 Drive.turnToHeading(180,degrees);//OG 90
Drive.setDriveVelocity(20,percent);//Og 60


 Drive.driveFor(forward,15,inches);//Og 17inches
Drive.driveFor(reverse,4,inches);
Drive.turnToHeading(155,degrees);//OG 90
Drive.driveFor(forward,7,inches);//Og 4inches
Drive.turnToHeading(80,degrees);
Drive.setDriveVelocity(75,percent);
Drive.driveFor(forward,18,inches);


Drive.turnToHeading(100,degrees);




 }
if (type==1)
{
//Sets Velocity
Drive.setDriveVelocity(60,percent);
intake.setVelocity(100,percent);
conveyer.setVelocity(100,percent);
Drive.setTurnVelocity(40,percent);
Drive.setTurnConstant(0.8);
//Clamps the stake
Drive.driveFor(reverse,16,inches);




Drive.turnToHeading(30,degrees);
Drive.setDriveVelocity(40,percent);//OG 20 percent


Drive.driveFor(reverse,25,inches);
 clamp.close();
 //Scores Preload
//  Drive.driveFor(forward,5,inches);
 wait(.5,seconds);
   Drive.setDriveVelocity(60,percent);






conveyer.spin(forward);
wait(1,seconds);


clamp.open();




conveyer.stop();
Drive.driveFor(forward,12,inches);


Drive.turnToHeading(330,degrees);//OG 48
Drive.setDriveVelocity(40,percent);//OG 20 percent


Drive.driveFor(reverse,29.5,inches);//Clamps middle stake aka really risky
clamp.close();


Drive.turnToHeading(10,degrees);
// Drive.driveFor(forward, 5 ,inches);//New addition start
// clamp.open();
// Drive.driveFor(reverse, 5 ,inches);//New addition end
// clamp.close();
Drive.setDriveVelocity(60,percent);


intake.spin(reverse);
conveyer.spin(forward);
Drive.driveFor(forward,15.5,inches);
Drive.turnToHeading(215,degrees);
clamp.open();
Drive.setDriveVelocity(100,percent);


Drive.driveFor(reverse,80,inches);






}
if (type==2)
{


//Sets Velocity
  Drive.setDriveVelocity(60,percent);
  intake.setVelocity(100,percent);
  conveyer.setVelocity(100,percent);
 Drive.setTurnVelocity(40,percent);
Drive.setTurnConstant(0.8);
//Clamps the stake
Drive.driveFor(reverse,16,inches);




  Drive.turnToHeading(330,degrees);
  Drive.setDriveVelocity(40,percent);//OG 20 percent


  Drive.driveFor(reverse,25,inches);
   clamp.close();
   //Scores Preload
  //  Drive.driveFor(forward,5,inches);
   wait(.5,seconds);
     Drive.setDriveVelocity(60,percent);






  conveyer.spin(forward);
  wait(1,seconds);


clamp.open();




conveyer.stop();
Drive.driveFor(forward,12,inches);


Drive.turnToHeading(30,degrees);//OG 48
Drive.setDriveVelocity(40,percent);//OG 20 percent


Drive.driveFor(reverse,29,inches);
clamp.close();


Drive.turnToHeading(0,degrees);
// Drive.driveFor(forward, 5 ,inches);//New addition start
// clamp.open();
// Drive.driveFor(reverse, 5 ,inches);//New addition end
// clamp.close();
Drive.setDriveVelocity(60,percent);


intake.spin(reverse);
conveyer.spin(forward);
Drive.driveFor(forward,14,inches);
Drive.turnToHeading(145,degrees);
clamp.open();
Drive.setDriveVelocity(100,percent);


Drive.driveFor(reverse,80,inches);






}
if (type==3)
{


 //Sets Velocity




 Drive.setDriveVelocity(60,percent);//Og 60
 intake.setVelocity(100,percent);
 conveyer.setVelocity(100,percent);
Drive.setTurnVelocity(40,percent);//Og 40
Drive.setTurnConstant(1.2);
//Clamps the stake




 Drive.turnToHeading(28,degrees);//OG 35
 Drive.driveFor(reverse,40,inches);
  clamp.close();
   //Scores Preload




 conveyer.spin(forward);
 //Scores secondary ring




 Drive.turnToHeading(80,degrees);//OG 90


 Drive.setDriveVelocity(100,percent);//Og 60




 Drive.driveFor(forward,43,inches);
 conveyer.spin(forward);
 intake.spin(reverse);
 Drive.driveFor(reverse,8,inches);


 Drive.turnToHeading(180,degrees);//OG 90
 Drive.setDriveVelocity(20,percent);//Og 60


 Drive.driveFor(forward,15,inches);//Og 17inches
 Drive.driveFor(reverse,4,inches);
 Drive.turnToHeading(205,degrees);//OG 90
 Drive.driveFor(forward,7,inches);//Og 4inches
 Drive.turnToHeading(280,degrees);
 conveyer.stop();
 intake.stop();
 Drive.setDriveVelocity(75,percent);
 Drive.driveFor(forward,20,inches);








}
if (type==4)
{

 //wait for gyro initialization
 //wait(5000,msec);



 //setup
 Drive.setStopping(hold);
 Controller.Screen.print("Skills is Running");


 Drive.setDriveVelocity(100,percent);
 Drive.setTurnVelocity(90,percent);
 Drive.setTurnConstant(1.0);
 Drive.setTurnThreshold(1.0);


 intake.setVelocity(100,percent);
 conveyer.setVelocity(100,percent);
 clamp.open();


 //score preload on alliance


 conveyer.spin(forward);
 wait(1000,msec);
 conveyer.stop();


 //Go to stake
 move(10);
 Drive.turnToHeading(270,degrees);
 move(-21);


 //Clamp Stake
 clamp.close();
 Drive.driveFor(reverse,2,inches,true);


 //Get ready to score
 Drive.turnToHeading(0,degrees);
 intake.spin(reverse);
 conveyer.spin(forward);


     //Score 4 rings
     Drive.turnToHeading(0,degrees);
      move(24);


     Drive.turnToHeading(90,degrees);


     move(21.5);
     Drive.turnToHeading(180,degrees);
     move(26);


     wait(2000,msec);


    


     conveyer.stop();


     //Drop stake in corner
     Drive.turnToHeading(310,degrees);
     intake.stop();
     Drive.driveFor(forward,2,inches);
     clamp.open();
     move(-12);
     wait(500,msec);


     //Go back to line
     move(13);
  
     //Long Stretch
     // heads 70 inches


       Drive.turnToHeading(88,degrees);
        std::cout << Inertial.heading(degrees);



       movePID(-72,0.1,200);
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
     clamp.close();


     Drive.driveFor(forward,4,inches,true);




     //Get ready to score
     intake.spin(reverse);
     conveyer.spin(forward);


     //Score 4 rings
     Drive.turnToHeading(0,degrees);
     move(22); //used to be 24


     Drive.turnToHeading(270,degrees);
     move(21.5);//used to be 24


     Drive.turnToHeading(179,degrees);
     move(30);


     //Drop stake in corner
     Drive.turnToHeading(45,degrees); //45


     intake.stop();
     conveyer.stop();
     clamp.open();


     Drive.setTimeout(1, seconds);
     Drive.driveFor(reverse,15,inches);   
     Drive.setTimeout(99, seconds);
    
     
      Drive.driveFor(forward,8,inches);   


 Drive.turnToHeading(180,degrees);
  Drive.driveFor(reverse,55,inches);


 Drive.turnToHeading(215,degrees);
  Drive.driveFor(reverse,68,inches);


 Drive.turnToHeading(130,degrees);


  Drive.setTimeout(3, seconds);
 Drive.driveFor(reverse,80,inches);
  Drive.setTimeout(99, seconds);


 Drive.driveFor(forward,40,inches);


 Drive.turnToHeading(240,degrees);
 Drive.driveFor(reverse,200,inches);


     //Initialize for next attempt
     Drive.setDriveVelocity(slowdrivetrainspeed,percent);
     Drive.setTurnVelocity(slowdrivetrainspeed,percent);
     clamp.open();

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
   std::cout<<"close";


 }
 else
 {
   doinker.open();
   doinkerOpen=true;
   std::cout<<"open";


 }
}






void ButtonLeftPressed(){
 Drive.setTurnVelocity(40,percent);
 Drive.setTurnConstant(0.8);
 controllerMove=false;
 Drive.turnToHeading(270,degrees); 
 controllerMove=true;
}


void ButtonRightPressed(){
 Drive.setTurnVelocity(40,percent);
 Drive.setTurnConstant(0.8);
 controllerMove=false;
 Drive.turnToHeading(90,degrees); 
 controllerMove=true;
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
  wallStake.spinToPosition(40,degrees); // Might need to get tuned
}
void ButtonBPressed(){
  //Resets Wallstake
  wallStake.setVelocity(60,percent);
  wallStake.spinToPosition(0,degrees);
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
Controller.ButtonLeft.pressed (ButtonLeftPressed); //Aligns the Robot left
Controller.ButtonRight.pressed (ButtonRightPressed); //Aligns the robot right


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
