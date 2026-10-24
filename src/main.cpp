/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "ai_functions.h"

using namespace vex;

brain Brain;
// Robot configuration code.
motor leftFront = motor(PORT2, ratio18_1, true);
motor leftMiddle = motor(PORT11, ratio18_1, true);
motor leftBack = motor(PORT13, ratio18_1, true);
motor rightFront = motor(PORT10, ratio18_1, false);
motor rightMiddle = motor(PORT18, ratio18_1, false);
motor rightBack = motor(PORT19, ratio18_1, false);
//2,11,13
//10, 18,19
// motor leftDrive = motor(PORT1, ratio18_1, true);
// motor rightDrive = motor(PORT21, ratio18_1, true);

motor primaryIntake = motor(PORT12, ratio6_1, false);
motor secondaryIntake = motor(PORT20, ratio6_1, true);
motor thirdIntake = motor(PORT17, ratio6_1, true);

triport Triport1 = triport(PORT3);
digital_out armgrab = digital_out(Triport1.D);
digital_out goalgrabber = digital_out(Triport1.A);
distance ds = distance(PORT6);
gps gpsmain = gps(PORT1, 180);

// gps GPS = gps(PORT15, -127, -165, distanceUnits::mm, 180);
//smartdrive Drivetrain = smartdrive(leftDrive, rightDrive, GPS, 319.19, 320, 40, mm, 1);
// Controls arm used for raising and lowering rings
// motor Arm = motor(PORT3, ratio18_1, false);
// Controls the chain at the front of the arm
// used for pushing rings off of the arm
// motor Chain = motor(PORT8, ratio18_1, false);

inertial imu = inertial(PORT14);


// A global instance of competition
competition Competition;

// create instance of jetson class to receive location and other
// data from the Jetson nano
//
ai::jetson  jetson_comms;


Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
ZERO_TRACKER_NO_ODOM,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(leftFront,leftMiddle,leftBack),

//Right Motors:
motor_group(rightFront,rightMiddle,rightBack),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT14,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
1.66667,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
358.8,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-2,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

/*----------------------------------------------------------------------------*/
// Create a robot_link on PORT1 using the unique name robot_32456_1
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot
#define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT6, "robot_32456_1", linkType::manager );
#else
#pragma message("building for the worker")
ai::robot_link       link( PORT6, "robot_32456_1", linkType::worker );
#endif

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(8, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(6, .45, 0, 3.15, 15); //4.25 
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1.5, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

// void default_constants(){
//   // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
//   chassis.set_drive_constants(8, 1.5, 0, 10, 0);
//   chassis.set_heading_constants(6, .4, 0, 1, 0);
//   chassis.set_turn_constants(6, .4235, 0, 3.15, 15); //4.25 
//   chassis.set_swing_constants(12, .3, .001, 2, 15);

//   // Each exit condition set is in the form of (settle_error, settle_time, timeout).
//   chassis.set_drive_exit_conditions(1.5, 300, 3000);
//   chassis.set_turn_exit_conditions(1.5, 300, 3000);
//   chassis.set_swing_exit_conditions(1, 300, 3000);
// }
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Auto_Isolation Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous isolation  */
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void clampone(void* param) {
    chassis.drive_distance(-36);
    return;
}

void clamptwo(void* param) {
    chassis.drive_distance(-39);
    return;
}

bool intakeOn = false;
int intakeTask(){
  int stuckCounter = 0;
  while (true) { 
    if(intakeOn){
      primaryIntake.spin(forward, 100, percent);
      secondaryIntake.spin(forward, 100, percent);
      thirdIntake.spin(forward, 100, percent);

      if(thirdIntake.torque(Nm)>0.33 ){
        stuckCounter ++;
        if(stuckCounter > 5){
          printf("Reverse \n");
          thirdIntake.spin(forward,-50,percent);
          wait(0.4,sec);
          stuckCounter = 0;
        }
      }
      else{
        stuckCounter = 0;
      }


    } 
    else {
      //primaryIntake.stop();
      secondaryIntake.stop();
      thirdIntake.stop();
    }
    wait(20,msec);
  }
}





void auto_Isolation(void) {
  // //GPS.calibrate();
  // //waitUntil(!(GPS.isCalibrating()));
  //initialization

  default_constants();
  imu.setHeading(270, degrees);
  chassis.set_heading(270);

  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_turn_constants(8, .425, .03, 3, 15);
  chassis.set_turn_constants(8, .45, 0, 3.15, 15); //4.25 


  chassis.turn_to_angle(300); //+60
  chassis.set_drive_constants(6, 1.5, 0, 10, 0);
  thread clampback(clampone,nullptr);
  wait(1250,msec);
  goalgrabber.set(true); //grab first mobile goal


  chassis.set_drive_constants(8, 1.5, 0, 10, 0);
  //ntakeOn=true;
  intakeOn = true;
  wait(200,msec);
  chassis.turn_to_angle(90);
  chassis.drive_distance(24);  //first ring
  wait(300,msec);
  chassis.drive_distance(-24);
  wait(200,msec);
  chassis.turn_to_angle(272.5);
  chassis.drive_distance(26);
  intakeOn = true;
  wait(400,msec);

  chassis.turn_to_angle(44);
  
  chassis.drive_distance(46);
  wait(300,msec);
  chassis.turn_to_angle(31);
  wait(300,msec);
  armgrab.set(true); 
  wait(500,msec);
  chassis.turn_to_angle(34);
  chassis.drive_distance(-27);
  chassis.turn_to_angle(37);
  armgrab.set(false);
  wait(200,msec);
  chassis.turn_to_angle(49);
  chassis.drive_distance(17);
  intakeOn = true; 
  wait(300,msec);
  
  chassis.set_drive_constants(6, 1.5, 0, 10, 0);
  wait(500,msec);
  primaryIntake.spin(forward,100,percent);
  wait(200,msec);
  intakeOn = false;
  wait(100,msec);
  chassis.turn_to_angle(19);
  chassis.drive_distance(16);
  wait(200,msec);

  chassis.turn_to_angle(52);
  armgrab.set(true);
  wait(500,msec);
  chassis.drive_distance(-30);
  armgrab.set(false);
  chassis.turn_to_angle(59);
  wait(100,msec);
  intakeOn = true;
  chassis.drive_distance(15);
  wait(650,msec);

  chassis.turn_to_angle(245);
  wait(200,msec);
  chassis.drive_distance(5);
  chassis.turn_to_angle(225);
  wait(200,msec);
  chassis.drive_distance(54.5);
  wait(400,msec);

  chassis.drive_distance(-9);
  wait(200,msec);
  chassis.turn_to_angle(45);
  wait(300,msec);
  chassis.drive_distance(-6);
  wait(500,msec);
  goalgrabber.set(false);

  //working part start
  intakeOn = false;
  chassis.drive_distance(18); 
  primaryIntake.spin(forward,100,percent);
  chassis.turn_to_angle(102);
  chassis.set_drive_constants(8, 1.5, 0, 10, 0); //

  chassis.drive_distance(54);  //cross to blue side
  wait(300,msec);
  chassis.set_drive_constants(10, 1.5, 0, 10, 0); //

  intakeOn = false;
  chassis.turn_to_angle(326); //214
  chassis.drive_distance(14.5);
  chassis.turn_to_angle(228); //232 //308
  chassis.set_drive_constants(6, 1.5, 0, 10, 0);
  thread clampit(clamptwo,nullptr);
  wait(1250,msec);
  goalgrabber.set(true); //grab first mobile goal
  chassis.set_drive_constants(9, 1.5, 0, 10, 0);
  //chassis.drive_distance(3);
  chassis.turn_to_angle(180);
  intakeOn = true;
  chassis.drive_distance(24);
  //second on blue side
  wait(500,msec);
  chassis.turn_to_angle(42.5);
  chassis.drive_distance(31);
  chassis.turn_to_angle(180);
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.drive_distance(22);



  wait(200,msec);
  chassis.set_drive_constants(9, 1.5, 0, 10, 0);

  chassis.turn_to_angle(125);
  chassis.drive_distance(5);
  
  armgrab.set(true); //
  chassis.turn_to_angle(195); //swipe blue ring 
  armgrab.set(false);
  chassis.turn_to_angle(138);
  chassis.drive_distance(17);


  wait(1000,msec);
  chassis.drive_distance(-15);
  chassis.turn_to_angle(315);
  chassis.drive_distance(-15);
  goalgrabber.set(false); //set down second goal
  wait(800,msec);
  chassis.drive_distance(15);
  chassis.set_drive_constants(12, 1.5, 0, 10, 0);

  chassis.drive_distance(-15);
  chassis.drive_distance(15);


  // //working part end



  // wait(400,msec);
  // chassis.drive_distance(15);
  // wait(400,msec);
  // chassis.turn_to_angle(270);
  // wait(200,msec);
  // chassis.drive_distance(-42);
  // wait(400,msec);
  
  // chassis.turn_to_angle(235);
  // goalgrabber.set(false);
  // thread clampit(clamptwo,nullptr);
  // wait(1250,msec);
  // goalgrabber.set(true);
  // chassis.set_drive_constants(6, 1.5, 0, 10, 0);
  // wait(400,msec);
  // chassis.turn_to_angle(214);
  // wait(200,msec);
  // chassis.drive_distance(40.5);
  // intakeOn = true;
  // wait(400,msec);
  // chassis.drive_distance(-34);

  // chassis.turn_to_angle(89.5);
  // intakeOn = true;
  // chassis.drive_distance(16.5);
  // intakeOn = true;
  // wait(300,msec);

  // chassis.turn_to_angle(215);
  // chassis.drive_distance(28.75);
  // wait(300,msec);

  // chassis.turn_to_angle(89.5);
  // chassis.drive_distance(19);
  // intakeOn = true;
  // wait(300,msec);
  
  // chassis.turn_to_angle(127.5);
  // chassis.drive_distance(23);
  // intakeOn = true;
  // wait(700,msec);
  // chassis.drive_distance(-10);
  // chassis.turn_to_angle(345);
  // wait(300,msec);
  // chassis.drive_distance(-8);
  // wait(200,msec);
  // goalgrabber.set(false);
  // chassis.drive_distance(4);



}



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                        Auto_Interaction Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous interaction*/
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void auto_Interaction(void) {
  // Add functions for interaction phase
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool firstAutoFlag = true;

void autonomousMain(void) {
  // ..........................................................................
  // The first time we enter this function we will launch our Isolation routine
  // When the field goes disabled after the isolation period this task will die
  // When the field goes enabled for the second time this task will start again
  // and we will enter the interaction period. 
  // ..........................................................................
  printf("ok\n");
  if(firstAutoFlag)
    auto_Isolation();
  else 
    auto_Interaction();

  firstAutoFlag = false;
}




controller Controller;

void inertialreading() {
  while (true) { 
    Controller.Screen.clearLine(1);
    Controller.Screen.setCursor(1, 1);  
    Controller.Screen.print("Inertial: %.4f", imu.heading());
    wait(500, msec);
  }
}

int main() {

  imu.setHeading(0, degrees);

  // local storage for latest data from the Jetson Nano
  static AI_RECORD local_map;

  // Run at about 15Hz
  int32_t loop_time = 33;

  // start the status update display
  thread t1(dashboardTask);

  thread t2(inertialreading);

  thread t3(intakeTask);

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomousMain);

  // print through the controller to the terminal (vexos 1.0.12 is needed)
  // As USB is tied up with Jetson communications we cannot use
  // printf for debug.  If the controller is connected
  // then this can be used as a direct connection to USB on the controller
  // when using VEXcode.
  //
  //FILE *fp = fopen("/dev/serial2","wb");
  this_thread::sleep_for(loop_time);

  //Arm.setVelocity(60, percent);

  // chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  // chassis.set_heading_constants(6, .4, 0, 1, 0);
  // chassis.set_turn_constants(12, .4, .03, 3, 15);
  // chassis.set_swing_constants(12, .3, .001, 2, 15);

  // // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  // chassis.set_drive_exit_conditions(1.5, 300, 5000);
  // chassis.set_turn_exit_conditions(1, 300, 3000);
  // chassis.set_swing_exit_conditions(1, 300, 3000);
  // chassis.turn_to_angle(180);
  
  while(1){
    wait(100,msec);
  }
  
  // while(1) {
  //     chassis.control_arcade();
  //     if(Controller.ButtonR1.pressing()){
  //       intake();
  //     }  
  //     else if(!Controller.ButtonR2.pressing()) {
  //       primaryIntake.stop();
  //       secondaryIntake.stop();
  //       thirdIntake.stop();
  //     }
  //     if(Controller.ButtonR2.pressing()){
  //       intakestop();
  //     } 
  //     else if(!Controller.ButtonR1.pressing()) {
  //       primaryIntake.stop();
  //       secondaryIntake.stop();
  //       thirdIntake.stop();
  //     }
  //     if(Controller.ButtonL1.pressing()){
  //       goalgrabber.set(true);
  //     }
  //     if(Controller.ButtonL2.pressing()){
  //      armgrab.set(true);
  //     }
      
      jetson_comms.get_data( &local_map );

      // set our location to be sent to partner robot
      link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );

      // fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az)

      // request new data    
      // NOTE: This request should only happen in a single task.    
      jetson_comms.request_map();

      // Allow other tasks to run
      this_thread::sleep_for(loop_time);
  }
