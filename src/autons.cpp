#include "autons.hpp"
#include <cstdint>
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/slew.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(11, 0, 20);
  chassis.pid_drive_constants_set(20, 0,100 );
  chassis.pid_turn_constants_set(3, 0.05, 19.82, 15);
  chassis.pid_swing_constants_set(6, 0, 65);

  chassis.pid_turn_exit_condition_set(80_ms, 0.5_deg, 250_ms, 1.5_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  myChassis->moveDistance(40_in);
  
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// . . .
// Make your own autonomous functions here!
// . . .

void Blue_Negative_AWP(){chassis.pid_drive_set(-10,70); // goes backwards 8 inches to allign with alliance
                                               // wall stake and to move rings out of the way
  chassis.pid_wait(); // waits for the robot to travel 8 inches

  chassis.pid_turn_set(-89,TURN_SPEED); // turns to the left with back facing the
                                                      // alliance stake 
  chassis.pid_wait(); // waits till the robot is facing -89 degrees

  chassis.pid_drive_set(-7,DRIVE_SPEED); // Goes backwards 7 inches to be touch alliance stake
  chassis.pid_wait(); // waits for the robot to travel -7 inches

  chassis.pid_drive_set(2,50);
  chassis.pid_wait();

  IntakeToggle(); // Turns on the intake to make preload come out
  pros::delay(400); // waits 0.4 second to ensure preload gets on the alliance stake 
  IntakeReverseToggle(); // Reverse to get hook off of preload
  pros::delay(150); // waits 0.15 seconds to ensure hook is out
  IntakeToggle(); // turn off the intake

  chassis.pid_drive_set(6,DRIVE_SPEED); // Goes forward 8 inches to get off alliance stake
  chassis.pid_wait(); // waits for the robot to travel 8 inches

  chassis.pid_turn_set(-232,TURN_SPEED); // turns to the left till the back is facing the goal
  chassis.pid_wait(); // waits till the robot is facing -232 degrees

  chassis.pid_drive_set(-38,60,true); // Goes backwards 38 inches till it hits the goal
  chassis.pid_wait_until(-26); // when the robot has traveled -26 inches
  chassis.pid_speed_max_set(30); // slow down the robot speed to 30
  chassis.pid_wait(); // waits for the robot to travel -38 inches
  
  ClampToggle(); // Clamp piston extends 
  pros::delay(300); // Waits 300 ms to ensure we have goal secured

  chassis.pid_turn_set(-362,TURN_SPEED); // Turns to the left till the front faces the single stack
  chassis.pid_wait(); // Waits till the robot is facing -362 degrees

  IntakeToggle();

  chassis.pid_drive_set(30,80,true); // Goes foward 30 inches to intake botton ring on stack
  chassis.pid_wait_until(22); // when the robot has traveled 22 inches
  chassis.pid_speed_max_set(40); // slow down the robot speed to 40
  chassis.pid_wait(); // Waits for the robot to travel 30 inches

  pros::delay(500); // waits 500 ms to ensure intake has the ring before any movement

  chassis.pid_turn_set(-176,TURN_SPEED); // Turns to left till the front is facing the ladder
  chassis.pid_wait(); // Waits till the robot is facing -176 degrees

  chassis.pid_drive_set(40,60); // Goes foward 40 inches to touch the ladder
  chassis.pid_wait(); // Waits for the robot to travel 40 inches
  

}
void Blue_Positive_AWP(){
  Task_Toggle(); // Toggles the color sorter task and turns it off
  IntakeToggle(); // Turns on the intake
  IntakePistonToggle(); // Toggles Intake piston to etend

  chassis.pid_drive_set(8,60); // goes forward 8 inches towards the stacked rings
                                             // in front of alliance stake
  chassis.pid_wait(); // Waits for the robot to travel 8 inches 
  IntakePistonToggle(); // Toggles Intake piston to retract

  pros::delay(250); // waits 250 ms to ensure ring is in the intake
  chassis.pid_drive_set(-8,50); // goes backward 8 inches to go back where it started
  chassis.pid_wait(); // waits for the robot to travel -8 inches

  IntakeToggle(); // Toggles Intake to turns off

  chassis.pid_drive_set(16,50); // goes forward 16 inches to line up in front 
                                              // of the alliance stake
  chassis.pid_wait(); // waits for the robot to travel 16 inches

  chassis.pid_turn_set(-70,TURN_SPEED); // Turns to the left 70 degrees to push the ring in 
                                                      // front of the alliance stake away
  chassis.pid_wait_quick_chain(); // waits for the robot to turn to -68 degrees to leave early and go to
                                  // the next line of code

  chassis.pid_turn_set(-50,TURN_SPEED); // turns to the right 20 degrees for the back to face
                                                      // the alliance stake
  chassis.pid_wait(); // Waits for the robot to turn to -50 degrees.
  
  chassis.pid_drive_set(-16,40); // goes backward 16 inches till the back touches the alliance stake
  chassis.pid_wait_until(-11); // when the robot reaches -11 inches 
  IntakeToggle(); // Intake starts spinning
  chassis.pid_wait(); // waits for the robot to finish traveling -16 inches
  //Color_sorter.set_value(1);
  pros::delay(1000); // waits for 1 second to to ensure the ring gets on the stake.

  chassis.pid_drive_set(5,DRIVE_SPEED);
  chassis.pid_wait();
  //Color_sorter.set_value(0);

  chassis.pid_turn_set(-140,TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(38,DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(500);

  IntakeToggle();
  chassis.pid_turn_set(-200,TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-30,DRIVE_SPEED);
  chassis.pid_wait_until(-14);
  chassis.pid_speed_max_set(20);
  chassis.pid_wait();
  IntakeToggle();
  ClampToggle();

  pros::delay(150);

  chassis.pid_turn_set(-118,TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(17,60);
  chassis.pid_wait();
  
  pros::delay(500);

  chassis.pid_turn_set(49,TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(33,DRIVE_SPEED);
  chassis.pid_wait_until(20);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait();

  pros::delay(500);
  
}
void Red_Negative_AWP(){
  //Task_Toggle(); // Disables color sorter task
  chassis.pid_drive_set(-10,70); // goes backwards 8 inches to allign with alliance
                                               // wall stake and to move rings out of the way
  chassis.pid_wait(); // waits for the robot to travel 8 inches

  chassis.pid_turn_set(89,TURN_SPEED); // turns to the left with back facing the
                                                      // alliance stake 
  chassis.pid_wait(); // waits till the robot is facing 89 degrees

  chassis.pid_drive_set(-7,DRIVE_SPEED); // Goes backwards 7 inches to be touch alliance stake
  chassis.pid_wait(); // waits for the robot to travel -7 inches

  chassis.pid_drive_set(2,50);
  chassis.pid_wait();

  IntakeToggle(); // Turns on the intake to make preload come out
  pros::delay(400); // waits 0.4 second to ensure preload gets on the alliance stake 
  IntakeReverseToggle(); // Reverse to get hook off of preload
  pros::delay(150); // waits 0.15 seconds to ensure hook is out
  IntakeToggle(); // turn off the intake

  chassis.pid_drive_set(6,DRIVE_SPEED); // Goes forward 8 inches to get off alliance stake
  chassis.pid_wait(); // waits for the robot to travel 8 inches

  chassis.pid_turn_set(232,TURN_SPEED); // turns to the left till the back is facing the goal
  chassis.pid_wait(); // waits till the robot is facing 232 degrees

  chassis.pid_drive_set(-38,60,true); // Goes backwards 38 inches till it hits the goal
  chassis.pid_wait_until(-26); // when the robot has traveled -26 inches
  chassis.pid_speed_max_set(30); // slow down the robot speed to 30
  chassis.pid_wait(); // waits for the robot to travel -38 inches
  
  ClampToggle(); // Clamp piston extends 
  pros::delay(300); // Waits 300 ms to ensure we have goal secured

  chassis.pid_turn_set(362,TURN_SPEED); // Turns to the left till the front faces the single stack
  chassis.pid_wait(); // Waits till the robot is facing 362 degrees

  IntakeToggle();

  chassis.pid_drive_set(30,80,true); // Goes foward 30 inches to intake botton ring on stack
  chassis.pid_wait_until(22); // when the robot has traveled 22 inches
  chassis.pid_speed_max_set(40); // slow down the robot speed to 40
  chassis.pid_wait(); // Waits for the robot to travel 30 inches

  pros::delay(500); // waits 500 ms to ensure intake has the ring before any movement

  chassis.pid_turn_set(176,TURN_SPEED); // Turns to left till the front is facing the ladder
  chassis.pid_wait(); // Waits till the robot is facing 176 degrees

  IntakeReverseToggle();

  chassis.pid_drive_set(40,60); // Goes foward 40 inches to touch the ladder
  chassis.pid_wait(); // Waits for the robot to travel 40 inches

}
void Red_Positive_AWP(){
  Task_Toggle();
  IntakeToggle();
  IntakePistonToggle();

  chassis.pid_drive_set(8,60);
  chassis.pid_wait();
  IntakePistonToggle();

  pros::delay(250);
  chassis.pid_drive_set(-8,50);
  chassis.pid_wait();

  IntakeToggle();

  chassis.pid_drive_set(16,50);
  chassis.pid_wait();

  chassis.pid_turn_set(70,TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(50,TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-16,40);
  chassis.pid_wait_until(-11);
  IntakeToggle();
  chassis.pid_wait();
  //Color_sorter.set_value(1);
  pros::delay(1000);

  chassis.pid_drive_set(5,DRIVE_SPEED);
  chassis.pid_wait();
  //Color_sorter.set_value(0);

  chassis.pid_turn_set(140,TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(38,DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(500);

  IntakeToggle();
  chassis.pid_turn_set(200,TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-30,DRIVE_SPEED);
  chassis.pid_wait_until(-14);
  chassis.pid_speed_max_set(20);
  chassis.pid_wait();
  IntakeToggle();
  ClampToggle();

  pros::delay(150);

  chassis.pid_turn_set(118,TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(17,60);
  chassis.pid_wait();
  
  pros::delay(500);

  chassis.pid_turn_set(-49,TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(31,DRIVE_SPEED);
  chassis.pid_wait_until(20);
  chassis.pid_speed_max_set(40);
  chassis.pid_wait();

  
}

void Blue_Single_Goal(){
  //AutonAutoClamp.resume();
  //AutoClamp.suspend();

  chassis.pid_drive_set(-30,70,true);
  chassis.pid_wait();

  pros::delay(300);
  IntakeToggle();

  chassis.pid_turn_set(-105,TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(21,80);
  chassis.pid_wait();

  pros::delay(400);

  chassis.pid_drive_set(-5,80);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-87,TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(14,60);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-15,60);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-48,TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(18,80);
  chassis.pid_wait();

  pros::delay(800);

  chassis.pid_turn_set(90,TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(46,DRIVE_SPEED);
  chassis.pid_wait_until(20);
  IntakePistonToggle();
  chassis.pid_speed_max_set(30);
  chassis.pid_wait();

  IntakePistonToggle();

  chassis.pid_drive_set(-5,30);
  chassis.pid_wait_quick_chain();

  //AutonAutoClamp.suspend();
  //AutoClamp.resume();

}

void Red_Single_Goal(){
  //AutonAutoClamp.resume();
  //AutoClamp.suspend();

  chassis.pid_drive_set(-30,70,true);
  chassis.pid_wait();

  pros::delay(300);
  IntakeToggle();

  chassis.pid_turn_set(105,TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(21,80);
  chassis.pid_wait();

  pros::delay(400);

  chassis.pid_drive_set(-5,80);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(87,TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(14,60);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-15,60);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(48,TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(18,80);
  chassis.pid_wait();

  pros::delay(800);

  chassis.pid_turn_set(-90,TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(46,DRIVE_SPEED);
  chassis.pid_wait_until(20);
  IntakePistonToggle();
  chassis.pid_speed_max_set(30);
  chassis.pid_wait();

  IntakePistonToggle();

  chassis.pid_drive_set(-5,30);
  chassis.pid_wait_quick_chain();

  // chassis.pid_turn_set(-150,40);
  // chassis.pid_wait_quick_chain();

  //AutonAutoClamp.suspend();
 // AutoClamp.resume();

}

void SigAutoRN(){
 //NOJAM.resume(); // Resumes the anti jam task
 //AutoClamp.suspend(); // Suspends the autonomous clamp task
 //AutonAutoClamp.resume(); // Resumes the autonomous clamp task
 
 Arm.move(127); // Uses the wall stake mech to put ring on the alliance stake
 pros::delay(550); // Waits 0.55 second to ensure ring is on the stake
 Arm.move(0); // Stops the wall stake mech

 chassis.pid_drive_set(-4,50); // Goes backwards -4 inches to get a bette angle to
                                             // Intake ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to move -2 inches to exit quickly
                                 //then goes to the next line of code

 Arm.move_absolute(0,127); // Brings back the arm

 chassis.pid_turn_set(-50,TURN_SPEED); // Turns to the right 50 degrees to face the
                                                     // ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn 48 degree to exit quickly
                                 //then goes to the next line of code

 Test=3; // Spins the first stage motor to intake the ring

 IntakePistonToggle(); // Intake goes up to intake top ring
 pros::delay(250); // waits 0.25 seconds to ensure intake is up and spinning

 chassis.pid_drive_set(13,DRIVE_SPEED); // Goes forward 13 inches to intake top ring
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 11 inches to exit quickly
                                 //then goes to the next line of code

 IntakePistonToggle(); // Intake goes down to drag ring when it backs up
 pros::delay(250); // waits 0.25 seconds to ensure intake is down
 
 chassis.pid_drive_set(-5,40); // Goes backwards 5 inches to get off the ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -3 inches to exit quickly
                                 //then goes to the next line of code

 chassis.pid_turn_set(40,TURN_SPEED); // Turns to the left 88 degrees to make the back face the goal
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn -36 degrees to exit quickly
                                 //then goes to the next line of code

 chassis.pid_drive_set(-32,DRIVE_SPEED); // Goes backwards 32 inches to grab the goal
 chassis.pid_wait_until(-16); // When the robot has traveled -18 inches
 chassis.pid_speed_max_set(35); // Slow down the robot speed to 40
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -30 inches to exit quickly
                                 //then goes to the next line of code
 ClampToggle(); // Clamp piston extends to grab goal
 pros::delay(300); // Waits 0.3 seconds to ensure the goal is in the clamp

 Test=1; // Intake starts spinning to intake ring onto goal

 chassis.pid_turn_set(184,TURN_SPEED); // Turns to the left 237 degrees to face ladder
 chassis.pid_wait(); // Waits for the robot to turn to -184 degrees

 chassis.pid_drive_set(16,90); // Goes forward 15 inches to grab ring from the center
 chassis.pid_wait(); // Waits for the robot to travel 13 inches
 pros::delay(300); // Waits 0.3 seconds to ensure the ring gets on the goal

 chassis.pid_turn_set(162,TURN_SPEED); // Turns to the right 38 degrees to face the second 
                                                      // middle ring stack 
 chassis.pid_wait(); // Waits for the robot to turn to -156 degrees  

 chassis.pid_drive_set(12,80); // Goes forward 12 inches to intake the ring
 chassis.pid_wait(); // Waits for the robot to travel 12 inches 
 pros::delay(400);

 chassis.pid_drive_set(-12,40); // Goes backwards 12 inches to get a better angle at the 
                                              // single stack rings
 chassis.pid_wait(); // Waits for the robot to travel -12 inches
                                                  
 chassis.pid_turn_set(98,80); // Turns to the right 58 degrees to face the single ring stack
 chassis.pid_wait(); // Waits for the robot to turn to -98 degrees

 chassis.pid_drive_set(11,DRIVE_SPEED); //Goes forward 11 inches to intake bottom ring
 chassis.pid_wait(); // Waits for the robot to travel 11 inches
 pros::delay(300); // Waits 0.3 seconds to ensure the ring is in the intake

 chassis.pid_turn_set(-50,TURN_SPEED); // Turns to the right 148 degrees to face ladder
 chassis.pid_wait(); // Waits for the robot to turn to 50 degrees
//  NOJAM.suspend(); // Suspends the anti jam task

 chassis.pid_drive_set(14,DRIVE_SPEED); // Goes forward 18 inches to touch the ladder
 chassis.pid_wait(); // Waits for the robot to travel 18 inches

//  Arm.move(127); // Uses the wall stake mech to touch ladder
//  pros::delay(600); // Waits 0.6 second to ensure arm is up
//  Arm.move(0); // Stops the wall stake mech

}

void SigAutoBN(){
 //NOJAM.resume(); // Resumes the anti jam task
 //AutoClamp.suspend(); // Suspends the autonomous clamp task
 //AutonAutoClamp.resume(); // Resumes the autonomous clamp task
 
 Arm.move(127); // Uses the wall stake mech to put ring on the alliance stake
 pros::delay(550); // Waits 0.55 second to ensure ring is on the stake
 Arm.move(0); // Stops the wall stake mech

 chassis.pid_drive_set(-4,50); // Goes backwards -4 inches to get a bette angle to
                                             // Intake ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to move -2 inches to exit quickly
                                 //then goes to the next line of code

 Arm.move_absolute(0,127); // Brings back the arm

 chassis.pid_turn_set(50,TURN_SPEED); // Turns to the right 50 degrees to face the
                                                     // ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn 48 degree to exit quickly
                                 //then goes to the next line of code

 Test=3; // Spins the first stage motor to intake the ring

 IntakePistonToggle(); // Intake goes up to intake top ring
 pros::delay(250); // waits 0.25 seconds to ensure intake is up and spinning

 chassis.pid_drive_set(13,DRIVE_SPEED); // Goes forward 13 inches to intake top ring
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 11 inches to exit quickly
                                 //then goes to the next line of code

 IntakePistonToggle(); // Intake goes down to drag ring when it backs up
 pros::delay(250); // waits 0.25 seconds to ensure intake is down
 
 chassis.pid_drive_set(-5,40); // Goes backwards 5 inches to get off the ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -3 inches to exit quickly
                                 //then goes to the next line of code

 chassis.pid_turn_set(-38,TURN_SPEED); // Turns to the left 88 degrees to make the back face the goal
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn -36 degrees to exit quickly
                                 //then goes to the next line of code

 chassis.pid_drive_set(-32,DRIVE_SPEED); // Goes backwards 32 inches to grab the goal
 chassis.pid_wait_until(-18); // When the robot has traveled -18 inches
 chassis.pid_speed_max_set(40); // Slow down the robot speed to 40
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -30 inches to exit quickly
                                 //then goes to the next line of code
 ClampToggle(); // Clamp piston extends to grab goal

 Test=1; // Intake starts spinning to intake ring onto goal

 chassis.pid_turn_set(-184,TURN_SPEED); // Turns to the left 237 degrees to face ladder
 chassis.pid_wait(); // Waits for the robot to turn to -184 degrees

 chassis.pid_drive_set(15,90); // Goes forward 15 inches to grab ring from the center
 chassis.pid_wait(); // Waits for the robot to travel 13 inches
 pros::delay(300); // Waits 0.3 seconds to ensure the ring gets on the goal

 chassis.pid_turn_set(-156,TURN_SPEED); // Turns to the right 38 degrees to face the second 
                                                      // middle ring stack 
 chassis.pid_wait(); // Waits for the robot to turn to -156 degrees  

 chassis.pid_drive_set(12,80); // Goes forward 12 inches to intake the ring
 chassis.pid_wait(); // Waits for the robot to travel 12 inches 
 pros::delay(400);

 chassis.pid_drive_set(-12,40); // Goes backwards 12 inches to get a better angle at the 
                                              // single stack rings
 chassis.pid_wait(); // Waits for the robot to travel -12 inches
                                                  
 chassis.pid_turn_set(-98,80); // Turns to the right 58 degrees to face the single ring stack
 chassis.pid_wait(); // Waits for the robot to turn to -98 degrees

 chassis.pid_drive_set(11,DRIVE_SPEED); //Goes forward 11 inches to intake bottom ring
 chassis.pid_wait(); // Waits for the robot to travel 11 inches
 pros::delay(300); // Waits 0.3 seconds to ensure the ring is in the intake

 chassis.pid_turn_set(50,TURN_SPEED); // Turns to the right 148 degrees to face ladder
 chassis.pid_wait(); // Waits for the robot to turn to 50 degrees
//  NOJAM.suspend(); // Suspends the anti jam task

 chassis.pid_drive_set(18,DRIVE_SPEED); // Goes forward 18 inches to touch the ladder
 chassis.pid_wait(); // Waits for the robot to travel 18 inches

 Arm.move(127); // Uses the wall stake mech to touch ladder
 pros::delay(600); // Waits 0.6 second to ensure arm is up
 Arm.move(0); // Stops the wall stake mech

}
void SigAWPR(){
 NOJAM.resume();
//  Blue_Mode.remove();
//  Red_Mode.resume();

 chassis.pid_drive_set(8,50); // Goes forward 8 inches to get closer to alliance stake
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 6 inches to exit quickly
                                 //then goes to the next line of code

 Arm.move(-70); // Uses the fish mech to put ring on the alliance stake
 pros::delay(720); // Waits 0.6 second to ensure ring is on the stake
 Arm.move(0); // Stops the fish mech

 chassis.pid_drive_set(-10,50); // Goes backwards 10 inches to get a bette angle to
                                             // Intake ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to move -8 inches to exit quickly
                                 //then goes to the next line of code

 chassis.pid_turn_set(-45,TURN_SPEED); // Turns to the left 45 degrees to face the
                                                     // ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn -43 degree to exit quickly
                                 //then goes to the next line of code

 IntakePistonToggle(); // Intake goes up to intake top ring
 IntakeToggle(); // Intake starts spinning to intake ring
 pros::delay(200); // waits 0.2 seconds to ensure intake is up and spinning

 chassis.pid_drive_set(6,DRIVE_SPEED); // Goes forward 5 inches to intake top ring
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 4 inches to exit quickly
                                 //then goes to the next line of code

 IntakePistonToggle(); // Intake goes down to drag ring when it backs up
 pros::delay(400); // waits 0.6 seconds to ensure intake is down
 
 chassis.pid_drive_set(-6,50); // Goes backwards 6 inches to get off the ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -4 inches to exit quickly
                                 //then goes to the next line of code

 IntakeToggle(); // Intake stops spinning to keep ring in intake

 chassis.pid_turn_set(28,TURN_SPEED); // Turns to the right 73 degrees to make the back face the goal
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn 26 degrees to exit quickly
                                 //then goes to the next line of code

 chassis.pid_drive_set(-27,DRIVE_SPEED); // Goes backwards 27 inches to grab the goal
 chassis.pid_wait_until(-15); // When the robot has traveled -15 inches
 chassis.pid_speed_max_set(40); // Slow down the robot speed to 50
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -25 inches to exit quickly
                                 //then goes to the next line of code

 ClampToggle(); // Clamp piston extends to grab goal
 pros::delay(300); // waits 0.3 seconds to ensure goal is secured

 IntakeToggle(); // Intake starts spinning to intake ring onto goal

 chassis.pid_turn_set(112,TURN_SPEED); // Turns to the rirght 84 degrees to ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn to 110 degrees to exit quickly
                                 //then goes to the next line of code
  
 chassis.pid_drive_set(14,DRIVE_SPEED); // Goes forward 14 inches to intake the botton ring
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 12 inches to exit quickly
                                 //then goes to the next line of code
 pros::delay(300); // waits 0.3 seconds to ensure ring is in the intake

 chassis.pid_drive_set(-4,50); // Goes backwards 4 inches to get better angle on 
                                                      // the rings in the middle
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -2 inches to exit quickly
                                 //then goes to the next line of code

 chassis.pid_turn_set(154,TURN_SPEED); // Turns to the right 52 degrees to face postive side
                                                    // to drop off the goal
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn to 162 degrees to exit quickly
                                 //then goes to the next line of code

 chassis.pid_drive_set(-60,DRIVE_SPEED); // Goes backwards 60 inches till it goes 
                                                                    // to the drop-off zone
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -58 inches to exit quickly
                                 //then goes to the next line of code

 ClampToggle(); // Clamp piston retracts to drop off goal
 Arm_Set_Toggle(); // Brings the arm back to set to turn properly
 pros::delay(300);// waits 0.3 seconds to ensure goal is out of clamp and arm is set

 chassis.pid_turn_set(53,TURN_SPEED); // Turns to the left 107 degrees to face the goal
                                                    // On positive side
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn to 55 degrees to exit quickly
                                 //then goes to the next line of code

 chassis.pid_drive_set(-28,DRIVE_SPEED); // Goes backawrd 32 inches to grab the goal
 chassis.pid_wait_until(-16); // When the robot has traveled -16 inches
 chassis.pid_speed_max_set(40); // Slow down the robot speed to 40
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -30 inches to exit quickly
                                 //then goes to the next line of code

 ClampToggle(); // Clamp piston extends to grab goal
 Arm.move(127); // Arm goes up to prevent from intake getting jammed
 pros::delay(300); // waits 0.3 seconds to ensure goal is secured and arm is up
 Arm.move(0); // Stops the arm

 chassis.pid_turn_set(-38,TURN_SPEED); // Turns to the left 99 degrees to face ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn to -40 degrees to exit quickly
                                 //then goes to the next line of code

 chassis.pid_drive_set(17,DRIVE_SPEED); // Goes forward 16 inches to intake the botton ring
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 14 inches to exit quickly
                                 //then goes to the next line of code
 pros::delay(300);

 chassis.pid_turn_set(129,TURN_SPEED); // Turns to the right 172 degrees to face the ladder
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn to 128 degrees to exit quickly
                                 //then goes to the next line of code
  
 chassis.pid_drive_set(31,DRIVE_SPEED); // Goes forward 32 inches to get to the touch 
                                                      // the ladder using the arm
 chassis.pid_wait(); // Waits for the robot to travel 32 inches

 chassis.pid_turn_set(120,TURN_SPEED);
 chassis.pid_wait_quick_chain();

 NOJAM.suspend();

}

void SigAWPB(){
 NOJAM.resume();
//  Blue_Mode.resume();
//  Red_Mode.remove();

 chassis.pid_drive_set(8,50); // Goes forward 8 inches to get closer to alliance stake
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 6 inches to exit quickly
                                 //then goes to the next line of code

 Arm.move(60); // Uses the fish mech to put ring on the alliance stake
 pros::delay(900); // Waits 0.6 second to ensure ring is on the stake
 Arm.move(0); // Stops the fish mech

 chassis.pid_drive_set(-10,50); // Goes backwards 10 inches to get a bette angle to
                                             // Intake ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to move -8 inches to exit quickly
                                 //then goes to the next line of code

// chassis.pid_drive_set(-2,DRIVE_SPEED);
// chassis.pid_wait_quick_chain();

 chassis.pid_turn_set(45,TURN_SPEED); // Turns to the right 45 degrees to face the
                                                     // ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn 43 degree to exit quickly
                                 //then goes to the next line of code

 IntakePistonToggle(); // Intake goes up to intake top ring
 IntakeToggle(); // Intake starts spinning to intake ring
 pros::delay(200); // waits 0.2 seconds to ensure intake is up and spinning

 chassis.pid_drive_set(6,DRIVE_SPEED); // Goes forward 5 inches to intake top ring
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 4 inches to exit quickly
                                 //then goes to the next line of code

 IntakePistonToggle(); // Intake goes down to drag ring when it backs up
 pros::delay(600); // waits 0.6 seconds to ensure intake is down
 
 chassis.pid_drive_set(-6,40); // Goes backwards 6 inches to get off the ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -4 inches to exit quickly
                                 //then goes to the next line of code

 IntakeToggle(); // Intake stops spinning to keep ring in intake

 chassis.pid_turn_set(-28,TURN_SPEED); // Turns to the left 73 degrees to make the back face the goal
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn -26 degrees to exit quickly
                                 //then goes to the next line of code

 chassis.pid_drive_set(-27,DRIVE_SPEED); // Goes backwards 27 inches to grab the goal
 chassis.pid_wait_until(-15); // When the robot has traveled -15 inches
 chassis.pid_speed_max_set(40); // Slow down the robot speed to 50
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -25 inches to exit quickly
                                 //then goes to the next line of code

 ClampToggle(); // Clamp piston extends to grab goal
 pros::delay(300); // waits 0.3 seconds to ensure goal is secured

 IntakeToggle(); // Intake starts spinning to intake ring onto goal

 chassis.pid_turn_set(-112,TURN_SPEED); // Turns to the left 84 degrees to ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn to -110 degrees to exit quickly
                                 //then goes to the next line of code
  
 chassis.pid_drive_set(14,DRIVE_SPEED); // Goes forward 14 inches to intake the botton ring
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 12 inches to exit quickly
                                 //then goes to the next line of code
 pros::delay(300); // waits 0.3 seconds to ensure ring is in the intake

 chassis.pid_drive_set(-4,50); // Goes backwards 4 inches to get better angle on 
                                                      // the rings in the middle
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -2 inches to exit quickly
                                 //then goes to the next line of code

 chassis.pid_turn_set(-164,TURN_SPEED); // Turns to the left 52 degrees to face postive side
                                                    // to drop off the goal
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn to -162 degrees to exit quickly
                                 //then goes to the next line of code

 chassis.pid_drive_set(-60,DRIVE_SPEED); // Goes backwards 60 inches till it goes 
                                                                    // to the drop-off zone
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -58 inches to exit quickly
                                 //then goes to the next line of code

 ClampToggle(); // Clamp piston retracts to drop off goal
 Arm_Set_Toggle(); // Brings the arm back to set to turn properly
 pros::delay(300);// waits 0.3 seconds to ensure goal is out of clamp and arm is set

 chassis.pid_turn_set(-57,TURN_SPEED); // Turns to the right 107 degrees to face the goal
                                                    // On positive side
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn to -55 degrees to exit quickly
                                 //then goes to the next line of code

 chassis.pid_drive_set(-32,DRIVE_SPEED); // Goes backawrd 32 inches to grab the goal
 chassis.pid_wait_until(-16); // When the robot has traveled -16 inches
 chassis.pid_speed_max_set(40); // Slow down the robot speed to 40
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -30 inches to exit quickly
                                 //then goes to the next line of code

 ClampToggle(); // Clamp piston extends to grab goal
 Arm.move(127); // Arm goes up to prevent from intake getting jammed
 pros::delay(300); // waits 0.3 seconds to ensure goal is secured and arm is up
 Arm.move(0); // Stops the arm

 chassis.pid_turn_set(42,TURN_SPEED); // Turns to the right 99 degrees to face ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn to 40 degrees to exit quickly
                                 //then goes to the next line of code

 chassis.pid_drive_set(16,DRIVE_SPEED); // Goes forward 16 inches to intake the botton ring
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 14 inches to exit quickly
                                 //then goes to the next line of code

 chassis.pid_turn_set(-130,TURN_SPEED); // Turns to the left 172 degrees to face the ladder
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn to 128 degrees to exit quickly
                                 //then goes to the next line of code
  
 chassis.pid_drive_set(32,DRIVE_SPEED); // Goes forward 32 inches to get to the touch 
                                                      // the ladder using the arm
 chassis.pid_wait(); // Waits for the robot to travel 32 inches
  
}

void SigAutoRP(){
 //NOJAM.resume();
 //AutoClamp.suspend();
 //AutonAutoClamp.resume();

 Arm.move(127); // Uses the wall stake mech to put ring on the alliance stake
 pros::delay(550); // Waits 0.55 second to ensure ring is on the stake
 Arm.move(0); // Stops the wall stake mech

 chassis.pid_drive_set(-4,50); // Goes backwards 4 inches to get a better angle to
                                             // Intake ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to move -2 inches to exit quickly
                                 //then goes to the next line of code
 Arm.move_absolute(0,127); // Brings the arm back

 chassis.pid_turn_set(45,TURN_SPEED); // Turns to the left 45 degrees to face the
                                                     // ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn 43 degree to exit quickly
                                 //then goes to the next line of code
 Test=3; // Turns on first stage of intake to keep ring only in first stage

 IntakePistonToggle(); // Intake goes up to intake top ring
 pros::delay(250); // waits 0.25 seconds to ensure intake is up and spinning

 chassis.pid_drive_set(15,DRIVE_SPEED); // Goes forward 15 inches to intake top ring
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 13 inches to exit quickly
                                 //then goes to the next line of code

 IntakePistonToggle(); // Intake goes down to drag ring when it backs up
 pros::delay(250); // waits 0.25 seconds to ensure intake is down
 
 chassis.pid_drive_set(-5,40); // Goes backwards 5 inches to get off the ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -3 inches to exit quickly
                                 //then goes to the next line of code

 chassis.pid_turn_set(-35,TURN_SPEED); // Turns to the left 80 degrees to make the back face the goal
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn 33 degrees to exit quickly
                                 //then goes to the next line of code

 chassis.pid_drive_set(-27,DRIVE_SPEED); // Goes backwards 27 inches to grab the goal
 chassis.pid_wait_until(-15); // When the robot has traveled -15 inches
 chassis.pid_speed_max_set(40); // Slow down the robot speed to 40
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -25 inches to exit quickly
                                 //then goes to the next line of code
 ClampToggle(); // Clamp piston extends to grab goal
 pros::delay(300); // waits 0.3 seconds to ensure goal is secured

 Test=1; // Turns on both intake motors to intake ring onto goal

 chassis.pid_turn_set(-145,TURN_SPEED); // Turns to the right 110 degrees to ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn to 143 degrees to exit quickly
                                 //then goes to the next line of code
  
 chassis.pid_drive_set(17,DRIVE_SPEED); // Goes forward 17 inches to intake the botton ring
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 15 inches to exit quickly
                                 //then goes to the next line of code
 pros::delay(350); // waits 0.35 seconds to ensure ring is in the intake

 chassis.pid_turn_set(40,80); // Turns to the left 200 degrees to face the ladder
 chassis.pid_wait(); // waits for the robot to turn to -40

 chassis.pid_drive_set(22,DRIVE_SPEED); // Goes forward 22 inches to get to the touch 
                                                      // the ladder using the arm
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 20 inches

 Arm.move(127); // Uses the wall stake mech to touch ladder
 pros::delay(700); // Waits 0.7 second to ensure wall stake mech touches ladder
 Arm.move(0); // Stops the wall stake mech

//  chassis.pid_turn_set(90,80);
//  chassis.pid_wait();
//  Test=2; //

//  chassis.pid_drive_set(20,60);
//  chassis.pid_wait();

//  chassis.pid_drive_set(-6,60);
//  chassis.pid_wait();

// //  Arm.move_absolute(1800, 127);
 
// //  chassis.pid_drive_set(28,40); // Goes forward 28 inches to get to the touch 
// //                                                       // the ladder using the arm
// //  chassis.pid_wait(); // Waits for the robot to travel 32 inches

//  NOJAM.suspend();
 //AutonAutoClamp.suspend();
 //AutoClamp.resume();
}

void SigAutoBP(){
 //NOJAM.resume();
 //AutoClamp.suspend();
 AutonAutoClamp.resume();

 Arm.move(127); // Uses the wall stake mech to put ring on the alliance stake
 pros::delay(550); // Waits 0.55 second to ensure ring is on the stake
 Arm.move(0); // Stops the wall stake mech

 chassis.pid_drive_set(-4,50); // Goes backwards 4 inches to get a better angle to
                                             // Intake ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to move -2 inches to exit quickly
                                 //then goes to the next line of code
 Arm.move_absolute(0,127); // Brings the arm back

 chassis.pid_turn_set(-45,TURN_SPEED); // Turns to the left 45 degrees to face the
                                                     // ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn 43 degree to exit quickly
                                 //then goes to the next line of code
 Test=3; // Turns on first stage of intake to keep ring only in first stage

 IntakePistonToggle(); // Intake goes up to intake top ring
 pros::delay(250); // waits 0.25 seconds to ensure intake is up and spinning

 chassis.pid_drive_set(15,DRIVE_SPEED); // Goes forward 15 inches to intake top ring
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 13 inches to exit quickly
                                 //then goes to the next line of code

 IntakePistonToggle(); // Intake goes down to drag ring when it backs up
 pros::delay(250); // waits 0.25 seconds to ensure intake is down
 
 chassis.pid_drive_set(-5,40); // Goes backwards 5 inches to get off the ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -3 inches to exit quickly
                                 //then goes to the next line of code

 chassis.pid_turn_set(35,TURN_SPEED); // Turns to the left 80 degrees to make the back face the goal
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn 33 degrees to exit quickly
                                 //then goes to the next line of code

 chassis.pid_drive_set(-27,DRIVE_SPEED); // Goes backwards 27 inches to grab the goal
 chassis.pid_wait_until(-15); // When the robot has traveled -15 inches
 chassis.pid_speed_max_set(40); // Slow down the robot speed to 40
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel -25 inches to exit quickly
                                 //then goes to the next line of code
 ClampToggle(); // Clamp piston extends to grab goal
 pros::delay(300); // waits 0.3 seconds to ensure goal is secured

 Test=1; // Turns on both intake motors to intake ring onto goal

 chassis.pid_turn_set(145,TURN_SPEED); // Turns to the right 110 degrees to ring stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn to 143 degrees to exit quickly
                                 //then goes to the next line of code
  
 chassis.pid_drive_set(17,DRIVE_SPEED); // Goes forward 17 inches to intake the botton ring
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 15 inches to exit quickly
                                 //then goes to the next line of code
 pros::delay(350); // waits 0.35 seconds to ensure ring is in the intake

 chassis.pid_turn_set(-40,80); // Turns to the left 200 degrees to face the ladder
 chassis.pid_wait(); // waits for the robot to turn to -40

 chassis.pid_drive_set(22,DRIVE_SPEED); // Goes forward 22 inches to get to the touch 
                                                      // the ladder using the arm
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel 20 inches

 Arm.move(127); // Uses the wall stake mech to touch ladder
 pros::delay(700); // Waits 0.7 second to ensure wall stake mech touches ladder
 Arm.move(0); // Stops the wall stake mech

//  chassis.pid_turn_set(90,80);
//  chassis.pid_wait();
//  Test=2; //

//  chassis.pid_drive_set(20,60);
//  chassis.pid_wait();

//  chassis.pid_drive_set(-6,60);
//  chassis.pid_wait();

// //  Arm.move_absolute(1800, 127);
 
// //  chassis.pid_drive_set(28,40); // Goes forward 28 inches to get to the touch 
// //                                                       // the ladder using the arm
// //  chassis.pid_wait(); // Waits for the robot to travel 32 inches

//  NOJAM.suspend();
 //AutonAutoClamp.suspend();
 //AutoClamp.resume();
}

void GoalRush(){

 NOJAM.resume();

 chassis.pid_drive_set(-45,DRIVE_SPEED); // Goes forward ? inches to grab the goal in the middle
 chassis.pid_wait_until(-28); // When the robot has traveled -20 inches
 chassis.pid_speed_max_set(40); // Slow down the robot speed to 40
 chassis.pid_wait(); // Waits for the robot to travel ? inches to exit quickly
                                 //then goes to the next line of code

 ClampToggle(); // Clamp piston extends to grab goal
 IntakeToggle(); // Intake starts spinning to intake ring onto goal
 pros::delay(400); // waits 0.3 seconds to ensure goal is secured


 chassis.pid_drive_set(30,DRIVE_SPEED); // Goes backwards ? inches to get off the goal stack
 chassis.pid_wait_quick_chain(); // Waits for the robot to travel ? inches to exit quickly
                                 //then goes to the next line of code

 chassis.pid_turn_set(100,TURN_SPEED); // Turns to the right 90 degrees to face the goal
 chassis.pid_wait_quick_chain(); // Waits for the robot to turn to 90 degrees

 ClampToggle(); // Clamp piston retracts to drop off goal
 pros::delay(300); // waits 0.3 seconds to ensure goal is out of clamp

 chassis.pid_turn_set(-30,TURN_SPEED);
 chassis.pid_wait_quick_chain();

 chassis.pid_drive_set(-23,DRIVE_SPEED);
 chassis.pid_wait_until(-10);
 chassis.pid_speed_max_set(40);
 chassis.pid_wait_quick_chain();

  ClampToggle();
  pros::delay(300);
 
 chassis.pid_turn_set(20,TURN_SPEED);
 chassis.pid_wait_quick_chain();
 IntakePistonToggle();

 chassis.pid_drive_set(25,DRIVE_SPEED);
 chassis.pid_wait_quick_chain();

 IntakePistonToggle();
 pros::delay(400);

 chassis.pid_drive_set(-5,40);
 chassis.pid_wait_quick_chain();
 pros::delay(400);



    
}

void Skills(){
  //AutoClamp.suspend();
  //AutonAutoClamp.suspend();
  //Red_Mode.suspend();

  chassis.pid_drive_set(-1,30); // goes backwards 1 inches to be closer to alliance stake
  chassis.pid_wait(); // waits for the robot to travel -1 inches

  //Color_sorter.set_value(1);
  IntakeToggle(); // Turns on the intake to put preload on alliance stake
  pros::delay(500); // waits 0.5 seconds to ensure ring is on the alliance stake
  IntakeReverseToggle(); // Reverse to get hook off of preload
  pros::delay(200); // waits 0.2 seconds to ensure hook is out
  IntakeToggle(); // turn off the intake

  chassis.pid_drive_set(24,DRIVE_SPEED,false); // Goes forward 24 inches
  chassis.pid_wait(); // waits for the robot to travel 24 inches

  chassis.pid_turn_set(-66,TURN_SPEED); // turns to the left till the robot faces -66 degrees 
                                                      // or till the back faces the right corner
  chassis.pid_wait(); // waits for the robot to turn to -66 degree

  chassis.pid_drive_set(-72,50,true); // goes backwards 72 inches to ensure the robot pushes 
                                                            // the goal till it reaches to the corner
  chassis.pid_wait(); // waits for the robot to travel -80 inches

  chassis.pid_drive_set(55,50,true); // goes forward 55 inches to bring it back to the center
  chassis.pid_wait(); // waits for the robot to travel 50 inches

  chassis.pid_turn_set(80,TURN_SPEED); //turns to the right till the robot faces 80 degrees
                                                     // or till the back faces the left corner
  chassis.pid_wait(); // waits for the robot to turn to 80 degree

  chassis.pid_drive_set(-80,50,true); // goes backwards 80 inches to ensure the robot pushes
                                                            // the goal till it reaches to the corner
  chassis.pid_wait(); // waits for the robot to travel -80 inches

  chassis.pid_turn_set(45,TURN_SPEED); // turns to the left till the robot faces 45 degrees
                                                     // or till the front is facing under the ladder
  chassis.pid_wait(); // waits for the robot to turn to 45 degree

  chassis.pid_drive_set(142,DRIVE_SPEED,true); // goes forward 142 inches to get across the field
  chassis.pid_wait(); // waits for the robot to travel 128 inches

  chassis.pid_turn_set(93,TURN_SPEED); // turns to the right till the robot faces 93 degrees
                                                      // or till the back faces the left corner of the other side
  chassis.pid_wait(); // waits for the robot to turn to 93 degree

  chassis.pid_drive_set(-88,DRIVE_SPEED,true ); // goes backwards 808 inches to ensure the robot
                                                                       // pushes the goal till it almost reaches the corner
  chassis.pid_wait(); // waits for the robot to travel -88 inches

  chassis.pid_turn_set(130,25); // turns to the right till the robot faces 130 degrees
                                                      // or till the back is facing towards the corner
  chassis.pid_wait(); // waits for the robot to turn to 130 degree

  chassis.pid_drive_set(-18,50); // goes backwards 18 inches to push the goal into the corner
  chassis.pid_wait(); // waits for the robot to travel -18 inches

  chassis.pid_drive_set(20,50);
  chassis.pid_wait();

  chassis.pid_turn_set(75,50); 
  chassis.pid_wait();

  chassis.pid_drive_set(98,DRIVE_SPEED); // goes backwards 19 inches to stop touching the goal to make them count
  chassis.pid_wait(); // waits for the robot to travel -19 inches.

  chassis.pid_drive_set(-20,80);
  chassis.pid_wait();

}

void SkillsV2(){
// AutoClamp.suspend();
 //AutonAutoClamp.suspend();
  Red_Mode.suspend();
 NOJAM.resume();

 //Color_sorter.set_value(1);
 Test=1;
 pros::delay(300); // waits 0.5 seconds to ensure ring is on the alliance stake

 chassis.pid_drive_set(5,DRIVE_SPEED,false); // Goes forward 5 inches to turn properly
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 3 inches and exit early for next line of code

 chassis.pid_turn_set(-100,TURN_SPEED); // turns to the right 135 degrees to face the goal
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 135 degrees

 chassis.pid_drive_set(-20,DRIVE_SPEED); // Goes backward 26 inches to clamp the goal
 chassis.pid_wait_until(-7); // When the robot has traveled -10 inches
 chassis.pid_speed_max_set(40); // Slow down the robot speed to 40
 chassis.pid_wait_quick_chain(); // waits for the robot to travel -26 inches

 ClampToggle(); // Clamp piston extends to grab goal
 pros::delay(300); // waits 0.3 seconds to ensure goal is secured

 chassis.pid_turn_set(4,TURN_SPEED); // turns to the left 135 degrees to face the ring
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 0 degrees

 chassis.pid_drive_set(22,80); // Goes forward 26 inches to intake the ring
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 24 inches and exit early for next line of code

 chassis.pid_turn_set(65,TURN_SPEED); // turns to the right 80 degrees to face the ring on line
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 78 degrees and exit early for next line of code

 chassis.pid_drive_set(32,80); // Goes forward 50 and intake the ring
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 48 inches and exit early for next line of code
 pros::delay(300); // waits 0.3 seconds to ensure ring is in the intake

 chassis.pid_turn_set(204,TURN_SPEED); // turns to the right 120 degrees to face the thrid ring
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 198 degrees and exit early for next line of code

 chassis.pid_drive_set(20,80); // Goes forward 36 inches to intake the ring
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 34 inches and exit early for next line of code

 chassis.pid_turn_set(178,TURN_SPEED); // turns to the left 20 degrees to face the forth ring
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 180 degrees and exit early for next line of code

 chassis.pid_drive_set(27,30); // Goes forward 36 inches to intake the rings
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 34 inches and exit early for next line of code
 pros::delay(400); // waits 0.4 seconds to ensure goal is in the corner

 chassis.pid_turn_set(60,TURN_SPEED); // turns to the left 135 degrees to face the sixth ring
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 43 degrees and exit early for next line of code

 chassis.pid_drive_set(14,80); // Goes forward 17 inches to intake the rings
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 16 inches and exit early for next line of code
 pros::delay(400); // waits 0.4 seconds to ensure goal is in the corner

 chassis.pid_turn_set(-18,TURN_SPEED); // turns to the left 55 degrees to face the goal
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to -8 degrees and exit early for next line of code

 chassis.pid_drive_set(-12,40); // Goes forward 18 inches to push the goal to the corner
 chassis.pid_wait(); // waits for the robot to travel 16 inches and exit early for next line of code

 ClampToggle(); // Clamp piston retracts to drop off goal
 pros::delay(400); // waits 0.4 seconds to ensure goal is out of clamp

 chassis.pid_drive_set(6,DRIVE_SPEED); // Goes forward 5 inches to get out of the corner
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 3 inches and exit early for next line of code

 chassis.pid_turn_set(88,TURN_SPEED); // turns to the right 90 degrees to make the back face the ladder
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 89 degrees and exit early for next line of code

 chassis.pid_drive_set(-80,DRIVE_SPEED); // Goes backwards 86 inches to grab the goal
 chassis.pid_wait_until(-65); // When the robot has traveled -40 inches
 chassis.pid_speed_max_set(40); // Slow down the robot speed to 40
 chassis.pid_wait_quick_chain(); // waits for the robot to travel -86 inches

 ClampToggle(); // Clamp piston extends to grab goal
 pros::delay(250); // waits 0.3 seconds to ensure goal is secured

 chassis.pid_turn_set(5,TURN_SPEED); // turns to the left 90 degrees to face the ring stack
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 0 degrees and exit early for next line of code

 chassis.pid_drive_set(22,80); // Goes forward 26 inches to intake the ring
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 24 inches and exit early for next line of code

 chassis.pid_turn_set(-58 ,TURN_SPEED); // turns to the right 80 degrees to face the ring on line
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 78 degrees and exit early for next line of code

 chassis.pid_drive_set(35,80); // Goes forward 50 and intake the ring
 chassis.pid_wait_until(15);
 chassis.pid_speed_max_set(40);
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 48 inches and exit early for next line of code
 pros::delay(300); // waits 0.3 seconds to ensure ring is in the intake

 chassis.pid_turn_set(-192,TURN_SPEED); // turns to the right 120 degrees to face the thrid ring
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 198 degrees and exit early for next line of code

 chassis.pid_drive_set(28,80); // Goes forward 36 inches to intake the ring
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 34 inches and exit early for next line of code

 chassis.pid_turn_set(-174,TURN_SPEED); // turns to the left 20 degrees to face the forth ring
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 180 degrees and exit early for next line of code

 chassis.pid_drive_set(24,30); // Goes forward 36 inches to intake the rings
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 34 inches and exit early for next line of code
 pros::delay(300); // waits 0.3 seconds to ensure ring is in the intake

 chassis.pid_turn_set(-48,TURN_SPEED); // turns to the left 135 degrees to face the fifth ring
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 43 degrees and exit early for next line of code

 chassis.pid_drive_set(10,80); // Goes forward 10 inches to intake the rings
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 10 inches and exit early for next line of code
 pros::delay(300); // waits 0.3 seconds to ensure ring is in the goal

 chassis.pid_turn_set(30,TURN_SPEED); // turns to the left till the robot faces 30 degrees
                                      // or till the back is facing the corner
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 28 degree

 chassis.pid_drive_set(-10,DRIVE_SPEED); // goes backwards 6 inches to put the goal into the corner
 chassis.pid_wait_quick_chain(); // waits for the robot to travel -4 inches
 pros::delay(250); // waits 0.3 seconds to ensure goal is in the corner
 
 ClampToggle(); // Clamp piston retracts to drop off goal
 pros::delay(250); // waits 0.3 seconds to ensure goal is out of clamp

 chassis.pid_turn_set(48,TURN_SPEED); // turns to the right 10 degrees to face under the ladder
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 48 degrees

 chassis.pid_drive_set(88,DRIVE_SPEED,true); // goes forward 142 inches to get across the field
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 128 inches
 Intakefirst.move(0); // Turns off the intake to capture ring
 IntakeSecond.move(0);
 
 chassis.pid_turn_set(3,TURN_SPEED); // turns to the right 3 degrees to face the ladder
 chassis.pid_wait_quick_chain();

 chassis.pid_drive_set(18,DRIVE_SPEED);
 chassis.pid_wait_quick_chain();

 chassis.pid_turn_set(130,TURN_SPEED); 
 chassis.pid_wait_quick_chain();

 chassis.pid_drive_set(-13,40);
 chassis.pid_wait_quick_chain();

 ClampToggle(); // Clamps onto the middle goal in the other side
 Test =1;
 pros::delay(250); // waits 0.3 seconds to ensure goal and ring are secured 

 chassis.drive_angle_set(129); // Sets the drive angle to 129 degrees
 pros::delay(300);  // waits 0.3 seconds to ensure the robot iis calibrated

 chassis.pid_drive_set(29,55); // Goes forward 29 inches to intake the second ring
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 27 inches and exit early for next line of code

 chassis.pid_turn_set(84,TURN_SPEED); // turns to the left 44 degrees to face the third ring 
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 82 degrees and exit early for next line of code

 chassis.pid_drive_set(22,55); // Goes forward 21 inches to intake the third ring
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 19 inches and exit early for next line of code

 chassis.pid_turn_set(120,80); // turns to the right 36 degrees to back up to get closer to fourth ring
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 118 degrees and exit early for next line of code

 chassis.pid_drive_set(-48,DRIVE_SPEED); // Goes backwards 48 inches to get closer to fourth ring
 chassis.pid_wait_quick_chain(); // waits for the robot to travel -46 inches and exit early for next line of code

 chassis.pid_turn_set(218,TURN_SPEED); // turns to the right 98 degrees to face the fourth ring
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 216 degrees and exit early for next line of code

 chassis.pid_drive_set(36,60); // Goes forward 36 inches to intake the fourth ring
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 34 inches and exit early for next line of code

 chassis.pid_turn_set(264,TURN_SPEED); // turns to the right 46 degrees to face the fifth ring
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 262 degrees and exit early for next line of code

 chassis.pid_drive_set(34,60); // Goes forward 35 inches to intake the fifth ring
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 33 inches and exit early for next line of code
 pros::delay(300); // waits 0.3 seconds to ensure ring is in the goal

 chassis.pid_turn_set(347,-TURN_SPEED); // turns to the left 83 degrees to face the sixth ring
                                                      // Below the stack
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 345 degrees and exit early for next line of code

 chassis.pid_drive_set(22,60); // Goes forward 22 inches to intake the sixth ring
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 20 inches and exit early for next line of code

 chassis.pid_drive_set(-10,50); // Goes backwards 10 inches to not intake blue ring and 
                                              // to get better angle on putting the goal in the corner
 chassis.pid_wait_quick_chain(); // waits for the robot to travel -8 inches and exit early for next line of code
 pros::delay(300); // waits 0.3 seconds to ensure ring is in the intake

 chassis.pid_turn_set(160,-TURN_SPEED); // turns to the left 185 degrees to face the corner
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 158 degrees and exit early for next line of code
 pros::delay(400); // waits 0.4 seconds to ensure ring is in the goal

 ClampToggle(); // unclamps the goal to drop it off
 Intakefirst.move(0); // Stops the intake 
 IntakeSecond.move(0);
 pros::delay(150); // waits 0.250 seconds to ensure goal is out of clamp
 chassis.pid_drive_set(2,DRIVE_SPEED);
 chassis.pid_wait_quick_chain();
 ClampToggle();
 pros::delay(150);
 chassis.pid_drive_set(-35,70); // Goes backwards 44 inches to get it in the corner
 chassis.pid_wait_quick_chain(); // waits for the robot to travel -42 inches and exit early for next line of code
 
 chassis.pid_drive_set(6,DRIVE_SPEED); // Goes forward 6 inches to get out of the corner
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 4 inches and exit early for next line of code

 chassis.pid_turn_set(80,TURN_SPEED);  // turns to the left 80 degrees to face the alliance stake
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 78 degrees and exit early for next line of code

 Test = 1;
 chassis.pid_drive_set(24,DRIVE_SPEED); // Goes forward 24 inches to intake the ring
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 22 inches and exit early for next line of code
 pros::delay(200); // waits 0.2 seconds to ensure ring is in the intake
 Test =0;

 chassis.pid_drive_set(30,90); // Goes forward 32 inches to get in front of alliance stake
 chassis.pid_wait(); // waits for the robot to travel 32 inches

 chassis.pid_turn_set(171,TURN_SPEED); // turns to the right 91 degrees to face the alliance stake
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 169 degrees and exit early for next line of code

 chassis.pid_drive_set(-23.5,50); // Goes backwards 24 inches to allgn the goal with the alliance stake
 chassis.pid_wait(); // waits for the robot to travel -24 inches

 chassis.pid_drive_set(1.5,40); // Goes forward 3 inches to put ring on alliance stake
 chassis.pid_wait();  // waits for the robot to travel 3 inches
 Intakefirst.move(127); // Turns on the intake to put ring on alliance stake
 IntakeSecond.move(127);
 pros::delay(1000); // waits 0.3 seconds to ensure ring is in the intake

 chassis.pid_turn_set(250,TURN_SPEED); // turns to the right 79 degrees to face fourth goal
 chassis.pid_wait_quick_chain(); // waits for the robot to turn to 248 degrees and exit early for next line of code

 chassis.pid_drive_set(-60,70); // Goes backwards 70 inches to push the goal into corner
 chassis.pid_wait_quick_chain(); // waits for the robot to travel -68 inches and exit early for next line of code

 chassis.pid_drive_set(15,DRIVE_SPEED); // Goes forward 15 inches to get off the goal stack
 chassis.pid_wait_quick_chain(); // waits for the robot to travel 13 inches and exit early for next line of code



//  Arm.move(127); // Arm goes up to hang
//  chassis.pid_drive_set(-40,80); // Goes forward 10 inches to hang
//  chassis.pid_wait_quick_chain();
//  Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // Arm holds the position
//  Arm.move(0); // Arm goes up to hang

 
}