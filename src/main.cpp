#include "main.h"
#include "EZ-Template/auton.hpp"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-7,-5,-9},  // Left Chassis Ports (negative port will reverse it!)
    {13,17,19},     // Right Chassis Ports (negative port will reverse it!)

    16,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();
  //gyro.reset();
  //Color_sorter.set_value(0);
  OP.set_led_pwm(100);
  //22Blue_Mode.suspend();
  // Red_Mode.suspend();


  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      //Auton("Swing Example\n\nSwing in an 'S' curve", swing_example),
      // Auton("Blue Negative AWP\n\nDoes a AWP in blue negative side.", Blue_Negative_AWP),
      // Auton("Blue Positive AWP\n\nDoes a AWP in blue positive side.", Blue_Positive_AWP),
      //Auton("Red Negative AWP\n\nDoes a AWP in red negative side.", Red_Negative_AWP),
      //Auton("Red Positive AWP\n\nDoes a AWP in red positive side.", Red_Positive_AWP),
      // Auton("Blue Single Goal\n\n Gets one goal and fills it up.", Blue_Single_Goal),
      //Auton("Red Single Goal\n\n Gets one goal and fills it up.", Red_Single_Goal),
      // Auton("Blue Rush \n\ngoes for middle goal.", Blue_Rush),
      //Auton("Red Rush \n\ngoes for middle goal.", Red_Rush),
      Auton("Sig Negative Red\n\n scores on alliance stake and scores 4 more on the neg goal.",SigAutoRN),
      Auton("Sig Negative Blue\n\n scores on alliance stake and scores 4 more on the neg goal.",SigAutoBN),
      Auton("Sig Positive Red\n\n scores on alliance stake and scores 2 more on the pos goal.",SigAutoRP),
      Auton("Sig Positive Blue\n\n scores on alliance stake and scores 2 more on the pos goal.",SigAutoBP),
      Auton("Sig AWP \n\n Scores on alliance stake, two more on neg goal, 1 more on pos goal", SigAWPR),
      Auton("Elim Auto \n\n Gets thrid goal and puts on ring on it.", GoalRush),
      Auton("Skills", SkillsV2),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble("..");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/** 
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() { // Driver Control
  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;
  //AutonAutoClamp.suspend();
  NOJAM.suspend();
  // Blue_Mode.suspend();
  // Red_Mode.suspend();
  

  
  while (true) {

    chassis.opcontrol_tank();  // Tank control

    // . . 
    // Put more user control code here!
    // . . .
      // Trigger the selected autonomous routine
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) { // When button Up is pressed
        autonomous(); // Activate selected autonomous
        chassis.drive_brake_set(driver_preference_brake); // Set brakes to drivers preference
      }
      if(master.get_digital(DIGITAL_R2)){ // When holding button R2 
       Intakefirst.move(127);
       //Test = 1;
      }
      else if(master.get_digital(DIGITAL_R1)){ // When holding button R1
        Intakefirst.move(-127); // Spin Inakte first motor to 127 voltages (forwards)
      }
      else if(master.get_digital(DIGITAL_DOWN)){ // When holding button down
        Arm_Out();
       if(Arm.get_position() < -300){
        Intake_P.set_value(1); // Brings the intake up
       }

      }
      else if(master.get_digital(DIGITAL_L2)){
        Intakefirst.move(127);
        if( RingStop.get_distance() < 225){
         // pros::delay(5);
          Intakefirst.move(0);
        }
       }
      else{
        Intakefirst.move(0);
        Test = 0;
        Intake_P.set_value(0);
      } 
      
      if(master.get_digital_new_press(DIGITAL_L1)){
        Arm_Toggle();
      }
      if(master.get_digital_new_press(DIGITAL_LEFT)){ // When button left is pressed
       Task_Toggle(); // Toggle to turn on or off on task
      }
      if(master.get_digital_new_press(DIGITAL_RIGHT)){ // When button right is pressed
        LeftDoinker_Toggle(); // Toggle to extend or retract doinker
      }
      if(master.get_digital_new_press(DIGITAL_Y)){ // When button Y is pressed
        RightDoinker_Toggle();
      }
      if(master.get_digital_new_press(DIGITAL_B)){ // When button B is pressed
        ClampToggle(); // Toggle to extend or retract clamp
      }
      // if(master.get_digital_new_press(DIGITAL_A)){
      //   Blue_Mode.remove();
      // }
      // if (master.get_digital_new_press(DIGITAL_X)){
      //   Red_Mode.remove();
      // }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}


//
//you didn't see that