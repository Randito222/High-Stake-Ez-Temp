#include <cmath>
#include <iostream>
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "main.h"
#include "okapi/api/util/mathUtil.hpp"
#include "pros/colors.h"
#include "pros/colors.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "subsystems.hpp"

//                                //
//                                //
//          PID Settings          //
//                                //
//                                //

const double pi = 3.14159265359; // The value for PI
const double Wheel_circumference = 3 * okapi::pi; // Wheel size * PI
const double Track_Width = 10.25; // Width from left wheel to right wheel
const double Wheel_base = 10; // From far edge of front wheel to far edge of back wheel
const double CIRCUMDIAMETER = sqrt(pow(Track_Width, 2) + pow(Wheel_base, 2)); 
// Calculate diagonal distance between nonadjacent wheels using pythagorean theorum.
double inchesToTicks(double inches) 
{
  // Derived from: length/circumference = deg/360
  double ticks = (inches / Wheel_circumference) * 360;
  return ticks;
}
double ticksToInches(double ticks) {
  double inches = (ticks * Wheel_circumference) / 360;
  return inches;
}

double timer(){
 double start_time = pros::millis(); //gets the value of how long it's been when the program started
 // do stuff
 double new_time = pros::millis() - start_time;  // new time with old time
 return new_time; // returns the difference
}

double getBaseAvg(){
  double total = ((Left_Front.get_position() + Left_Middle.get_position() + Left_Back.get_position() + 
    Right_Front.get_position() + Right_Middle.get_position() + Right_Back.get_position()) /6.0);
    // Adds all Wheel motor's encoder values and divides with the amount of wheels
    return total; // Returns the average encoder value
}

void ClearEncorders(){
    Left_Front.tare_position();
    Left_Middle.tare_position();
    Left_Back.tare_position();
    Right_Front.tare_position();
    Right_Middle.tare_position();
    Right_Back.tare_position();
// sets encoder values to 0;
}

void StopBase(){
    Left_Front.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    Left_Middle.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    Left_Back.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    Right_Front.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    Right_Middle.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    Right_Back.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // Sets what type of brake mode the motors should have
    Left_Front.brake();
    Left_Middle.brake();
    Left_Back.brake();
    Right_Front.brake();
    Right_Middle.brake();
    Right_Back.brake();
    // Makes the motors stop
}

double KeepInRange(float Value, double MinValue, double MaxValue)
{
 if(Value < MinValue){ // if Value is less than Minimum Value
    Value = MinValue; // Set Value to Minimum Value
 }
 if(Value > MaxValue){ // if Value is greater than Maximum Value
    Value = MaxValue; // Ser Value to Maximum Value
 }
 return Value; // Reutn Value
}

float GetHeading(){
  return gyro.get_rotation(); // Returns the degree it is facing on the inertial sensor
}

//                                //
//                                //
//          PID Controls          //
//                                //
//                                //

// PID_Controller_turn: A function to control turning of a robot using a PID controller.
void PID_Controller_turn(float MaxSpeed, float Target, float Timeout)
{ 
// Inputs:
  // - MaxSpeed: The maximum speed at which the robot can turn
  // - Target: The target angle to reach, relative to the current position
  // - Timeout: The time allowed for the function to execute
  gyro.set_rotation(0); // Reset the gyro sensor's rotation to 0
  Target = GetHeading() + Target; // Calculate the absolute target angle by adding current heading to Target
  float error = Target - GetHeading(); // Calculate initial error (difference between target and current heading)
  
  // PID control variables
  float integral; // Accumulated error for integral term
  float derivative; // Change in error for derivative term
  float prevError; // Previous error for calculating derivative
  float kP = 1.5; // Proportional constant
  float kI = 0; // Integral constant
  float kD = 0; // Derivative constant
  float FinalPower; // The final power/speed output
  float active_zone = 15; // The range within which the integral term is active to avoid overshooting
  
  float power_limit = 3 / kI; // Limits the integral term to prevent excessive speed adjustments
  float ExitThreshold = 0.5; // Minimum error threshold to exit loop (when within 0.5 degrees of target)
  
  ClearEncorders(); // Reset encoders to start fresh position measurement
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5, 1, "Gyro: %3f", GetHeading()); // Display current gyro heading

  // Main control loop - runs until the error is within the ExitThreshold
  while(fabs(error) > ExitThreshold) {
    error = Target - GetHeading(); // Recalculate error each loop iteration

    // Accumulate integral only if error is within active_zone and not zero
    if(fabs(error) < active_zone && error != 0) {
      integral = error + integral; // Update integral term
    }
    else {
      integral = 0; // Reset integral if out of active_zone to avoid overshoot
    }
    integral = KeepInRange(integral, -power_limit, power_limit); // Limit integral term to power_limit range
    
    derivative = error - prevError; // Calculate derivative as the difference from previous error
    prevError = error; // Store current error as previous for next iteration
    
    // Compute the final motor power using PID formula
    FinalPower = ((kP * error) + (kI * integral) + (kD * derivative));
    FinalPower = KeepInRange(FinalPower, -MaxSpeed, MaxSpeed); // Limit FinalPower to maximum speed  
    
    // Apply calculated power to the motors to turn the robot
    Left_Front.move(FinalPower);
    Left_Middle.move(FinalPower); 
    Left_Back.move(FinalPower);
    Right_Front.move(-FinalPower);
    Right_Middle.move(-FinalPower); 
    Right_Back.move(-FinalPower);
    
    // Print current error and gyro heading to screen for debugging
    pros::screen::print(pros::E_TEXT_LARGE, 5, 20, "ERROR: %f", error);
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, 40, "Gyro: %3f", GetHeading());
    
    // Display the target heading on the controller screen for debugging
    // master.clear_line(1);
    // master.print(1, 1, "Heading: %f", Target);
    
    pros::delay(20); // Small delay to prevent excessive processor usage
  }
  
  StopBase(); // Stop all motors once the target is reached
}

// PID_Controller_move: A function to control straight movement of a robot using a PID controller.
void PID_Controller_move(float MaxSpeed, double Value, float Timeout) {
// Inputs:
  // - MaxSpeed: The maximum speed at which the robot can move
  // - Value: The target distance in inches to move
  // - Timeout: The time allowed for the function to execute
  
  // Calculate the initial error as the difference between target position (converted to ticks) and current position
  double error = inchesToTicks(Value) - getBaseAvg(); 
  
  // PID control variables
  float integral;     // Accumulated error for the integral term
  float derivative;   // Change in error for the derivative term
  float prevError;    // Previous error for calculating derivative term
  float ExitThreshold = 0.5; // Minimum error threshold for exiting the loop
  float FinalPower;   // Final motor power/speed output
  
  // PID tuning constants
  float kP = 1.35;  // Proportional constant
  float kI = 0.00;  // Integral constant
  float kD = 0.22;  // Derivative constant
  
  float active_zone = 15; // Zone within which the integral term is active to help reach target smoothly
                           // (Used to prevent overshooting when close to the target)

  float power_limit = 40 / kI; // Limits the effect of the integral term to avoid excessive adjustments

  // ClearEncoders: Resets motor encoders to ensure accurate position tracking
  ClearEncorders();

  // Main control loop - continues until the error is within ExitThreshold
  while(fabs(error) > ExitThreshold) {
    error = inchesToTicks(Value) - getBaseAvg(); // Recalculate error based on current position

    // Only accumulate integral if within active zone and error is non-zero
    if (fabs(error) < active_zone && error != 0) {
      integral = integral + error; // Accumulate integral term
    } else {
      integral = 0; // Reset integral if out of active zone
    }
    integral = KeepInRange(integral, -power_limit, power_limit); // Limit integral term to prevent instability

    derivative = error - prevError; // Calculate derivative as the change in error
    prevError = error; // Store current error as previous error for next loop

    // Calculate the final power using PID formula
    FinalPower = ((kP * error) + (kI * integral) + (kD * derivative));
    FinalPower = KeepInRange(FinalPower, -(fabs(MaxSpeed)), fabs(MaxSpeed)); // Limit final power to MaxSpeed

    // Move motors to apply calculated power and move the robot forward
    Left_Front.move(FinalPower);
    // Left_Middle.move(FinalPower); // Uncomment if additional motors are needed
    Left_Back.move(FinalPower);
    Right_Front.move(FinalPower);
    // Right_Middle.move(FinalPower); // Uncomment if additional motors are needed
    Right_Back.move(FinalPower);

    // Print the current error for debugging purposes
    pros::screen::print(pros::E_TEXT_LARGE, 5, 20, "ERROR: %f", error);

    pros::delay(20); // Small delay to prevent excessive processor usage
  }

  StopBase(); // Stop all motors once the target position is reached
}

// Movingcommand: A function to move the robot forward or backward at a given speed for a specified duration.
void Movingcommand(int Time, int speed) {
// Inputs:
  // - Time: The duration in milliseconds for which the robot should move
  // - speed: The speed at which the robot should move (positive for forward, negative for backward)

  // Set each motor to move at the specified speed
  Left_Front.move(speed);   // Move the left front motor
  Left_Middle.move(speed);  // Move the left middle motor
  Left_Back.move(speed);    // Move the left back motor
  Right_Front.move(speed);  // Move the right front motor
  Right_Middle.move(speed); // Move the right middle motor
  Right_Back.move(speed);   // Move the right back motor

  pros::delay(Time); // Delay for the specified time to keep the robot moving

  StopBase(); // Stop all motors after the delay to end the movement
}

//                                //
//                                //
//        Intake Controls         //
//                                //
//                                //

// Initialize toggle variables for intake system control
int IntakeToggleV = -1;         // Tracks the forward toggle state (1 for on, -1 for off)
int IntakeReverseToggleV = -1;   // Tracks the reverse toggle state (1 for reverse on, -1 for off)

void IntakeToggle() {
// If `IntakeToggleV` is 1, the intake motors will run at full speed
// If `IntakeToggleV` is -1, the intake motors will stop
  
  // Toggle the `IntakeToggleV` state each time this function is called
  IntakeToggleV = -1 * IntakeToggleV;
  
  // If IntakeToggleV is 1, move both intake motors forward at full speed
  if (IntakeToggleV == 1) {
    //Intakefirst.move(127);      // Move the first intake motor at full speed
    Test = 1;
  }
  
  // This condition attempts to check if both toggles (forward and reverse) are active,
  // but `IntakeToggleV == 1` is redundant here since it was already checked in the previous `if` condition.
  else if (IntakeReverseToggleV == 1 && IntakeToggleV == 1) {
    Intakefirst.move(127);      // Move the first intake motor at full speed
    
  }
  
  // If IntakeToggleV is -1, stop both intake motors
  else {
    //Intakefirst.move(0);        // Stop the first intake motor
    Test = 0;
  }
}

// IntakeToggle: A function to toggle the intake motors on or off, depending on the current state of `IntakeToggleV`
void IntakeReverseToggle(){
// If `IntakeToggleV` is 1, the intake motors will run reverse at full speed
// If `IntakeToggleV` is -1, the intake motors will stop
  
  // Toggle the `IntakeReverseToggleV` state each time this function is called
  IntakeReverseToggleV = -1 * IntakeReverseToggleV;

  // If IntakeToggleV is 1, move both intake motors reverse at full speed
  if(IntakeReverseToggleV == 1){
    Intakefirst.move(-127); // moves the first intake motor reverse at full speed
   Test = 2;
  }
  // This condition attempts to check if both toggles (forward and reverse) are active,
  // but `IntakeReverseToggleV == 1` is redundant here since it was already checked in the previous `if` condition.
  else if(IntakeReverseToggleV == 1 && IntakeToggleV == 1){
    //Intakefirst.move(-127); // Move the first intake motor reverse at full speed
    Test = 2;
  }
  // If IntakeToggleV is -1, stop both intake motors
  else{
    //Intakefirst.move(0); // Stops the first intake motor
   Test = 0;
  }
}

void IntakeControls(){
  while(true){
   if(Test == 1){
    Intakefirst.move(127);
    IntakeSecond.move(127);
   }
   else if(Test == 2){
    Intakefirst.move(-127);
    IntakeSecond.move(-127);
   }
   else{
    Intakefirst.move(0);
    IntakeSecond.move(0);
   }
  pros::delay(20);
  }
}

// Clamp Controls

void ClampToggle(){
 ClampV = -1 * ClampV;
 if(ClampV == 1){
  Clamp.set_value(1);
 }
 else if(ClampV == -1){
  Clamp.set_value(0);
 }
 
}

//                                //
//                                //
//     Intake Piston Controls     //
//                                //
//                                //

// Initialize a toggle variable for the intake piston control
int IntakePistonV = -1; // Tracks the toggle state of the intake piston (1 for on, -1 for off)

// IntakePistonToggle: A function to toggle the intake piston and intake motors on or off based on the state of `IntakePistonV`
void IntakePistonToggle() {
  
  // Toggle the `IntakePistonV` state each time this function is called
  IntakePistonV = -1 * IntakePistonV;
  
  // If IntakePistonV is 1, activate the intake piston and start the intake motors
  if (IntakePistonV == 1) {
    Intake_P.set_value(1);       // Set the intake piston to its active position
    // Intakefirst.move(127);       // Move the first intake motor at full speed
    // Intakesecond.move(127);      // Move the second intake motor at full speed
  }
  
  // If IntakePistonV is -1, deactivate the intake piston and stop the intake motors
  else if (IntakePistonV == -1) {
    Intake_P.set_value(0);       // Set the intake piston to its inactive position
    // Motors are not set here, so they will continue in their current state (no specific command to stop them)
  }
}

//                                //
//                                //
//  Color Sorter Piston Controls  //
//                                //
//                                //

// int CSV = -1; // Toggle state variable for the color sorter (1 for on, -1 for off)

// // ColorSorter_Toggle: A function to toggle the color sorter mechanism on or off
// void ColorSorter_Toggle() {
  
//   // Toggle the CSV state each time this function is called
//   CSV = CSV * -1;
  
//   // If CSV is 1, activate the color sorter
//   if (CSV == 1) {
//     Color_sorter.set_value(1); // Set the color sorter to its active position
//   }
  
//   // If CSV is -1, deactivate the color sorter
//   else if (CSV == -1) {
//     Color_sorter.set_value(0); // Set the color sorter to its inactive position
//   }
// }

//                                //
//                                //
//        Doinker Controls        //
//                                //
//                                //

int RightDoinkerV = -1;  // Toggle state variable for the Doinker (1 for on, -1 for off)

// RightDoinker_Toggle: A function to toggle the Right Doinker mechanism on or off
void RightDoinker_Toggle(){

  // Toggle the CSV state each time this function is called
  RightDoinkerV = RightDoinkerV * -1;

  // If RightDoinkerV is 1, activate the color sorter
  if (RightDoinkerV == 1){
    RightDoinker.set_value(1); // Set the Right Doinker to its active position
  }

  // If RightDoinkerV is -1, deactivate the color sorter
  else if (RightDoinkerV == -1){
    RightDoinker.set_value(0); // Set the Right Doinker to its inactive position
  }
}




//                                //
//                                //
//               Arm              //
//                                //
//                                //


void Arm_Set_Toggle(){

  Arm.move_absolute(350,127);  

}

void Arm_Out(){
  Arm.move_absolute(2500 ,127);
}

bool Arm_PS_Bool = false;
void Arm_past_score(){
 Arm_PS_Bool = !Arm_PS_Bool;
 if(Arm_PS_Bool == true){
  Arm.move_absolute(1420, 127);
 }
 else{
  Arm.move_absolute(1820, 127);
 }
}

bool Arm_DR_bool = false;
bool Arm_Score_bool = false;
void Arm_Descore_Reset(){
  //Arm_PS_Bool = false;
  Arm_DR_bool = !Arm_DR_bool;
  if(Arm_DR_bool == true){
    Arm.move_absolute(0,127);
    Arm_Score_bool = false;
  }
  else{
    Arm.move_absolute(1100, 127);
  }
}

void Arm_score(){
  Arm_PS_Bool = false;
  Arm_Score_bool = !Arm_Score_bool;

  if(Arm_Score_bool == true){
    Arm.move_absolute(200, 127);
    Arm_DR_bool=false;
  }
  else{
    IntakeSecond.move(-20);
   pros::delay(50);
   IntakeSecond.move(0);
    Arm.move_absolute(1200, 127);
  }
}




void Arm_Toggle(){
  ArmV = ArmV +1;
  if(ArmV == 1){
    Arm_Set_Toggle();
  }
  else if(ArmV == 2){
    Intakefirst.move(-20);
   pros::delay(50);
   Intakefirst.move(0);
    Arm.move_absolute(1300, 127);
  }
  else if(ArmV == 3){
    Arm.move_absolute(2100, 127);
  }
  else if(ArmV == 4){
    Arm.move_absolute(2500, 127);
  }
  else if(ArmV == 5){
    Arm.move_absolute(3200, 127);
    ArmV = 0;
  }
}

//                                //
//                                //
//              Tasks             //
//                                //
//                                //

// Get_Color: A variable to return the hue of a color
double Get_Color(){
  return OP.get_hue(); // Return hue that the optical sensor sees
}

// RedColorSensor_Task: A function that continuously monitors a color sensor to control a color sorter mechanism
// This function runs in an infinite loop, checking for red and blue hues and activating or deactivating the sorter based on the detected color.
void RedColorSensor_Task(){
   
  while (true) {

    if (Test == 1){
      int hue = OP.get_hue();

            // Check if the hue corresponds to red (typical red hue is around 0-30 degrees)
      if (hue >= 61) {
          // Reverses intake for a bit
          pros::delay(160); // Delays for 120 ms
          Intakefirst.move(-127); // Reverses inatke
          pros::delay(180); // Delay for 300 ms
      }
      else if(hue <= 60){
        Intakefirst.move(127);
        IntakeSecond.move(127);
      }
    }
    else if (Test ==2){
      Intakefirst.move(-127); // Spin Inakte first motor to 127 voltages (forwards)
      IntakeSecond.move(-127);
    }
    else if(Test == 3){
      IntakeSecond.move(127);
    }
    else{
      Intakefirst.move(0);
      IntakeSecond.move(0);
    }
       

   pros::delay(20); // Delay to prevent excessive CPU usage
    

   }
}


// RedColorSensor_Task: A function that continuously monitors a color sensor to control a color sorter mechanism
// This function runs in an infinite loop, checking for red and blue hues and activating or deactivating the sorter based on the detected color.
void BlueColorSensor_task(){
   
  while (true) {

    if (Test == 1){
      int hue = OP.get_hue();

            // Check if the hue corresponds to red (typical red hue is around 0-30 degrees)
      if (hue <= 20) {
          // Reverses intake for a bit
          pros::delay(120); // Delays for 120 ms
          Intakefirst.move(-127); // Reverses inatke
          IntakeSecond.move(-127); // Reverses intake 
          pros::delay(150); // Delay for 300 ms
          
      }
      else{
        Intakefirst.move(127);
        IntakeSecond.move(127);
        // Testing 
        if (Intakefirst.get_actual_velocity() == 0) {
                // Reverse the intake for 200 ms
                Intakefirst.move(-127);
                IntakeSecond.move(-127);
                pros::delay(220);
                // Resume normal intake operation
                Intakefirst.move(127);
            }
      }
    }
    else if (Test ==2){
      Intakefirst.move(-127); // Spin Inakte first motor to 127 voltages (forwards)
      IntakeSecond.move(-127);
    }
    else if(Test == 3){
      IntakeSecond.move(127);
    }
    else{
      Intakefirst.move(0);
      IntakeSecond.move(0);
    }
       

   pros::delay(20); // Delay to prevent excessive CPU usage
    

   }
}

// Boolean variable to control the automatic clamp mode; initially set to true
bool AutoClampBool = true;

bool pistonState = false; // Track piston state for toggling

// ClampOut: A function that controls a clamp mechanism based on limit switch inputs, 
//           controller button presses, and the AutoClampBool mode.
int countign = 0;
void ClampOut() {
  
   while (true) {
        // Read the current states of limit switches and the controller button
        // bool DistanceLow = Distance_S.get_distance() < 120; // Check if LimitSwitch1 is pressed
        // // bool limit2Pressed = LimitSwitch2.get_value(); // Check if LimitSwitch2 is pressed
        bool buttonBPressed = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B); // Check if Button B is pressed

        if (Distance_S1.get_distance() < 170 && Disntance_S2.get_distance() < 170) {
            // If both limit switches are pressed, activate the clamp
            pistonState = true;   // Update the piston state as active
            Clamp.set_value(pistonState); // Extend the piston

            if (buttonBPressed) {
              // If Button B is pressed while both switches are active, deactivate the piston
              // pistonState = false;    // Update the piston state as inactive
              // Clamp.set_value(pistonState); // Retract the piston
              while(true){
                countign++;
                master.print(0, 0, "Counting: %d", countign);
               if(Distance_S1.get_distance() < 170 || Disntance_S2.get_distance() < 170){
                pistonState = false;
                Clamp.set_value(pistonState);
               }
               else if(!(Distance_S1.get_distance() < 170 || Disntance_S2.get_distance() < 170)){
                pistonState = false;
                Clamp.set_value(pistonState);
                break;
               }
               pros::delay(20);
            }
            }
         } 
        else if (buttonBPressed) {
            // If Button B is pressed
            // if (Distance_S1.get_distance() > 80 && Disntance_S2.get_distance() > 80) {
                // If neither limit switch is pressed, toggle the piston state
                pistonState = !pistonState; // Toggle the piston state
                Clamp.set_value(pistonState); // Set the piston based on the new state
                pros::delay(20);       // Short delay to prevent rapid state changes
            // } else if (Distance_S1.get_distance() < 80 && Disntance_S2.get_distance() < 80) {
            //     // If either limit switch is pressed, deactivate the piston
            //     Clamp.set_value(false); // Retract the piston
            //     pistonState = false;    // Update the piston state as inactive
             //}
        }

        // Add a short delay to prevent excessive looping and CPU usage
        pros::delay(20);
    }
}



//AutonClampOut: Controls the clamp mechanism autonomously based on limit switch inputs.
void AutonClampOut() {
  while (true) {  
    // Check if either limit switch is pressed
    if (Distance_S1.get_distance() < 130 && Disntance_S2.get_distance() < 130) {
        // If at least one switch is pressed, activate the clamp
        Clamp.set_value(1); // Extend the piston to hold an object
    } 
    pros::delay(20); // Add a small delay to reduce CPU usage in this loop
  }
}

// Initialize CancelV to -1, used to track toggling of the AutonClampCancel function
int CancelV = -1;

//AutonClampCancel: Toggles the autonomous clamp mechanism on/off.
void AutonClampCancel() {
  // Toggle CancelV between -1 and 1
  CancelV = CancelV * -1;

  if (CancelV == 1) {
    // If CancelV is 1, suspend the autonomous clamp task and deactivate the clamp
    AutonAutoClamp.suspend(); // Pause the autonomous clamp task
    Clamp.set_value(0);       // Retract the clamp piston
  } else if (CancelV == -1) {
    // If CancelV is -1, resume the autonomous clamp task
    AutonAutoClamp.resume(); // Resume the autonomous clamp task
  }
}

// Task Toggle: A function that toggles between suspending and resuming tasks based on a toggle variable
int TaskV = -1; // Toggle state variable for task control (1 to suspend Blue_Mode, -1 to resume it)

void Task_Toggle() {
  
  // Toggle the state of TaskV each time this function is called
  TaskV = TaskV * -1;
  
  // If TaskV is 1, suspend the Blue_Mode task
  if (TaskV == 1) {
    Red_Mode.suspend(); // (Commented out) This line would suspend Red_Mode if uncommented
    //Blue_Mode.suspend();   // Suspend the Blue_Mode task
    IntakeC.resume(); // Resume the IntakeC task
  }
  
  // If TaskV is -1, resume the Blue_Mode task
  else if (TaskV == -1) {
    Red_Mode.resume(); // (Commented out) This line would resume Red_Mode if uncommented
    //Blue_Mode.resume();    // Resume the Blue_Mode task
    IntakeC.suspend(); // Resume the IntakeC task
  }
}

void Anti_Jam(){

 while (true) {
        // Check if the R2 button is pressed
        if (Test == 1) { // if intake is on
            // Move the intake forward
            Intakefirst.move(127);
            IntakeSecond.move(127);
            pros::delay(50); // delay to prevent reversing when it starts

            // Check if the voltage of the intake equals 0
            if (IntakeSecond.get_actual_velocity() == 0) {
                // Reverse the intake for 200 ms
                IntakeSecond.move(-127);
                pros::delay(220);
                // Resume normal intake operation
                IntakeSecond.move(127);
            }
        }
        else if(Test == 2){ // if intake is on reverse
          // Move the intake in reverse
          Intakefirst.move(-127);
          IntakeSecond.move(-127);
        } 
        else if (Test==3) {
          Intakefirst.move(127);
          IntakeSecond.move(0);
        
        }
        else if(Test == 0) { // if intake is off
            // Stop the intake
            Intakefirst.move(0);
        }

        pros::delay(20); // Delay to prevent excessive CPU usage
  }
}

void Task_Creation_Toggle(){
  bool Mode = false;
  Mode = !Mode;
  if(Mode == true){
    Blue_Mode.remove();
    Red_Mode.create(RedColorSensor_Task);
  }
  else{
    Red_Mode.remove();
    Blue_Mode.create(BlueColorSensor_task);
  }
}
