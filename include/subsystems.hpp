#pragma once

#include "api.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

// Your motors, sensors, etc. should go here.  Below are examples

// Motors

inline pros::Motor Left_Front(-7);
inline pros::Motor Left_Middle(-8);
inline pros::Motor Left_Back(-9);
inline pros::Motor Right_Front(13);
inline pros::Motor Right_Middle(17);
inline pros::Motor Right_Back(19);
inline pros::Motor Intakefirst(-15);   
inline pros::Motor IntakeSecond(2);
inline pros::Motor Arm(12);


// Pneumatics

inline pros::adi::DigitalOut Clamp ('C',false);
inline pros::adi::DigitalOut Intake_P ('D',false);
inline pros::adi::DigitalOut PTO ('B',false);
inline pros::adi::DigitalOut LeftDoinker('H',false);
inline pros::adi::DigitalOut RightDoinker('F',false);


// Sensors

inline pros::IMU gyro(12);
inline pros::Optical OP(14);
//inline pros::adi::DigitalIn LimitSwitch1 ('G');
inline pros::adi::DigitalIn LimitSwitch2 ('E');
inline pros::ADIAnalogIn LineTrack('A');
inline pros::ADIAnalogIn RingStopper('G');
inline pros::Distance RingStop(1);
