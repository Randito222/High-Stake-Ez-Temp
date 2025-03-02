#pragma once

#include "api.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

// Your motors, sensors, etc. should go here.  Below are examples

// Motors

inline pros::Motor Left_Front(-17);
inline pros::Motor Left_Middle(-18);
inline pros::Motor Left_Back(-19);
inline pros::Motor Right_Front(9);
inline pros::Motor Right_Middle(7);
inline pros::Motor Right_Back(8);
inline pros::Motor Intakefirst(10);   
inline pros::Motor IntakeSecond(-1);
inline pros::Motor Arm(-5);


// Pneumatics

inline pros::adi::DigitalOut Clamp ('D',false);
inline pros::adi::DigitalOut Intake_P ('H',false);
inline pros::adi::DigitalOut PTO ('C',false);
inline pros::adi::DigitalOut RightDoinker('E',false);


// Sensors

inline pros::IMU gyro(12);
inline pros::Optical OP(3);
inline pros::adi::DigitalIn LimitSwitch1 ('G');
inline pros::adi::DigitalIn LimitSwitch2 ('B');
inline pros::ADIAnalogIn LineTrack('A');
//inline pros::ADIAnalogIn RingStopper('G');
inline pros::Distance Distance_S1(13);
inline pros::Distance Disntance_S2(20);
inline pros::adi::DigitalIn LBReset({14,'A'});