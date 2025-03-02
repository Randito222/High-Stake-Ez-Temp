#pragma once


#include "EZ-Template/drive/drive.hpp"

extern Drive chassis;

void default_constants();

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();

extern void Blue_Negative_AWP();
extern void Blue_Positive_AWP();
extern void Red_Negative_AWP();
extern void Red_Positive_AWP();
extern void Blue_Single_Goal();
extern void Red_Single_Goal();
extern void Blue_Rush();
extern void Red_Rush();
extern void SigAutoRN();
extern void SigAutoBN();
extern void SigAWPR();
extern void SigAWPB();
extern void SigAutoRP();
extern void SigAutoBP();
extern void ElimBluePos();
extern void ElimBlueNeg();
extern void DistruptionAuto();
extern void GoalRush();
extern void Skills();
extern void SkillsV2();