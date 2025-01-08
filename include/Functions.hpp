//PID Settings
#include "pros/rtos.hpp"
extern double inchesToTicks(double inches); 
extern double ticksToInches(double ticks);
extern double timer();

extern double getBaseAvg();
extern void ClearEncorders();
extern void StopBase();

extern double KeepInRange(double Value, double MinValue, double MaxValue);

extern float GetHeading();

// PID Controller
extern void PID_Controller_turn(float MaxSpeed, float Target, float Timeout);
extern void PID_Controller_move(float MaxSpeed, double Value, float Timeout);
extern void PID_Controller_Slew(float RightMaxSpeed, float LeftMaxSpeed, float Target, float Timeout);

//Basic movement
extern void Movingcommand(int time, int speed);

// Intake controls
extern void IntakeToggle();
extern void IntakeReverseToggle();

// Clamp controls
inline int ClampV = -1;
extern void ClampToggle();

// Intake Piston Controls
extern void IntakePistonToggle();

// Color Sorter Piston Controls
extern void ColorSorter_Toggle();

// Doinker Controls
extern void RightDoinker_Toggle();
extern void LeftDoinker_Toggle();

// Arm Controls
extern void Arm_Set_Toggle();
extern void Arm_Out();
extern void Arm_Toggle();


// Tasks
extern double Get_Color();
extern void RedColorSensor_Task();
extern void BlueColorSensor_task();
extern void ClampOut();
extern void AutonClampOut();
extern void AutonClampCancel();
extern void Task_Toggle();
extern void Anti_Jam();
inline int Test = 0;
extern void Blue_v_Red();
inline bool Mode = false;



//inline pros::Task Red_Mode(RedColorSensor_Task); // Creates a task to detect Red
//inline pros::Task Blue_Mode(BlueColorSensor_task); // Creates a task to detect Blue
//inline pros::Task AutoClamp(ClampOut); // Creates a task for auto clamp
//inline pros::Task AutonAutoClamp(AutonClampOut); // Creates a task for auton auto clamp
inline pros::Task NOJAM(Anti_Jam); // Creates a task for intake
//inline pros::Task ColorSide(Blue_v_Red);




