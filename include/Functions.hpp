//PID Settings
#include "okapi/api.hpp"
#include "pros/rtos.hpp"

using namespace okapi;


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
extern void IntakeControls();

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
inline int ArmV=0;
extern void Arm_score();
extern void Arm_past_score();
extern void Arm_Descore_Reset();
extern void Arm_Toggle();
extern void Arm_emergency_reset();


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



inline bool AUTON = false;
inline pros::Task Red_Mode(RedColorSensor_Task); // Creates a task to detect Red
inline pros::Task Blue_Mode(BlueColorSensor_task); // Creates a task to detect Blue
//inline pros::Task AutoClamp(ClampOut); // Creates a task for auto clamp
inline pros::Task AutonAutoClamp(AutonClampOut); // Creates a task for auton auto clamp
//inline pros::Task NOJAM(Anti_Jam); // Creates a task for intake
inline pros::Task IntakeC(IntakeControls); // Creates a task for intake when no color sorter

inline void Task_Creation_Toggle();


inline std::shared_ptr<ChassisController> myChassis =
  ChassisControllerBuilder()
    .withMotors({1, 2}, {-3, -4})
    // Green gearset, 4 in wheel diam, 11.5 in wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
    .build();

inline std::shared_ptr<AsyncMotionProfileController> profileController = 
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(myChassis)
    .buildMotionProfileController();




