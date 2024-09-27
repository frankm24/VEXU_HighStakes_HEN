/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       University of Delaware Team 1                             */
/*    Created:      5/17/2024, 2:33:21 PM                                     */
/*    Description:  Software for Robot HEN of UD's VEXU Team                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include <iostream>
#include "vex.h"
#include <math.h>

using namespace vex;


// Definitions
#define TURN_SPEED_RATIO 0.5

// Global Constants
// Make sure to define a motor with the right gear ratio (motor gear color)
const gearSetting RED_GEAR = ratio36_1; // 100 RPM - high torque & low speed (e.g. lifting arms & moving claws,)
const gearSetting GREEN_GEAR = ratio18_1; // 200 RPM - standard gear ratio for drivetrain applications 
const gearSetting BLUE_GEAR = ratio6_1; // 600 RPM - low torque & high speed (e.g.  intake rollers & flywheels))

// A global instance of vex::brain
vex::brain Brain;

// Global instance of competition
competition compete;

// Global instance of controller
controller primary_controller = controller(primary);

// define your global instances of motors and other devices here
motor left_motor_front = motor(PORT1, GREEN_GEAR, false);
motor left_motor_mid = motor(PORT3, GREEN_GEAR, false);
motor left_motor_back = motor(PORT5, GREEN_GEAR, false);
motor_group left_motor_group = motor_group(left_motor_front, left_motor_mid, left_motor_back);

motor right_motor_front = motor(PORT2, GREEN_GEAR, true);
motor right_motor_mid = motor(PORT4, GREEN_GEAR, true);
motor right_motor_back = motor(PORT6, GREEN_GEAR, true);
motor_group right_motor_group = motor_group(right_motor_front, right_motor_mid, right_motor_back);

// Code block for Pre-Autonomous 
void pre_auton(void) {
    Brain.Screen.print("Pre-Autonomous start!");
    Brain.Screen.newLine();


    // All activities that occur before the competition starts
    Brain.Screen.print("Pre-Autonomous complete.");
    Brain.Screen.newLine();
}

// Code block for Autonomous
void autonomous(void) {
    Brain.Screen.print("Autonomous start!");
    Brain.Screen.newLine();

    // Autonomous loop
    while(true){

    }
}

void dual_stick_drive(void){

    // Controls for Up-Down and Left-Right movement
    float leftStick = (float)(primary_controller.Axis3.position() / 100.0);             // Vertical Movement
    float rightStick = primary_controller.Axis1.position() / (float)100.0;            // Horizontal Movement
   
    
    // Motor speed percentage based on cubed function
    float ySpeed = pow(leftStick, 3);
    float xSpeed = pow(rightStick /*TURN_SPEED_RATIO*/, 3);

    //if(controller.buttonL2.pressing()):
        //xSpeed = (rightAxis_LR * SHIFT_TURN_SPEED_RATIO) ** 3

    
    if(fabs(ySpeed) > 0.05 or fabs(xSpeed) > 0.05){
        printf("Left Stick: %f, Right Stick: %f\n", ySpeed, xSpeed);

        //Set the velocity depending on the axis position
        left_motor_group.setVelocity((ySpeed + xSpeed) * 100,vex::percentUnits::pct);
        right_motor_group.setVelocity((ySpeed - xSpeed) * 100,vex::percentUnits::pct);

        left_motor_group.spin(forward);
        right_motor_group.spin(forward);

    }
    else{
        left_motor_group.stop(coast);
        right_motor_group.stop(coast);
    }

    return;
}

// Code block for User Control
void usercontrol(void) {
    Brain.Screen.print("User Control start!");
    Brain.Screen.newLine();
    
    // Usercontrol loop
    while(true){
        dual_stick_drive();
    }
}

int main() {
    Brain.Screen.print("Main start!");
    Brain.Screen.newLine();
    // Set up callbacks for autonomous and driver control periods.
    compete.autonomous(autonomous);
    compete.drivercontrol(usercontrol);

    // Run the pre-autonomous function.
    pre_auton();

    // Prevent main from exiting with an infinite loop.
    while (true) {
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
    
    Brain.Screen.print("Main complete.");
    Brain.Screen.newLine();
}