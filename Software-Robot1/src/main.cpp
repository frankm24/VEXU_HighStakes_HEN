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
#define BELT_THROW_POSITION 638 // Farthest Number of Degrees from starting position needed to throw ring (No more than 1 revolution around BELT)
#define BELTRANGE 7 // Margin of error around throw position
#define BELTSPEED -85


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
motor left_motor_mid = motor(PORT2, GREEN_GEAR, false);
motor left_motor_back = motor(PORT3, GREEN_GEAR, false);
motor_group left_motor_group = motor_group(left_motor_front, left_motor_mid, left_motor_back);

motor right_motor_front = motor(PORT6, GREEN_GEAR, true);
motor right_motor_mid = motor(PORT7, GREEN_GEAR, true);
motor right_motor_back = motor(PORT8, GREEN_GEAR, true);
motor_group right_motor_group = motor_group(right_motor_front, right_motor_mid, right_motor_back);

motor intake_motor = motor(PORT19, GREEN_GEAR, false);
motor belt_motor = motor(PORT16, GREEN_GEAR, false);

// Global Variables
volatile bool belt_toggle_state = false;
volatile bool color_detected = true; // TODO: Set up control to vision sensor

void intake_toggle(void){
    std::cout<<"Intake Toggle"<<std::endl;
}

void belt_toggle_on(void){
    std::cout<<"Belt Toggle On"<<std::endl;
    belt_toggle_state = true;

}

void belt_toggle_off(void){
    std::cout<<"Belt Toggle Off"<<std::endl;
    belt_toggle_state = false;
}

void belt_control(void){
    while(true){
        int belt_position = abs((((int)belt_motor.position(vex::rotationUnits::deg)) % BELT_THROW_POSITION));
        Brain.Screen.printAt(1, 150, "Belt Position: %6d", belt_position);
        belt_motor.position(vex::rotationUnits::deg);

        if(color_detected && belt_position >= -BELTRANGE/3 && belt_position <= BELTRANGE){
            belt_motor.stop(vex::brakeType::brake);
            wait(1, sec);
            while(belt_position >= 0 && belt_position <= BELTRANGE){
                belt_motor.setVelocity(BELTSPEED, vex::percentUnits::pct);
                belt_motor.spin(forward);
                belt_position = abs((((int)belt_motor.position(vex::rotationUnits::deg)) % BELT_THROW_POSITION));
            }
        }
        if(belt_toggle_state){
            belt_motor.setVelocity(BELTSPEED, vex::percentUnits::pct);
            belt_motor.spin(forward);  
        }
        else{
            belt_motor.stop(brake);
        }


    }
}

// Color Sensor
vision::signature CUS_BLUE = vision::signature(7, -4767, -3699, -4233, 6899, 8623, 7761, 3.2, 0);
vision::signature CUS_RED = vision::signature(6, 8849, 11299, 10074, -1761, -911, -1336, 1.9, 0);

vision vSens = vision(PORT20, 50, CUS_RED);

// Enum to track the current vision state
enum VisionState {
    RED,
    BLUE,
    OFF
};

VisionState currentState = RED; // Start with red vision


// Function to display the current status on the brain screen
void displayStatus() {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    
    switch (currentState) {
        case RED:
            Brain.Screen.print("Ejecting Red Rings");
            break;
        case BLUE:
            Brain.Screen.print("Ejecting Blue Rings");
            break;
        case OFF:
            Brain.Screen.print("Ejection Off");
            return; 
    }
}

// Main program loop
int vision_sensor_thread() {
    std::cout<<(int)vSens.getBrightness()<<std::endl;
    vSens.setBrightness((uint8_t) 50);
    std::cout<<(int)vSens.getBrightness()<<std::endl;
    while (true) {
        // Check if Button A is pressed to toggle vision state
        if (primary_controller.ButtonA.pressing()) {
            // Cycle through the states: RED -> BLUE -> OFF -> RED
            currentState = static_cast<VisionState>((currentState + 1) % 3);

            // Clear previous snapshots when turned off
            //if (currentState == OFF) vSens.setMode;

            // Wait a short period to prevent multiple toggles
            this_thread::sleep_for(200);
        }

        // Take a snapshot if vision is active
        if (currentState != OFF) vSens.takeSnapshot(currentState == RED ? CUS_RED : CUS_BLUE);

        // Display the current status on the screen
        displayStatus(); 
        //std::cout<<(int)vSens.objectCount<<std::endl;
        // Check if an object is detected
        if (vSens.objects[0].exists) {
            // TODO: Add code to eject ring
            color_detected = true;
            std::cout<<"Color Detected!"<<std::endl;
        }
        else{
            color_detected = false;
        }
        this_thread::sleep_for(100); 
    }
}


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
    float leftStick = -(float)(primary_controller.Axis3.position() / 100.0);             // Vertical Movement
    float rightStick = primary_controller.Axis1.position() / (float)-100.0;            // Horizontal Movement
   
    
    // Motor speed percentage based on cubed function
    float ySpeed = pow(leftStick, 3);
    float xSpeed = pow(rightStick /*TURN_SPEED_RATIO*/, 3);

    //if(controller.buttonL2.pressing()):
        //xSpeed = (rightAxis_LR * SHIFT_TURN_SPEED_RATIO) ** 3

    
    if(fabs(ySpeed) > 0.05 or fabs(xSpeed) > 0.05){
        //printf("Left Stick: %f, Right Stick: %f\n", ySpeed, xSpeed);

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
        bool buttonR1 = primary_controller.ButtonR1.pressing();
        bool buttonR2 = primary_controller.ButtonR2.pressing();
        bool buttonL1 = primary_controller.ButtonL1.pressing();
        bool buttonL2 = primary_controller.ButtonL2.pressing();
        

        
       
        

        if(buttonR1 && !buttonR2){
            intake_motor.setVelocity(-100, vex::percentUnits::pct);
            intake_motor.spin(forward);
        }
        else if(buttonR2 && !buttonR1){
            intake_motor.setVelocity(100, vex::percentUnits::pct);
            intake_motor.spin(forward); 
        }
        else{
            intake_motor.stop(brake);
        }
        /*
        if(buttonL1 && !buttonL2){
            belt_motor.setVelocity(-100, vex::percentUnits::pct);
            belt_motor.spin(forward);   
        }
        else if(buttonL2 && !buttonL1){
            belt_motor.setVelocity(100, vex::percentUnits::pct);
            belt_motor.spin(forward); 
        }
        else{
            belt_motor.stop(brake);
        }
        */

        dual_stick_drive();
    }
}

int main() {
    Brain.Screen.print("Main start!");
    Brain.Screen.newLine();

    // Set up callbacks for autonomous and driver control periods.
    compete.autonomous(autonomous);
    compete.drivercontrol(usercontrol);

    primary_controller.ButtonR1.pressed(intake_toggle);
    primary_controller.ButtonL1.pressed(belt_toggle_on);
    primary_controller.ButtonL2.pressed(belt_toggle_off);

    thread beltThread = thread(belt_control);
    thread visionThread = thread(vision_sensor_thread);

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