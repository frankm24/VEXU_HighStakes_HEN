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
#include "odometry.h"
#include "path_planning.h"

using namespace vex;


// Definitions
#define TURN_SPEED_RATIO 0.5
#define BELT_THROW_POSITION 638 // Farthest Number of Degrees from starting position needed to throw ring (No more than 1 revolution around BELT)
#define BELTRANGE 10 // Margin of error around throw position
#define BELTSPEED -100 // Speed of belt motor



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
motor left_motor_front = motor(PORT17, BLUE_GEAR, false);
motor left_motor_mid = motor(PORT13, BLUE_GEAR, false);
motor left_motor_back = motor(PORT10, BLUE_GEAR, false);
motor_group left_motor_group = motor_group(left_motor_front, left_motor_mid, left_motor_back);

motor right_motor_front = motor(PORT15, BLUE_GEAR, true);
motor right_motor_mid = motor(PORT8, BLUE_GEAR, true);
motor right_motor_back = motor(PORT9, BLUE_GEAR, true);
motor_group right_motor_group = motor_group(right_motor_front, right_motor_mid, right_motor_back);

motor intake_motor = motor(PORT18, GREEN_GEAR, false);
motor belt_motor = motor(PORT16, BLUE_GEAR, false);

digital_out Actuator = digital_out(Brain.ThreeWirePort.H);

// define your global instances of motors and other devices here
encoder left_encoder = encoder(Brain.ThreeWirePort.A);
encoder right_encoder = encoder(Brain.ThreeWirePort.C);
encoder offset_encoder = encoder(Brain.ThreeWirePort.E);

// define further software abstractions
Pose start_pose;
DriveConstants odometry_constants = {10, 5, 1};
ThreeWheelLocalizer localizer = ThreeWheelLocalizer(
    start_pose, odometry_constants, left_encoder, right_encoder, 
    offset_encoder);

// Global Variables
volatile bool belt_toggle_state = false;
volatile bool color_detected = true; // TODO: Set up control to vision sensor
volatile bool reverse_belt = false;


#define KP 0.01
#define LR_KP 0.05
#define TILEREVOLUTIONS 2.78
#define TIMEOUT_TIME 2000 // Time in milliseconds to wait for a command to complete
#define MINVOLTAGE 6
#define MAXVOLTAGE 8

// PID Control
double PIDControl(float target, float position){
    return (target - position) * KP;
}

void driveForward(int tiles){
    int t = (int)(tiles * TILEREVOLUTIONS);
    float avg_position = 0;

    left_motor_group.setPosition(0, degrees);
    right_motor_group.setPosition(0, degrees);

    uint32_t start_time = Brain.Timer.time();

    while(!(t+1 > avg_position && avg_position > t-1)){
        double left_position = left_motor_group.position(degrees);
        double right_position = right_motor_group.position(degrees);
        avg_position = (left_position + right_position) / 2;
        double drive = PIDControl(t, avg_position); // Calculate the drive value

        // Don't allow drive to go below minimum voltage (speed)
        drive = (drive < MINVOLTAGE && drive > 0) ? MINVOLTAGE : drive;
        drive = (drive > -MINVOLTAGE && drive < 0) ? -MINVOLTAGE : drive;

        // Don't allow drive to go above maximum voltage (speed)
        drive = (drive > MAXVOLTAGE) ? MAXVOLTAGE : drive;
        drive = (drive < -MAXVOLTAGE) ? -MAXVOLTAGE : drive;

        // P-Control between left and right motors
        double left_voltage_drive, right_voltage_drive = drive;
        if(left_position > right_position)
            left_voltage_drive += (right_position - left_position) * LR_KP; // If left is ahead, slow down left
        else
            right_voltage_drive += (left_position - right_position) * LR_KP; // If right is ahead, slow down right

        // Set the motor voltages
        left_motor_group.spin(forward, left_voltage_drive, voltageUnits::volt);
        right_motor_group.spin(forward, right_voltage_drive, voltageUnits::volt);
        
        uint32_t elapsed_time = Brain.Timer.time() - start_time;
        if(elapsed_time > TIMEOUT_TIME)
            break; // Break if the command takes too long

    }

    // Stop the motors
    left_motor_group.stop(brakeType::brake);
    right_motor_group.stop(brakeType::brake);

    return;
}


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
        /*
        if(color_detected && belt_position >= -BELTRANGE/3 && belt_position <= BELTRANGE){
            belt_motor.stop(vex::brakeType::brake);
            std::cout<<"Ejecting Ring!"<<std::endl;
            std::cout<<"Belt Position: "<<belt_position<<std::endl;

            wait(1, sec);
            while(belt_position >= 0 && belt_position <= BELTRANGE){
                if(reverse_belt)
                    belt_motor.setVelocity(-BELTSPEED, vex::percentUnits::pct);
                else
                    belt_motor.setVelocity(BELTSPEED, vex::percentUnits::pct);

                belt_motor.spin(forward);
                belt_position = abs((((int)belt_motor.position(vex::rotationUnits::deg)) % BELT_THROW_POSITION));
            }
        }
        */

       if(color_detected){
            wait(0.15, sec); // Wait until at peak
            std::cout<<"Ejecting Ring!"<<std::endl;
            
            belt_motor.stop(vex::brakeType::brake); // Briefly stop
            wait(0.45, sec);
            //belt_motor.setVelocity(BELTSPEED, vex::percentUnits::pct);
            belt_motor.spin(forward);
            wait(0.7, sec);


            /*
            while(belt_position >= 0 && belt_position <= BELTRANGE){
                if(reverse_belt)
                    belt_motor.setVelocity(-BELTSPEED, vex::percentUnits::pct);
                else
                    belt_motor.setVelocity(BELTSPEED, vex::percentUnits::pct);

                belt_motor.spin(forward);
                belt_position = abs((((int)belt_motor.position(vex::rotationUnits::deg)) % BELT_THROW_POSITION));
            }
            */
       }

        if(belt_toggle_state){
            if(reverse_belt)
                belt_motor.setVelocity(-BELTSPEED, vex::percentUnits::pct);
            else
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

//vision::code red_blue = vision::code(CUS_RED, CUS_BLUE);
//vision vSens = vision(PORT20, 50, red_blue);
vision vSens = vision(PORT20, 50, CUS_RED, CUS_BLUE);

// Enum to track the current vision state
enum VisionState {
    RED,
    BLUE,
    OFF
};

VisionState currentState = RED; // Start with red vision

void displayAutonomousStatus(Pose current_pose) {
     Brain.Screen.clearScreen();
            Brain.Screen.setPenColor(color::white);
            Brain.Screen.drawLine(240, 0, 240, 480); // Vertical center line
            Brain.Screen.drawLine(0, 120, 480, 120); // Horizontal center line

            int graph_x = 240 + static_cast<int>(current_pose.x);
            int graph_y = 120 - static_cast<int>(current_pose.y);
            
            Brain.Screen.setPenColor(color::red);
            Brain.Screen.drawPixel(graph_x, graph_y);
            Brain.Screen.drawLine(240,120,graph_x,graph_y);

            Brain.Screen.printAt(10, 200, "X: %.2f, Y: %.2f, Rotation: %f", current_pose.x, current_pose.y,current_pose.heading);

            int rotationX = static_cast<int>(30 * cos(current_pose.heading));
            int rotationY = static_cast<int>(30 * sin(current_pose.heading));

            Brain.Screen.setPenColor(color::blue);
            Brain.Screen.drawLine(graph_x,graph_y,graph_x + rotationX,graph_y - rotationY);
}

// Function to display the current status on the brain screen
void displayStatus() {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    std::cout<<belt_motor.temperature(temperatureUnits::celsius)<<std::endl;
    switch (currentState) {
        case RED:
            Brain.Screen.clearScreen(red);
            Brain.Screen.drawCircle(50, 50, 50, blue);
            primary_controller.Screen.clearScreen();
            primary_controller.Screen.print("Ejecting Red Rings\n");
            
            break;
        case BLUE:
            Brain.Screen.clearScreen(blue);
            primary_controller.Screen.print("Ejecting Blue Rings\n");
            Brain.Screen.drawCircle(50, 50, 50, red);
            break;
        case OFF:
            Brain.Screen.clearScreen(black);
            Brain.Screen.drawCircle(50, 50, 50, purple);
            primary_controller.Screen.print("Ejection Off\n");
            return; 
    }
}

// Vision Sensor Thread
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
            this_thread::sleep_for(250);
            //std::cout<<"Color Detected!"<<std::endl;
        }
        else{
            color_detected = false;
        }
        this_thread::sleep_for(50); 
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
    //PathFollowing::driveForward(10, localizer, odometry_constants, 
    //left_motor_group, right_motor_group);
    
    driveForward(1);
    wait(2, sec);
    //driveForward(-1);
    //wait(2, sec);
    driveForward(2);
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
    else {
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

        if(primary_controller.ButtonX.pressing()){
            Actuator.set(true);
            //std::cout<<"Actuator set to true"<<std::endl;
        }
        else if(primary_controller.ButtonB.pressing()){
            Actuator.set(false);
            //std::cout<<"Actuator set to false"<<std::endl;
        }

        if(primary_controller.ButtonY.pressing()){
            reverse_belt = true;
            //std::cout<<"Reverse Belt"<<std::endl;
        }
        else{
            reverse_belt = false;
        }
    

        
       
        

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