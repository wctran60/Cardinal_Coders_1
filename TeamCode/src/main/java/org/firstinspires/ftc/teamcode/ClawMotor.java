//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
///*
// This OpMode scans a single servo back and forward until Stop is pressed.
// The code is structured as a LinearOpMode
// INCREMENT sets how much to increase/decrease the servo position each cycle
// CYCLE_MS sets the update period.
// This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
// NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
// connected servos are able to move freely before running this test.
// Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//
//@TeleOp
//
//public class ClawMotor extends LinearOpMode {
//
//    static final double increment = 0.01;     // amount to slew servo each CYCLE_MS cycle
//    static final int cycle_ms = 50;     // period of each cycle (The servo moves increment degrees every cycle_ms milliseconds)
//    static final double max_position =  1.0;     // Maximum rotational position
//    static final double min_position =  0.0;     // Minimum rotational position
//
//    // Define class members
//    DcMotor RBMotor = hardwareMap.dcMotor.get("RBMotor");
//    Servo servo = hardwareMap.get("claw"); // Define the servo, naming it "claw."
//    double  position = (max_position - min_position) / 2; // Start at halfway position
//    public boolean rampUp = false;
//
//    @Override
//    public void runOpMode() {
//        waitForStart();
//
//        while(opModeIsActive()){
//
//            // slew the servo, according to the rampUp (direction) variable.
//            if (rampUp) {
//                // Keep stepping up until we hit the max value.
//                position += increment ;
//                if (position >= max_position) {
//                    position = max_position;
//                    rampUp = !rampUp;   // Switch ramp direction
//                }
//            }
//            else {
//                // Keep stepping down until we hit the min value.
//                position -= increment ;
//                if (position <= min_position) {
//                    position = min_position;
//                    rampUp = !rampUp;  // Switch ramp direction
//                }
//            }
//
//            // Display the current value
//            telemetry.addData("Servo Position", "%5.2f", position);
//            telemetry.addData(">", "Press Stop to end test." );
//            telemetry.update();
//
//            // Set the servo to the new position and pause;
//            servo.setPosition(position);
//            sleep(cycle_ms);
//            idle();
//        }
//
//        // Signal done;
//        telemetry.addData(">", "Done");
//        telemetry.update();
//    }
//}
//
//
