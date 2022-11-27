package org.firstinspires.ftc.teamcode;
//start setup of slider motor commands, temporarily use RBMotor port for this.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor LFMotor = hardwareMap.dcMotor.get("LFMotor");
        DcMotor LBMotor = hardwareMap.dcMotor.get("LBMotor");
        DcMotor RFMotor = hardwareMap.dcMotor.get("RFMotor");
        DcMotor RBMotor = hardwareMap.dcMotor.get("RBMotor");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            
            if (gamepad1.a) {
                raiseSlider(.4, 2000);
            } else {
                RBMotor.setPower(0);
            }


            //reduce motor speed by factor 40%
            double motorPowerFactor = 0.4;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = motorPowerFactor*(y + x + rx) / denominator;
            double backLeftPower = motorPowerFactor*(y - x - rx) / denominator;
            double frontRightPower = motorPowerFactor*(y - x + rx) / denominator;
            double backRightPower = motorPowerFactor*(y + x - rx) / denominator;

            LFMotor.setPower(frontLeftPower);
            LBMotor.setPower(backLeftPower);
            RFMotor.setPower(frontRightPower);
            RBMotor.setPower(backRightPower);

        }
    }





    //**TODO: Temporary set Slider motor as RBMotor until the Expansion Hub can be set up and label
    //as SliderMotor and change the label names in the raiseslider class and resetEncoders class below.
    void raiseSlider(double power, int ticks) {
        DcMotor RBMotor = hardwareMap.dcMotor.get("RBMotor");
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //**TODO: later add while loop and include touch sensor to prevent overextend up and down.
        //to avoid breaking string.
        RBMotor.setTargetPosition(ticks);
        RBMotor.setPower(power);
        while (RBMotor.isBusy()) {
        }
    }
    void resetEncoders(){
        DcMotor RBMotor = hardwareMap.dcMotor.get("RBMotor");

        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



}
