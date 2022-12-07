package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.lang.Math;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Slide testing")
public class SlideMotor extends LinearOpMode {
    DcMotor sliderMotor;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        DcMotor SliderMotor = hardwareMap.dcMotor.get("SliderMotor");
        SliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            //Need Expansion Hub to run this for slider and grasper
            //To test without Expansion Hub, plug slider motor and encoder wires to RBMotor port on Control Hub
            //        rename above hardwareMap from "SliderMotor" to "RBMotor"
            //press "Y" on gamepad to raise to max position
            //      "B" on gamepad to raise to mid position
            //      "A" on gamepad to go to lowest position
            if (gamepad1.y) {
                raiseSlider(0.4, 2700);
            }else if (gamepad1.b){
                raiseSlider(0.4, 1300);
            } else if (gamepad1.a){
                raiseSlider(0.4, 0);
            }else {
                SliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                SliderMotor.setPower(0.0);
            }
            //testing what appear on screen in Driver Hub: info inside this main loop or inside raiseSlider class.
            telemetry.addData("SLIDER POSITION in main program = ", SliderMotor.getCurrentPosition());
            telemetry.update();

            idle();
        }

    }



    public void raiseSlider(double power, int position) {
        DcMotor SliderMotor = hardwareMap.dcMotor.get("SliderMotor");
        SliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //**TODO: later add while loop and include touch sensor to prevent overextend up and down.
        //        to avoid breaking string.
        SliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SliderMotor.setTargetPosition(position);
        SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderMotor.setPower(power);
        telemetry.addData("SLIDER POSITION in Method = ", SliderMotor.getCurrentPosition());
        telemetry.update();
        while (SliderMotor.isBusy()) {
        }
        //SliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //SliderMotor.setPower(0.0);
    }



}


