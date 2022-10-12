package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class MotorTest extends OpMode {
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;


    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class,"RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class,"LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class,"RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class,"LBMotor");

        //Set Up Motor Direction
        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);

        //Set Motor Mode
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void init_loop() {

    }
    @Override
    public void loop() {
        RBMotor.setPower(0.25);
        RFMotor.setPower(0.25);
        LBMotor.setPower(-0.25);
        LFMotor.setPower(-0.25);
    }
}
