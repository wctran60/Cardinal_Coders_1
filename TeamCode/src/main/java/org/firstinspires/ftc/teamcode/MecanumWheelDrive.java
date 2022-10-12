package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class MecanumWheelDrive extends OpMode {

    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;




    public void moveDriveTrain(){
        double vertical = 0;
        double horizontal = 0;
        double pivot = 0;
        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        RFMotor.setPower(pivot + (-vertical + horizontal));
        LFMotor.setPower(pivot + (-vertical - horizontal));
        RBMotor.setPower(pivot + (-vertical - horizontal));
        LBMotor.setPower(pivot + (-vertical + horizontal));
    }
    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class,"RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class,"LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class,"RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class,"LBMotor");

    }
    @Override
    public void init_loop() {

    }
    @Override
    public void loop() {

    }
}

