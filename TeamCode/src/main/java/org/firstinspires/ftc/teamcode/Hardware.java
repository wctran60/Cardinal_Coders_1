package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware {
    //Hardware
    public DcMotor RFMotor = null;
    public DcMotor LFMotor = null;
    public DcMotor RBMotor = null;
    public DcMotor LBMotor = null;

    //Additional Variables
        HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    public Hardware(HardwareMap hwMap){

    }

    private void initialize(HardwareMap hwMap){
        hardwareMap = hwMap;

        //Connect Motors
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

        //Set Zero Power Behavior
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Motors to use no power
        RFMotor.setPower(0);
        LFMotor.setPower(0);
        RBMotor.setPower(0);
        LBMotor.setPower(0);

    }
}
