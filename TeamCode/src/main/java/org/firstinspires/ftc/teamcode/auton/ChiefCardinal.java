package org.firstinspires.ftc.teamcode.auton;
/**DEACTIVATING slider command, AprilTag command,
 //slider motor = temporarily assign as RBMotor.
 //drive parameters below are for Marcus/Oliver's home robot */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.lang.Math;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.openftc.apriltag.AprilTagDetection;


//@Autonomous(name = "Auto v1 Scrimmage")
@TeleOp(name = "ChiefCardinal:Test Autonomous")
public class ChiefCardinal extends LinearOpMode {
    DcMotor LFMotor;
    DcMotor RFMotor;
    DcMotor LBMotor;
    DcMotor RBMotor;

    //define IMU-Gyro
    BNO055IMU imu;
    Orientation angles;
    double imuHeadingAngleInRadian;
    double imuHeadingAngle;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    private double degrees;

    private ElapsedTime     runtime = new ElapsedTime();

    int detectionId;

    @Override
    public void runOpMode() throws InterruptedException {
        //Declaring IMU-Gyro parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor LFMotor = hardwareMap.dcMotor.get("LFMotor");
        DcMotor LBMotor = hardwareMap.dcMotor.get("LBMotor");
        DcMotor RFMotor = hardwareMap.dcMotor.get("RFMotor");
        DcMotor RBMotor = hardwareMap.dcMotor.get("RBMotor");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        //run AprilTag and returns detectionID as 0, 1, 2, or 3
        mainAuton();
        telemetry.addData("AprilTag,detectionId = ", detectionId);
        telemetry.update();



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
/* DEACTIVATE to test auto drive function
            //need Expansion Hub to run this for slider and grasper
            //to test without expansion Hub, plug slider motor to RBMotor port on Control Hub
            //press "A" on right side of gamepad to run
            if (gamepad1.a) {
                raiseSlider(0.4, 2700);
            } else {
                RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                RBMotor.setPower(0.0);
            }
            telemetry.addData("SLIDER POSITION in main program = ", RBMotor.getCurrentPosition());
            telemetry.update();  */



            //**TEST commands to program for autonomous, right keypad on gamepad to run.
            //direction=forward, backward, right, left, turnRight, turnLeft; power [-1,+1]; time in seconds
            if (gamepad1.y) {
                drive("forward", 0.3, .3);
            }else if (gamepad1.a){
                drive("backward", 0.3, .3);
            }else if (gamepad1.b){
                drive("right", 0.3, .3);
            }else if (gamepad1.x){
                drive("left", 0.3, .3);
            }else if (gamepad1.right_bumper){
                drive("turnRight", 0.3, .3);
            }else if (gamepad1.left_bumper){
                drive("turnLeft", 0.3, .3);
            }



            //This is for manual control.  to be erase when running Autonomous in competition
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            //reduce motor speed to 30% max
            double motorPowerFactor = 0.3;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = motorPowerFactor * (y + x + rx) / denominator;
            double backLeftPower = motorPowerFactor * (y - x + rx) / denominator;
            double frontRightPower = motorPowerFactor * (y - x - rx) / denominator;
            double backRightPower = motorPowerFactor * (y + x - rx) / denominator;

            LFMotor.setPower(frontLeftPower);
            LBMotor.setPower(backLeftPower);
            RFMotor.setPower(frontRightPower);
            RBMotor.setPower(backRightPower);

        }
    }




    public void drive(String direction, double power, double time){
                //direction=forward, backward, right, left, turnRight, turnLeft; power [-1,+1]; time in seconds
        double x=0;
        double y=0;
        double rx=0;
        if (direction=="forward"){
                y=-1;
                x=0;
                rx=0;
            }else if (direction=="backward"){
                y=1;
                x=0;
                rx=0;
            }else if (direction=="right"){
                y=0;
                x=-1;
                rx=0;
            }else if (direction=="left"){
                y=0;
                x=1;
                rx=0;
            }else if (direction=="turnRight"){
                y=0;
                x=0;
                rx=-1;
            }else if (direction=="turnLeft"){
                y=0;
                x=0;
                rx=1;
        }
        DcMotor LFMotor = hardwareMap.dcMotor.get("LFMotor");
        DcMotor LBMotor = hardwareMap.dcMotor.get("LBMotor");
        DcMotor RFMotor = hardwareMap.dcMotor.get("RFMotor");
        DcMotor RBMotor = hardwareMap.dcMotor.get("RBMotor");
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        LFMotor.setPower(power*frontLeftPower);
        LBMotor.setPower(power*backLeftPower);
        RFMotor.setPower(power*frontRightPower);
        RBMotor.setPower(power*backRightPower);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData(direction, "time %2.5f seconds elapsed", runtime.seconds());
            telemetry.update();
        }
    }















/*DEACTIVATE to test auto drive function
    //**TODO: Temporary set Slider motor as RBMotor (#1 motor slot) until the Expansion Hub can be set up and label
    //as SliderMotor and change the label names in the raiseslider class and resetEncoders class below.
    public void raiseSlider(double power, int position) {
        DcMotor RBMotor = hardwareMap.dcMotor.get("RBMotor");
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //**TODO: later add while loop and include touch sensor to prevent overextend up and down.
        //to avoid breaking string.
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setTargetPosition(position);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setPower(power);
        telemetry.addData("SLIDER POSITION in Method = ", RBMotor.getCurrentPosition());
        telemetry.update();
        while (RBMotor.isBusy()) {
        }
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setPower(0.0);
    }
    */


/*DEACTIVATE to test auto drive function
    //TODO: need to change RBMotor to motor used for Slider in Expansion Hub
    public void resetEncoders() {
        DcMotor RBMotor = hardwareMap.dcMotor.get("RBMotor");
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
*/


    //??run AprilTag and returns detectionID as 1, 2, or 3
    public void mainAuton() {
        System.out.println("Starting Method mainAuton");
        try {
            AprilTagAutonomousInitDetection at = new AprilTagAutonomousInitDetection();
            System.out.println("Before Run runOpMode");
            at.runOpMode();
            AprilTagDetection detection = at.getTagOfInterest();
            int detectionId = detection.id;
            System.out.println("TagId =" + detectionId);
            System.out.println("Completed Run runOpMode with tag = " + detectionId);
        }
        catch(Exception e) {
            System.out.println("Exception in MainOpenCv" + e);
        }
    }



}
