package org.firstinspires.ftc.teamcode.HOME;
//**start setup of slider motor commands, temporarily use RBMotor port for this.
//**for Marcus/Oliver's home robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.LightBlinker;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.lang.Math;



@TeleOp
public class MecanumDriveHOME extends LinearOpMode {
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


        resetEncoders();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            //command for slider Up and Down
            if (gamepad1.a) {
                raiseSlider(0.4, 2700);
            } else {
               RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
               RBMotor.setPower(0.0);
            }
            telemetry.addData("SLIDER POSITION in Main = ", RBMotor.getCurrentPosition());
            telemetry.update();

            //reduce motor speed by factor 40%
            double motorPowerFactor = 0.4;

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


    //**TODO: Temporary set Slider motor as RBMotor until the Expansion Hub can be set up and label
    //as SliderMotor and change the label names in the raiseslider class and resetEncoders class below.
    void raiseSlider(double power, int ticks) {
        DcMotor RBMotor = hardwareMap.dcMotor.get("RBMotor");
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //**TODO: later add while loop and include touch sensor to prevent overextend up and down.
        //to avoid breaking string.
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setTargetPosition(ticks);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setPower(power);
        telemetry.addData("SLIDER POSITION in Method = ", RBMotor.getCurrentPosition());
        telemetry.update();
        while (RBMotor.isBusy()) {
        }
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setPower(0.0);
    }

    void resetEncoders() {
        DcMotor RBMotor = hardwareMap.dcMotor.get("RBMotor");

        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}









/*
    void turnTo(double degrees){
         private Orientation lastAngles = new Orientation();
         Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         System.out.println(orientation.firstAngle);
         double error = degrees - orientation.firstAngle;

         if (error > 180) {
             error -= 360;
         } else if (error < -180) {
             error += 360;
         }
         turn(error);
    }

    public void turn(double degrees){
        resetAngle();
        double error = degrees;
        while (opModeIsActive() && Math.abs(error) > 1) {
            double motorPower = (error < 0.1 ? -0.5 : 0.5);
            DcMotor.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        DcMotor.setAllPower(0);
    }

    public double getAngle() {
        // Get current orientation
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }
*/