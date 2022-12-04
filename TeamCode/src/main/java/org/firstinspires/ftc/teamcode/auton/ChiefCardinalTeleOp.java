package org.firstinspires.ftc.teamcode.auton;
//drive parameters below are for Marcus/Oliver's home robot */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.lang.Math;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.auton.AprilTagAutonomousInitDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;




//@Autonomous(name = "Autonomous for Scrimmage v1")
@TeleOp(name = "ChiefCardinal: TeleOp for Scrimmage v1")
public class ChiefCardinalTeleOp extends LinearOpMode {
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

    private ElapsedTime runtime = new ElapsedTime();



    //merging codes from AprilTagAutonomousInitDetection Class into this main program
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    public AprilTagDetection getTagOfInterest() {
        return tagOfInterest;
    }

    public void setTagOfInterest(AprilTagDetection tagOfInterest) {
        this.tagOfInterest = tagOfInterest;
    }






    @Override
    public void runOpMode() throws InterruptedException {
        //Declaring IMU-Gyro parameters
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);

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


//merging codes from AprilTagAutonomousInitDetection Class into this main program
//Start AprilTag identification while lining up at time.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    setTagOfInterest(tagOfInterest);
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            setTagOfInterest(tagOfInterest);
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            AprilTagDetection tagOfInterest = new AprilTagDetection();
            tagOfInterest.id = 0;
            setTagOfInterest(tagOfInterest);
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            telemetry.addData("tag",  "tagNumber = 1");
            telemetry.update();
        } else if (tagOfInterest.id == MIDDLE) {
            telemetry.addData("tag",  "tagNumber = 2");
            telemetry.update();
        } else {
            telemetry.addData("tag",  "tagNumber = 3");
            telemetry.update();
        }





//Game start.
        waitForStart();

        if (isStopRequested()) return;


//use to control robot during TeleOp game phase
        while (opModeIsActive()) {
            /* DEACTIVATE to test auto drive function--SLIDER
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


            //Set move for robot
            //direction=forward, backward, right, left, turnRight, turnLeft; power [-1,+1]; time in seconds
            if (gamepad1.dpad_up) {
                drive("forward", 0.6, .3);
            } else if (gamepad1.dpad_down) {
                drive("backward", 0.6, .3);
            } else if (gamepad1.dpad_right) {
                drive("right", 0.6, .3);
            } else if (gamepad1.dpad_left) {
                drive("left", 0.6, .3);
            } else if (gamepad1.b) {
                drive("turnRight", 0.7, 0.7);
            } else if (gamepad1.x) {
                drive("turnLeft", 0.7, 0.7);
            }


            //This is for manual control.
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            //reduce motor speed to 50% max
            double motorPowerFactor = 0.5;
            double powerChange;

            //Decrease power with left TRIGGER (lower of the top side button) press with the other gamepad stick.
            if ((Math.abs(gamepad1.left_stick_y) > 0.1 && gamepad1.left_trigger>0.1) || (Math.abs(gamepad1.left_stick_x) > 0.1 && gamepad1.left_trigger>0.1) ||  (Math.abs(gamepad1.right_stick_x) > 0.1 && gamepad1.left_trigger>0.1)) {
                powerChange = -0.2;
                //Increase power with left TRIGGER (lower of the top side button) press with the other gamepad stick.
            } else if ((Math.abs(gamepad1.left_stick_y) > 0.1 && gamepad1.left_bumper) || (Math.abs(gamepad1.left_stick_x) > 0.1 && gamepad1.left_bumper) || (Math.abs(gamepad1.right_stick_x) > 0.1 && gamepad1.left_bumper)) {
                powerChange = 0.2;
            } else{powerChange = 0;}


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (motorPowerFactor + powerChange) * (y + x + rx) / denominator;
            double backLeftPower = (motorPowerFactor + powerChange) * (y - x + rx) / denominator;
            double frontRightPower = (motorPowerFactor + powerChange) * (y - x - rx) / denominator;
            double backRightPower = (motorPowerFactor + powerChange) * (y + x - rx) / denominator;

            LFMotor.setPower(frontLeftPower);
            LBMotor.setPower(backLeftPower);
            RFMotor.setPower(frontRightPower);
            RBMotor.setPower(backRightPower);

        }
    }




    //Setup gamepad input to send to motors
    public void drive(String direction, double power, double time) {
        //direction=forward, backward, right, left, turnRight, turnLeft; power [-1,+1]; time in seconds
        double x = 0;
        double y = 0;
        double rx = 0;
        if (direction == "forward") {
            y = -1;
            x = 0;
            rx = 0;
        } else if (direction == "backward") {
            y = 1;
            x = 0;
            rx = 0;
        } else if (direction == "right") {
            y = 0;
            x = -1;
            rx = 0;
        } else if (direction == "left") {
            y = 0;
            x = 1;
            rx = 0;
        } else if (direction == "turnRight") {
            y = 0;
            x = 0;
            rx = -1;
        } else if (direction == "turnLeft") {
            y = 0;
            x = 0;
            rx = 1;
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
        LFMotor.setPower(power * frontLeftPower);
        LBMotor.setPower(power * backLeftPower);
        RFMotor.setPower(power * frontRightPower);
        RBMotor.setPower(power * backRightPower);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData(direction, "time %2.5f seconds elapsed", runtime.seconds());
            telemetry.update();
        }
    }


    //merging codes from AprilTagAutonomousInitDetection Class into this main program
    //Displaying AprilTag info on Drive Hub screen
    void tagToTelemetry (AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}