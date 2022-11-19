package org.firstinspires.ftc.teamcode.auton;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class MainAuton {

    public void mainAuton() {
        System.out.println("Starting Method mainAuton");
                try {
                    AprilTagAutonomousInitDetection at = new AprilTagAutonomousInitDetection();
                    System.out.println("Before Run runOpMode");
                    at.runOpMode();
                    System.out.println("Completed Run runOpMode");
                }
                catch(Exception e) {
                    System.out.println("Exception in MainOpenCv" + e);
                }
    }















}
