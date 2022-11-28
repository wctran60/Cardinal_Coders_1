package org.firstinspires.ftc.teamcode.auton;

import org.openftc.apriltag.AprilTagDetection;

public class ChiefCardinal {

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
