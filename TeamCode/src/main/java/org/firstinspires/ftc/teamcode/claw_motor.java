/* Copyright (c) 2017 FIRST. All rights reserved.
 Redistribution and use in source and binary forms, with or without modification,
 are permitted (subject to the limitations in the disclaimer below) provided that
 the following conditions are met:
 Redistributions of source code must retain the above copyright notice, this list
 of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.
 Neither the name of FIRST nor the names of its contributors may be used to endorse or
 promote products derived from this software without specific prior written permission.
 NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
 This OpMode scans a single servo back and forward until Stop is pressed.
 The code is structured as a LinearOpMode
 INCREMENT sets how much to increase/decrease the servo position each cycle
 CYCLE_MS sets the update period.
 This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 connected servos are able to move freely before running this test.
 Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "claw" , group = "Servo")

public class claw_motor extends LinearOpMode {

    static final double increment = 0.01; // amount to slew servo each CYCLE_MS cycle
    static final int cycle = 50; // period of each cycle (The servo moves increment degrees every cycle_ms milliseconds)
    static final double max_position = 1.0; // Maximum rotational position
    static final double min_position = 0.0; // Minimum rotational position

    // Define class members
    Servo claw = hardwareMap.get(Servo.class, "claw"); // Define the servo, naming it "claw."
    double  position = (max_position - min_position) / 2; // Start at halfway position
    claw.setPosition(position);

    static public boolean rampUp = false;

    @Override
    public void runOpMode() {
        waitForStart();
        while(opModeIsActive()) {
            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp == true) {
                // Keep stepping up until we hit the max value.
                position += increment ;
                if (position >= max_position) { // if the servo reaches it's max position...
                    position = max_position;
                    rampUp = !rampUp; // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= increment ;
                if (position <= min_position) { // if the servo reaches it's min position...
                    position = min_position;
                    rampUp = !rampUp; // Switch ramp direction
                }
            }

            // Set the servo to the new position and pause;
            claw.setPosition(position);
            sleep(cycle);
            idle();
        }
    }
}
