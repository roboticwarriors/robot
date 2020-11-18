/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoGarry_FoundationMover", group="Auto")
public class AutoGarry_FoundationMover extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareGarry r   = new HardwareGarry();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 1;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        r.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        //Step 1: Raise Main Arm
        r.capRotate.setPosition(1);
        r.mainArm.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addLine()
                .addData("Raise", "Raise 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        if((runtime.seconds() > 0.3)){
            r.mainArm.setPower(0);
        }

        //Step 2: Drive Sideways to Build Site
        r.leftDrive.setPower(-FORWARD_SPEED);
        r.leftDrive2.setPower(0.9);
        r.rightDrive.setPower(0.9);
        r.rightDrive2.setPower(-0.7);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addLine()
                    .addData("Raise", "Raise 1: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 2:  Drive forward for 3 seconds
        r.leftDrive.setPower(FORWARD_SPEED);
        r.leftDrive2.setPower(FORWARD_SPEED);
        r.rightDrive.setPower(FORWARD_SPEED);
        r.rightDrive2.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addLine()
                    .addData("Raise", "Raise 1: Complete");
            telemetry.addLine()
                .addData("Path", "Leg 1: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 3: Lower Grabber
        r.leftDrive.setPower(0);
        r.leftDrive2.setPower(0);
        r.rightDrive.setPower(0);
        r.rightDrive2.setPower(0);
        r.leftGrab.setPosition(0.2);
        r.rightGrab.setPosition(0.8);
        sleep(1000);
        runtime.reset();
        telemetry.addLine()
                .addData("Raise", "Raise 1: Complete");
        telemetry.addLine()
                .addData("Path", "Leg 1: Complete");
        telemetry.addLine()
                .addData("Path", "Leg 2: Complete");
        telemetry.addLine()
                .addData("Grab", "Grab 1: Complete");
        telemetry.update();

        //Step 4: Drive Backward
        r.leftDrive.setPower(-FORWARD_SPEED);
        r.leftDrive2.setPower(-FORWARD_SPEED);
        r.rightDrive.setPower(-FORWARD_SPEED);
        r.rightDrive2.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addLine()
                    .addData("Raise", "Raise 1: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 1: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 2: Complete");
            telemetry.addLine()
                    .addData("Grab", "Grab 1: Complete");
            telemetry.addLine()
                .addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 5: Let Go
        r.leftDrive.setPower(0);
        r.leftDrive2.setPower(0);
        r.rightDrive.setPower(0);
        r.rightDrive2.setPower(0);
        r.leftGrab.setPosition(0.8);
        r.rightGrab.setPosition(0.2);
        sleep(1000);
        runtime.reset();
        telemetry.addLine()
                .addData("Raise", "Raise 1: Complete");
        telemetry.addLine()
                .addData("Path", "Leg 1: Complete");
        telemetry.addLine()
                .addData("Path", "Leg 2: Complete");
        telemetry.addLine()
                .addData("Grab", "Grab 1: Complete");
        telemetry.addLine()
                .addData("Path", "Leg 3: Complete");
        telemetry.addLine()
                .addData("Grab", "Grab 2: Complete");
        telemetry.update();

        //Step 6: Drive Sideways
        r.leftDrive.setPower(1.0);
        r.leftDrive2.setPower(-0.9);
        r.rightDrive.setPower(-0.9);
        r.rightDrive2.setPower(0.7);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addLine()
                    .addData("Raise", "Raise 1: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 1: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 2: Complete");
            telemetry.addLine()
                    .addData("Grab", "Grab 1: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 3: Complete");
            telemetry.addLine()
                    .addData("Grab", "Grab 2: Complete");
            telemetry.addLine()
                .addData("Path", "Leg 4: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //Step 7: Lower Main Arm
        r.leftDrive.setPower(0);
        r.leftDrive2.setPower(0);
        r.rightDrive.setPower(0);
        r.rightDrive2.setPower(0);
        r.mainArm.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addLine()
                    .addData("Raise", "Raise 1: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 1: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 2: Complete");
            telemetry.addLine()
                    .addData("Grab", "Grab 1: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 3: Complete");
            telemetry.addLine()
                    .addData("Grab", "Grab 2: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 4: Complete");
            telemetry.addLine()
                .addData("Raise", "Raise 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        if((runtime.seconds() > 0.3)){
            r.mainArm.setPower(0);
        }
        sleep(500);

        //Step 8: Move to Bridge Tape
        r.leftDrive.setPower(1.0);
        r.leftDrive2.setPower(-0.9);
        r.rightDrive.setPower(-0.9);
        r.rightDrive2.setPower(0.7);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addLine()
                    .addData("Raise", "Raise 1: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 1: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 2: Complete");
            telemetry.addLine()
                    .addData("Grab", "Grab 1: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 3: Complete");
            telemetry.addLine()
                    .addData("Grab", "Grab 2: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 4: Complete");
            telemetry.addLine()
                    .addData("Raise", "Raise 2: Complete");
            telemetry.addLine()
                    .addData("Path", "Leg 5: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        if((runtime.seconds() > 1.0)) {
            r.leftDrive.setPower(0);
            r.leftDrive2.setPower(0);
            r.rightDrive.setPower(0);
            r.rightDrive2.setPower(0);
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

/*
        // Step 2:  Spin right for 1.3 seconds
        r.leftDrive.setPower(TURN_SPEED);
        r.leftDrive2.setPower(TURN_SPEED);
        r.rightDrive.setPower(-TURN_SPEED);
        r.rightDrive2.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backwards for 1 Second
        r.leftDrive.setPower(-FORWARD_SPEED);
        r.leftDrive2.setPower(-FORWARD_SPEED);
        r.rightDrive.setPower(-FORWARD_SPEED);
        r.rightDrive2.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        r.leftDrive.setPower(0);
        r.leftDrive2.setPower(0);
        r.rightDrive.setPower(0);
        r.rightDrive2.setPower(0);
        r.claw.setPosition(1.0);
        r.capClaw.setPosition(0);
        r.capRotate.setPosition(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
 */
    }
}
