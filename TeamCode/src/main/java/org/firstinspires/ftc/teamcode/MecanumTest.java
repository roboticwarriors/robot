package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test: Mecanum", group="Test")
public class MecanumTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareGarry r = new HardwareGarry();     // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        double turn;
        double ForBack;
        double LeftRight;
        double FL;
        double FR;
        double RL;
        double RR;
        double maxMec1;
        double maxMec2;
        double maxMecSafe;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        r.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            //All Direction Mecanum Wheels
            /*
            {
                stickForwardBackward = -gamepad1.left_stick_y;
                stickLeftRight = gamepad1.left_stick_x;
                stickLeftRight = gamepad1.right_stick_x;

                FL = stickForwardBackward + stickLeftRight + stickLeftRight;
                FR = stickForwardBackward - stickLeftRight - stickLeftRight;
                BL = stickForwardBackward - stickLeftRight + stickLeftRight;
                BR = stickForwardBackward + stickLeftRight - stickLeftRight;
                maxMec1 = Math.max(Math.abs(FL), Math.abs(FR));
                maxMec2 = Math.max(Math.abs(BR), Math.abs(BL));
                maxMecSafe = Math.max(Math.abs(maxMec1), Math.abs(maxMec2));
                if (maxMecSafe > 1){
                    FL /= maxMecSafe;
                    FR /= maxMecSafe;
                    BL /= maxMecSafe;
                    BR /= maxMecSafe;
                }
                r.leftDrive.setPower(-FL);
                r.rightDrive.setPower(-FR);
                r.leftDrive2.setPower(BL);
                r.rightDrive2.setPower(-BR);
               }
             */

            {
                ForBack = -gamepad1.left_stick_y;
                LeftRight = gamepad1.left_stick_x;
                turn = gamepad1.right_stick_x;

                if (ForBack > 0.7) {
                    ForBack = 0.7;
                } else if (ForBack < -0.7) {
                    ForBack = -0.7;
                }
                if (LeftRight > 0.7) {
                    LeftRight = 0.7;
                } else if (LeftRight < -0.7) {
                    LeftRight = -0.7;
                }
                if (turn > 0.7) {
                    turn = 0.7;
                } else if (turn < -0.7) {
                    turn = -0.7;
                }
                if (ForBack > 0 || ForBack < 0) {
                    r.leftDrive.setPower(ForBack);
                    r.rightDrive.setPower(ForBack);
                    r.leftDrive2.setPower(ForBack);
                    r.rightDrive2.setPower(ForBack);
                } else if (LeftRight > 0 || LeftRight < 0) {
                    r.leftDrive.setPower(-LeftRight);
                    r.rightDrive.setPower(LeftRight);
                    r.leftDrive2.setPower(LeftRight);
                    r.rightDrive2.setPower(-LeftRight);
                } else if (turn > 0 || turn < 0) {
                    r.leftDrive.setPower(-turn);
                    r.rightDrive.setPower(turn);
                    r.leftDrive2.setPower(-turn);
                    r.rightDrive2.setPower(turn);
                } else {
                    r.leftDrive.setPower(0);
                    r.rightDrive.setPower(0);
                    r.leftDrive2.setPower(0);
                    r.rightDrive2.setPower(0);
                }
            }


            // Show joystick information as some other illustrative data
            telemetry.addLine("left joystick | ")
                    .addData("x", gamepad1.left_stick_x)
                    .addData("y", -gamepad1.left_stick_y);
            telemetry.addLine("right joystick | ")
                    .addData("x", gamepad1.right_stick_x)
                    .addData("y", gamepad1.right_stick_y);
            telemetry.update();

            // Pace this loop so jaw action is reasonable gear.
            sleep(50);
        }
    }
}
