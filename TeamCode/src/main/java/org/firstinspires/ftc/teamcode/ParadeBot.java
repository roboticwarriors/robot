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

@TeleOp(name="Teleop: Parade", group="Garry")
public class ParadeBot extends LinearOpMode {

    double ForBack = 0, LeftRight = 0 ;
    double FL = 0, FR = 0, RL = 0, RR = 0, turn = 0;
    double maxMec1,maxMec2, maxMecSafe;
    double speed = 0;

    /* Declare OpMode members. */
    HardwareParade r = new HardwareParade();     // Use a Pushbot's hardware


    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        r.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            ForBack = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            FL = ForBack + LeftRight + turn;
            FR = ForBack - LeftRight - turn;
            RL = ForBack - LeftRight + turn;
            RR = ForBack + LeftRight - turn;
            maxMec1 = Math.max(Math.abs(FL), Math.abs(FR));
            maxMec2 = Math.max(Math.abs(RR), Math.abs(RL));
            maxMecSafe = Math.max(Math.abs(maxMec1), Math.abs(maxMec2));
            if (maxMecSafe > 1) {
                FL /= maxMecSafe;
                FR /= maxMecSafe;
                RL /= maxMecSafe;
                RR /= maxMecSafe;
            }
            if (gamepad1.left_bumper){
                speed = 1;
            }
            else if (gamepad1.right_bumper){
                speed = 0;
            }
            if (speed==0){
                r.leftDrive.setPower(FL);
                r.rightDrive.setPower(FR);
//                r.leftDrive2.setPower(BL);
//                r.rightDrive2.setPower(BR);
            }
            else if (speed == 1 && turn == 0) {
                r.leftDrive.setPower(FL / 4);
                r.rightDrive.setPower(FR / 4);
//                r.leftDrive2.setPower(BL / 3);
//                r.rightDrive2.setPower(BR / 3);
            }
            else if (speed == 1 && turn > 0 || turn < 0){
                r.leftDrive.setPower(FL / 2);
                r.rightDrive.setPower(FR / 2);
//                r.leftDrive2.setPower(BL / 1.5);
//                r.rightDrive2.setPower(BR / 1.5);
            }

        }
    }
}
