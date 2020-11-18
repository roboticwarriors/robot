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

@TeleOp(name="Teleop: Parade2", group="Garry")
public class ParadeBot2 extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareParade r = new HardwareParade();     // Use a Pushbot's hardware

    private boolean slow = false;
    private static double MAX_DELTA = 0.01;
    private double previousFL;
    private double previousFR;

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
            try {
                Thread.sleep(50);
            } catch (InterruptedException ex){
                throw new RuntimeException(ex);
            }
            double stickForwardBackward = gamepad1.left_stick_y;
            double stickLeftRight = gamepad1.right_stick_x;

            // -1 to positive 1
            double FL = stickForwardBackward + stickLeftRight;
            double FR = stickForwardBackward - stickLeftRight;
            // start magic
            double maxMecSafe = Math.max(Math.abs(FL), Math.abs(FR));
            if (maxMecSafe > 1) {
                FL /= maxMecSafe;
                FR /= maxMecSafe;

            }
            telemetry.addLine("FL: " + FL + " FR: " + FR);
            telemetry.addLine("maxmecsafe: " + maxMecSafe);
            // end magic;
            // Max speed change
            // BREAKS GOING BACKWARDS

            double FLDelta = FL - previousFL;
            double FRDelta = FR - previousFR;
            double tempDelta = Math.min(MAX_DELTA, Math.abs(FLDelta));
            double tempDelta2 =  Math.min(MAX_DELTA, Math.abs(FRDelta));
           if (FLDelta < 0) {
               FLDelta = -tempDelta;
           } else {
               FLDelta = tempDelta;
           }
            if (FRDelta < 0) {
                FRDelta = -tempDelta2;
            } else {
                FRDelta = tempDelta2;
            }
           FL = previousFL + FLDelta;
           FR = previousFR + FRDelta;
            this.previousFL = FL;
            this.previousFR = FR;

            if (gamepad1.left_bumper){
                slow = true;
            }
            else if (gamepad1.right_bumper){
                slow = false;
            }
            if (!slow){
                r.leftDrive.setPower(FL);
                r.rightDrive.setPower(FR);
            }
            else if (stickLeftRight == 0) {
                r.leftDrive.setPower(FL / 4);
                r.rightDrive.setPower(FR / 4);
            }
            else if (stickLeftRight > 0 || stickLeftRight < 0){
                r.leftDrive.setPower(FL / 2);
                r.rightDrive.setPower(FR / 2);
            }
            telemetry.update();
        }
    }
}
