package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop: AutoTest", group="Garry")
public class AutoTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareGarry r = new HardwareGarry();

    @Override
    public void runOpMode() {
        double turn;
        double armMax, safeArmMax = 0;
        double ForBack, LeftRight;
        double FL, FR, RL, RR;
        double maxMec1, maxMec2, maxMecSafe;
        double armInOut;
        double speed = 0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        r.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        r.capRotate.setPosition(1);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            r.leftDrive.setPower(0.21);
            r.rightDrive.setPower(-0.2);
            r.leftDrive2.setPower(-0.22);
            r.rightDrive2.setPower(0.22);
        }
    }
}