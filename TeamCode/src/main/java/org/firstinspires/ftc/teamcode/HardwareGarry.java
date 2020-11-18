package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwareGarry
{
    /* Public OpMode members. */
    public DcMotor leftDrive   = null;
    public DcMotor leftDrive2   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  rightDrive2  = null;
    public DcMotor  mainArm     = null;
    public DcMotor sArm = null;
    public CRServo secondArm    = null;
    public Servo claw   = null;
    public Servo capClaw = null;
    public Servo capRotate    = null;
    public Servo leftGrab    = null;
    public Servo rightGrab    = null;


    public static final double NO_POWER     =  0 ;
    public static  final double POWER = 1;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareGarry(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDrive2  = hwMap.get(DcMotor.class, "left_drive2");
        rightDrive2 = hwMap.get(DcMotor.class, "right_drive2");
        mainArm    = hwMap.get(DcMotor.class, "mArm");
        sArm = hwMap.get(DcMotor.class, "sArm");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE if using AndyMark motors
        leftDrive2.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightDrive2.setDirection((DcMotor.Direction.FORWARD));

        sArm.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mainArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//makes the main arm lock in place when stopped
        sArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftDrive.setPower(NO_POWER);
        rightDrive.setPower(NO_POWER);
        leftDrive2.setPower(NO_POWER);
        rightDrive2.setPower(NO_POWER);
        mainArm.setPower(NO_POWER);
        sArm.setPower(NO_POWER);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mainArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        secondArm = hwMap.get(CRServo.class, "sArm");
        claw = hwMap.get(Servo.class, "claw");
        capClaw = hwMap.get(Servo.class, "cap");
        capRotate  = hwMap.get(Servo.class, "rotate");
        leftGrab  = hwMap.get(Servo.class, "left_filter");
        rightGrab  = hwMap.get(Servo.class, "right_filter");
        secondArm.setPower(0);
        claw.setPosition(.6);
        capClaw.setPosition(1);
        capRotate.setPosition(0.5);
        leftGrab.setPosition(0.8);
        rightGrab.setPosition(0.2);
    }
}

