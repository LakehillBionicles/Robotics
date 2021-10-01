package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled

public class LewaHardware extends LinearOpMode {
    /* Public OpMode members. */
    public DcMotor fsd = null;
    public DcMotor fpd = null;
    public DcMotor bpd = null;
    public DcMotor bsd = null;

    /*public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;*/

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 580;
    static final double DRIVE_GEAR_REDUCTION = (10/26.0);     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double COUNTS_PER_MOTOR_REV_BE = 8192;
    static final double DRIVE_GEAR_REDUCTION_BE = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES_BE = 3.5;     // For figuring circumference
    public static final double COUNTS_PER_INCH_BE = (COUNTS_PER_MOTOR_REV_BE * DRIVE_GEAR_REDUCTION_BE) /
            (WHEEL_DIAMETER_INCHES_BE * 3.1415);

    public final double ninetyTurn = (2 * Math.PI * 7.25);  //what is the radius of our wheels

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime time = new ElapsedTime();

    /* Constructor */
    public LewaHardware() {
    }


    @Override
    public void runOpMode() {
    }

    /* Initialize standard Hardware interfaces */
    //@Override
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        fpd = hwMap.get(DcMotor.class, "fpd");
        fsd = hwMap.get(DcMotor.class, "fsd");
        bpd = hwMap.get(DcMotor.class, "bpd");
        bsd = hwMap.get(DcMotor.class, "bsd");

        //set direction of rotations
        fsd.setDirection(DcMotorSimple.Direction.FORWARD);
        bsd.setDirection(DcMotorSimple.Direction.FORWARD);
        fpd.setDirection(DcMotorSimple.Direction.REVERSE);
        bpd.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to zero power
        fpd.setPower(0);
        bpd.setPower(0);
        fsd.setPower(0);
        bsd.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ///////////////////////

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   //new, could mess soomrthing up, but should be fine
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


}
