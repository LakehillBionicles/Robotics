package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
// added a bunch of motors and servos that we will need for later - might cause error
public class LewaHardware extends LinearOpMode {
    /* Public OpMode members. */
    public DcMotor fsd = null;
    public DcMotor fpd = null;
    public DcMotor bpd = null;
    public DcMotor bsd = null;

    public DcMotor sliderVertical = null;
    public DcMotor intakeMotor = null;
    public DcMotor duckSpinner = null;
    public Servo flippyBox = null;
    public AnalogInput fsr = null;

    public DistanceSensor dist = null;
    public ColorSensor color = null;
    public RevBlinkinLedDriver lights = null;

    public static final double intakeSpeed = 0.7;
    public static final double sliderSpeed = 0.5;
    public static final double spinnerSpeed = 0.5;

    public static final double loadingFlippy = 1.0; //i don't think these names are right but change it if you want it
    public static final double turningFlippy = 0.0; //change maybe




    /*public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;*/

    private ElapsedTime runtime = new ElapsedTime();

    /*static final double COUNTS_PER_MOTOR_REV = 580;
    static final double DRIVE_GEAR_REDUCTION = (1/1.5);     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);*/

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
      sliderVertical = hwMap.get(DcMotor.class, "sliderVertical");
      intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
      duckSpinner = hwMap.get(DcMotor.class, "duckSpinner");
      flippyBox = hwMap.get(Servo.class, "flippyBox");
      dist = hwMap.get(DistanceSensor.class, "dist");
      color = hwMap.get(ColorSensor.class, "color");
      fsr = hwMap.get(AnalogInput.class, "fsr");
      lights = hwMap.get(RevBlinkinLedDriver.class, "lights");



        //set direction of rotations
        fsd.setDirection(DcMotorSimple.Direction.REVERSE);
        bsd.setDirection(DcMotorSimple.Direction.REVERSE);
        fpd.setDirection(DcMotorSimple.Direction.FORWARD);
        bpd.setDirection(DcMotorSimple.Direction.FORWARD);

        sliderVertical.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        duckSpinner.setDirection(DcMotorSimple.Direction.REVERSE);




        // Set all motors to zero power
        fpd.setPower(0);
        bpd.setPower(0);
        fsd.setPower(0);
        bsd.setPower(0);
        sliderVertical.setPower(0);
        intakeMotor.setPower(0);
        duckSpinner.setPower(0);



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ///////////////////////

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);      //changed to FLOAT from BRAKE on 1/25
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


}
