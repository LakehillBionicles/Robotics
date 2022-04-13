package funGame;


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


public class botHardware extends LinearOpMode {

    public DcMotor fpd, fsd, bpd, bsd, arm = null;

    public Servo hand = null;

    public DistanceSensor dist = null;
    public ColorSensor color = null;
    public RevBlinkinLedDriver lights = null;

    public static final double intakeSpeed = 1.0;
    public static final double armSpeed = 1.0;

    public static final double handOpen = 0.0;
    public static final double handClose = 1.0;

    private ElapsedTime runtime = new ElapsedTime();

    //below are calculations for drive train (counts per motor shouldn't change but other will)


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime time = new ElapsedTime();

    /* Constructor */
    public botHardware() {
    }

    @Override
    public void runOpMode() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //define and init everything
        fpd = hwMap.get(DcMotor.class, "fpd");
        fsd = hwMap.get(DcMotor.class, "fsd");
        bpd = hwMap.get(DcMotor.class, "bpd");
        bsd = hwMap.get(DcMotor.class, "bsd");
        arm = hwMap.get(DcMotor.class, "arm");

        hand = hwMap.get(Servo.class, "hand");

        dist = hwMap.get(DistanceSensor.class, "dist");
        color = hwMap.get(ColorSensor.class, "color");
        lights = hwMap.get(RevBlinkinLedDriver.class, "lights");

        //set directions of rotation
        fsd.setDirection(DcMotorSimple.Direction.REVERSE);
        bsd.setDirection(DcMotorSimple.Direction.REVERSE);
        fpd.setDirection(DcMotorSimple.Direction.FORWARD);
        bpd.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

        //motors to zero
        fpd.setPower(0.0);
        fsd.setPower(0.0);
        bpd.setPower(0.0);
        bsd.setPower(0.0);
        arm.setPower(0.0);

        //set this way orginally - not sure why
        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //float is neutral and brake stops the robot from running
        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




    }

}
