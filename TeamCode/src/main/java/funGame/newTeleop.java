package funGame;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp
@Disabled

//TESTING GITHUB!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

public class newTeleop extends LinearOpMode {
    botHardware robot = new botHardware();

    double armSpeed = 1;

    @Override
    public void runOpMode (){
        runOpMode();
        waitForStart();




    }
    public void tankDrive(){
        if (gamepad1.left_stick_y>0.1 || gamepad1.left_stick_y<-0.1) {
            robot.fpd.setPower(gamepad1.left_stick_y);
            robot.bpd.setPower(gamepad1.left_stick_y);
        } else {
            robot.fpd.setPower(0);
            robot.bpd.setPower(0);
        }
        if(gamepad1.right_stick_y>0.1 || gamepad1.right_stick_y<-0.1) {
            robot.fsd.setPower(gamepad1.right_stick_y);
            robot.bsd.setPower(gamepad1.right_stick_y);
        } else {
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);
        }
    }

    public void moveArm() {
        if (gamepad1.right_bumper) {
            robot.arm.setPower(armSpeed);
        } else if (gamepad1.left_bumper) {
            robot.arm.setPower(armSpeed);
        } else {
            robot.arm.setPower(0);
        }
    }

    public void moveHand(){
        if(gamepad1.right_trigger>0.1) {
            robot.hand.setPosition(robot.hand.getPosition() + 0.1);
        } else if(gamepad1.left_trigger>0.1) {
            robot.hand.setPosition(robot.hand.getPosition() - 0.1);
        }

    }

    }
