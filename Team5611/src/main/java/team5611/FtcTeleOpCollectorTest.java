package team5611;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.annotation.Annotation;

import ftclib.FtcOpMode;
import ftclib.FtcServo;

@TeleOp(name="Collector Test", group="TeleOp")
public class FtcTeleOpCollectorTest extends FtcOpMode {
    CRServo leftServo;
    CRServo rightServo;

    @Override
    public void initRobot() {
        leftServo = hardwareMap.crservo.get(RobotInfo.LeftCollectorServoName);
        rightServo = hardwareMap.crservo.get(RobotInfo.RightCollectorServoName);
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
   }

    @Override
    public void runPeriodic(double elapsedTime) {
        double value = gamepad1.right_stick_y;
        leftServo.setPower(value);
        rightServo.setPower(value);
    }
}
