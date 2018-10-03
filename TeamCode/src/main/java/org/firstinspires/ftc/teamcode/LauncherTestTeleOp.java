package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Goerge on 10/2/2018.
 */

@TeleOp(name = "Launcher Test", group="Test")
public class LauncherTestTeleOp extends OpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    private static final String LeftMotorName = "LeftLauncher";
    private static final String RightMotorName = "RightLauncher";
    Controls controls = new DefaultControls(gamepad1,gamepad2);
    @Override
    public void init() {
     leftMotor = hardwareMap.dcMotor.get(LeftMotorName);
     rightMotor = hardwareMap.dcMotor.get(RightMotorName);

     leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
     rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

     leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
     rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        double power = controls.getTestMotorPower();
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}
