package org.firstinspires.ftc.teamcode.OldenGarbaege;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FTC on 11/7/2016.
 */

@TeleOp(name = "Test:  MotorTestControlOp",group = "Test")
@Disabled
public class MotorTestControlOp extends OpMode {
    DcMotor RLauncher;
    DcMotor LLauncher;
    DcMotor LiftMotor;
    boolean LauncherOn;
    boolean prevValue;
    @Override
    public void init() {
        RLauncher = hardwareMap.dcMotor.get("RLauncher");
        LLauncher = hardwareMap.dcMotor.get("LLauncher");
        LiftMotor = hardwareMap.dcMotor.get("LiftMotor");
    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        RLauncher.setPower(LauncherOn ? 1 : 0);
        LLauncher.setPower(LauncherOn ? 1 : 0);
        double changeValue = gamepad1.left_trigger + gamepad1.right_trigger * -1;
        LiftMotor.setPower(changeValue);

        if(gamepad1.left_bumper && gamepad1.left_bumper != prevValue){
            LauncherOn = !LauncherOn;
        }




        telemetry.addData("Status","Running!");
        prevValue = gamepad1.left_bumper;
    }
}
