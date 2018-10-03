package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by FTC on 11/21/2016.
 */

@TeleOp(name = "MechanumDrive",group = "drive")
public class MechanumDriveOp extends OpMode {
    MechanumDriveBase mechanumDriveBase;


    @Override
    public void init(){

        mechanumDriveBase = new MechanumDriveBase(hardwareMap,telemetry);
        mechanumDriveBase.init();
    }


    @Override
    public void start(){
        mechanumDriveBase.stopAllMotors();
    }

    @Override
    public void loop(){
        mechanumDriveBase.move(gamepad1.right_stick_x,gamepad1.right_stick_y,gamepad1.left_stick_x);
    }
}