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

@TeleOp(name = "GrabberOp",group = "drive")
public class GrabberOp extends OpMode {
    MechanumDriveBase mechanumDriveBase;
    DcMotor Grabber;
    DcMotor Lift;
    final static float GrabberSpeed  = 0.1f;


    @Override
    public void init(){

        mechanumDriveBase = new MechanumDriveBase(hardwareMap,telemetry);
        mechanumDriveBase.init();
        Grabber = hardwareMap.dcMotor.get("Grabber");
        Lift = hardwareMap.dcMotor.get("Lift");
        Grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public void start(){
        mechanumDriveBase.stopAllMotors();
    }

    @Override
    public void loop(){
        mechanumDriveBase.move(gamepad1.right_stick_x,gamepad1.right_stick_y,gamepad1.left_stick_x);
        Grabber.setPower(gamepad1.a?GrabberSpeed:0-(gamepad1.b?GrabberSpeed:0));
        int position = Grabber.getCurrentPosition();
        Lift.setPower((gamepad1.x?1:0)-(gamepad1.y?0.25:0));
        telemetry.addData("Encoder Position", position);

        Grabber.setTargetPosition(90);
    }
}