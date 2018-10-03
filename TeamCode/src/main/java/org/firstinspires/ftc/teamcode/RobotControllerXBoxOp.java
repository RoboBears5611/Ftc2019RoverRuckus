package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Created by FTC on 11/21/2016.
 */

@TeleOp(name = "Robot Controller (XBox)",group = "drive")
public class RobotControllerXBoxOp extends OpMode {


//    DcMotor ResettingMotor;
    RobotControllerBase robotControllerBase;
    final static double GrabberPositionAdjustmentAmount = 0.02;;

    @Override
    public void init(){
        robotControllerBase = new RobotControllerBase(telemetry,hardwareMap);
        robotControllerBase.init(new RobotControllerSettings());
    }


    @Override
    public void start(){
        robotControllerBase.start();
    }

    @Override
    public void loop(){
        RobotControllerDriveControls controls = new RobotControllerDriveControls();
        controls.moveX = -gamepad1.left_stick_x;
        controls.moveY = -gamepad1.left_stick_y;
        controls.moveTurn = gamepad1.right_stick_x;
        controls.moveForceFull = gamepad1.left_stick_button;
        controls.liftPower = gamepad1.right_trigger-gamepad1.left_trigger;
        // controls.grabberOpenCloseAdjustment = (gamepad1.dpad_right?GrabberPositionAdjustmentAmount:0)
                // - (gamepad1.dpad_left?GrabberPositionAdjustmentAmount:0);
        // controls.grabberToggleVariableControl = gamepad1.y;
        controls.grabberToggleOpenClosed = gamepad1.x;
        // controls.grabberVariableControlPosition = (gamepad1.dpad_up?0.5:0)-(gamepad1.dpad_down?0.5:0)+0.5;
        robotControllerBase.loop(controls);
    }

    
}
