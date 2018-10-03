package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Created by FTC on 11/21/2016.
 */

@TeleOp(name = "Robot Controller (Two Controllers)",group = "drive")
public class RobotControllerTwoControllersOp extends OpMode {


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
        robotControllerBase.stopAllMotors();
    }

    @Override
    public void loop(){
        RobotControllerDriveControls controls = new RobotControllerDriveControls();
        controls.moveX = -gamepad1.right_stick_x;
        controls.moveY = -gamepad1.right_stick_y;
        controls.moveTurn = gamepad1.left_stick_x;
        controls.moveForceFull = gamepad1.left_bumper;
        controls.liftPower = gamepad2.right_trigger-gamepad2.left_trigger;
        controls.grabberOpenCloseAdjustment = (gamepad2.dpad_right?GrabberPositionAdjustmentAmount:0)
                - (gamepad2.dpad_left?GrabberPositionAdjustmentAmount:0);
        controls.grabberToggleVariableControl = gamepad2.b;
        controls.grabberToggleOpenClosed = gamepad2.right_bumper;
        controls.grabberVariableControlPosition = (gamepad2.dpad_up?0.5:0)-(gamepad2.dpad_down?0.5:0)+0.5;
        robotControllerBase.loop(controls);
    }


}