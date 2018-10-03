package org.firstinspires.ftc.teamcode.OldenGarbaege;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


/**
 * Created by FTC on 11/21/2016.
 */

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