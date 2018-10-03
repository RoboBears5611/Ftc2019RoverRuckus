package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Date;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Created by FTC on 12/13/2016.
 */
@TeleOp(group="TEST",name="DriveTester3000 (OnBot)")
public class DriveTester3000Op extends OpMode {
    private TimeBasedMechanumDriveBase timeBasedMechanumDriveBase;
    private float x;
    private float y;
    private float turn;
    private int time;
    private float prevX;
    private float prevY;
    private float prevTurn;
    private int prevTime;
    private boolean canUndo= false;
    private ChangableField CurrentlyChanging = ChangableField.none;




    @Override
    public void init() {
        timeBasedMechanumDriveBase = new TimeBasedMechanumDriveBase(hardwareMap,telemetry);

        timeBasedMechanumDriveBase.init();
    }

    @Override
    public void loop() {
        //Change current field set to be modified
        if(gamepad1.x){
            CurrentlyChanging = ChangableField.x;
        }else if(gamepad1.y){
            CurrentlyChanging = ChangableField.y;
        }else if(gamepad1.a){
            CurrentlyChanging = ChangableField.turn;
        }else if(gamepad1.b){
            CurrentlyChanging = ChangableField.time;
        }

        //Change the selected field by an amount specified by the right stick's y, using the right bumper as a speed boost for impatient people.
        if(CurrentlyChanging==ChangableField.time){
            time-=(int)gamepad1.right_stick_y*2*(gamepad1.right_bumper?10:1);
            if(time<0)time=0;
        }else{
            float amount = -gamepad1.right_stick_y/500*(gamepad1.right_bumper?5:1);
            switch(CurrentlyChanging){
                case x:
                    x = Range.clip(x+amount,-1,1);
                    break;
                case y:
                    y= Range.clip(y+amount,-1,1);
                    break;
                case turn:
                    turn= Range.clip(turn+amount,-1,1);
                    break;
                default:
                    break;
            }
        }

        //execute set settings
        if(gamepad1.dpad_up&&(!timeBasedMechanumDriveBase.isBusy())){
            move(x,y,turn,time);
            prevX = x;
            prevY = y;
            prevTurn = turn;
            prevTime = time;
            canUndo = true;
        }

        //undo
        if(gamepad1.dpad_down&&!timeBasedMechanumDriveBase.isBusy()&&canUndo){
            canUndo = false;
            move(-prevX,-prevY,-prevTurn,prevTime);
        }

          TimeBasedMechanumDriveBase.TimeBasedMechanumDriveMoveResult result = timeBasedMechanumDriveBase.loop();

        telemetry.addData("X:  ",x);
        telemetry.addData("Y:  ",y);
        telemetry.addData("Turn:  ",turn);
        telemetry.addData("Time:  ",time);
        telemetry.addData("CurrentlyChanging:  ",CurrentlyChanging);
        telemetry.addData("Time Until Completion:  ",result.timeRemainingForCurrentAction);
        telemetry.addData("Currently Executing:  ",result.isBusy);
        telemetry.addData("Can Undo",canUndo);
    }
    private void move(float x, float y, float turn, long time){
        timeBasedMechanumDriveBase.addInstruction(x,y,turn,time);
        if(!timeBasedMechanumDriveBase.executeInstructions()) telemetry.addLine("Failed to execute autonomous instructions");
    }
    private enum ChangableField {
        none,x,y,turn,time
    }
}
