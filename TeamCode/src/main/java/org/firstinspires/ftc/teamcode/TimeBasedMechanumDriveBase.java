package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

public class TimeBasedMechanumDriveBase{
    private MechanumDriveBase mechanumDriveBase;
    private boolean actionIsExecuting;
    private Runnable runOnComplete;
    private Telemetry telemetry;
    public ArrayList<TimeBasedMechanumDriveMovement> instructions;
    private TimeBasedMechanumDriveMovement currentMovement;


    public TimeBasedMechanumDriveBase(HardwareMap hardwareMap,Telemetry telemetry){
        this.telemetry = telemetry;
        mechanumDriveBase = new MechanumDriveBase(hardwareMap,telemetry);
        instructions = new ArrayList<>();
    }
    public void init(){
        mechanumDriveBase.init();
    }
    public boolean isBusy(){
        return actionIsExecuting;
    }

    public void addInstruction(float x,float y, float turn, long timeMillis){
        instructions.add(new TimeBasedMechanumDriveMovement(x,y,turn,timeMillis));
    }

    public TimeBasedMechanumDriveMoveResult loop(){
        if(actionIsExecuting&&currentMovement.stopTime>=new Date().getTime()){
            currentMovement = null;
            if(!executeNextInstruction()) {
                actionIsExecuting = false;
                return new TimeBasedMechanumDriveMoveResult(false);
            }
        }
        return new TimeBasedMechanumDriveMoveResult(true, currentMovement.stopTime-new Date().getTime(),instructions.size());
    }

    public boolean executeInstructions(){
        if(instructions.isEmpty()||actionIsExecuting){
            return false;
        }
        executeNextInstruction();
        actionIsExecuting = true;
        return true;

    }

    private boolean executeNextInstruction(){
        if(currentMovement==null||instructions.isEmpty()) {
            return false;
        }
        currentMovement = instructions.remove(0);
        currentMovement.stopTime = new Date().getTime() + currentMovement.time;
        mechanumDriveBase.move(currentMovement.x, currentMovement.y, currentMovement.turn);
        return true;

    }


    public class TimeBasedMechanumDriveMoveResult{
        public boolean isBusy;
        public long timeRemainingForCurrentAction = 0;
//        public long timeRemainingForAllActions = 0;
        public int actionsRemaining = 0;
        public TimeBasedMechanumDriveMoveResult(boolean isBusy){
            this.isBusy = isBusy;
        }
        public TimeBasedMechanumDriveMoveResult(boolean isBusy, long timeRemainingForCurrentAction, int actionsRemaining){
            this.isBusy = isBusy;
            this.timeRemainingForCurrentAction = timeRemainingForCurrentAction;
//            this.timeRemainingForAllActions = timeRemainingForAllActions;
            this.actionsRemaining = actionsRemaining;
        }
    }
    public class TimeBasedMechanumDriveMovement{
        public float x;
        public float y;
        public float turn;
        public long time;
        public long stopTime = 0;
        public TimeBasedMechanumDriveMovement(float x,float y,float turn, long timeMillis){
            this.x = x;
            this.y = y;
            this.turn = turn;
            this.time = timeMillis;
        }
    }
}
