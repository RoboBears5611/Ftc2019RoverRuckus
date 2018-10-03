package org.firstinspires.ftc.teamcode.OldenGarbaege;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Created by FTC on 1/12/2017.
 */

@Autonomous(name = "Da Jig",group = "Autonomous")
@Disabled
public class DaJigOp extends LinearOpMode {
    private final static String LFMotorName = "LFMotor";
    private final static String RFMotorName = "RFMotor";
    private final static String LMMotorName = "LMMotor";
    private final static String RMMotorName = "RMMotor";
    private final static String FlapperName = "Flapper";
    private final static String RLauncherName = "RLauncher";
    private final static String LLauncherName = "LLauncher";

    private DcMotor LFMotor;
    private boolean LFMotorConnected;
    private DcMotor RFMotor;
    private boolean RFMotorConnected;
    private DcMotor LMMotor;
    private boolean LMMotorConnected;
    private DcMotor RMMotor;
    private boolean RMMotorConnected;

    private DcMotor flapper;
    private int flapperTarget;
    private int flapperMargin = 7;
    private DcMotor rLauncher;
    private DcMotor lLauncher;

    private int leftTarget;
    private int rightTarget;
    private final static int driveEncoderTicksPerRev = 120;
    private final static double flapperEncoderTicksPerRev = 120;
    private final static double totalFlapperChainRotationDegrees = 7350;

    private final static double drivePower = 0.5;
    private final static int flapperMaxSpeed = 500;
    private int driveMargin = 7;


    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize Stuff
        initializeMotors();
        flapper = hardwareMap.dcMotor.get(FlapperName);
        lLauncher = hardwareMap.dcMotor.get(LLauncherName);
        rLauncher = hardwareMap.dcMotor.get(RLauncherName);

        flapper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        flapper.setMaxSpeed(flapperMaxSpeed);
        LMMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Move!

        sendStatus("Starting...");
        bothRot(10);
        updateFor(5000,true);
        sendStatus("Finished updating section, running Thread.sleep(1500)");
        telemetry.update();
        Thread.sleep(1500);
        telemetry.addData("I hoth delayed","once");
        telemetry.update();
        sendStatus("DONE!");

    }
    private void sendStatus(String status){
        telemetry.addData("Status",status);
    }
    private void updateFor(int millis,boolean ceaseOnEnd) {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-startTime<=millis){
            update();
            sendStatus("updating!  updateFor_Millis:  "+millis+" passedTime:  "+(System.currentTimeMillis() - startTime));
        }
        if(ceaseOnEnd){
            cease();
        }
    }

    private void cease(){
        flapper.setPower(0);
        lLauncher.setPower(0);
        rLauncher.setPower(0);
        rightDrivePower(0);
        leftDrivePower(0);
        telemetry.addData("cease","CEASE O CAESAR");
        telemetry.update();
    }
    private void update(){
        if(Math.abs(flapper.getCurrentPosition()-flapperTarget)<=flapperMargin){
            if(gamepad2.x){
                flapperTarget = flapperTarget-(int)degToEt(totalFlapperChainRotationDegrees/2,flapperEncoderTicksPerRev);
            }
            flapper.setPower(0);
        }else{
            if(flapper.getCurrentPosition()<flapperTarget){
                flapper.setPower(1);
            }else if(flapper.getCurrentPosition()>flapperTarget){
                flapper.setPower(-1);
            }
        }
        telemetry.addData("LMMotor Pos",LMMotor.getCurrentPosition());
        telemetry.addData("LeftTarget",leftTarget);
        if(Math.abs(LMMotor.getCurrentPosition()-leftTarget)>=driveMargin){
            if(LMMotor.getCurrentPosition()>leftTarget) {
                rightDrivePower(-drivePower);
                leftDrivePower(-drivePower);
            }else{
                rightDrivePower(drivePower);
                leftDrivePower(drivePower);
            }
        }else{
            rightDrivePower(0);
            rightDrivePower(0);
        }
        try {
            waitOneFullHardwareCycle();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        telemetry.update();
    }

    private void rightDrivePower(double power) {
        if(RMMotorConnected)
            RMMotor.setPower(power);
        if(RFMotorConnected)
            RFMotor.setPower(power);
    }

    private void leftDrivePower(double power) {
        if(LMMotorConnected)
            LMMotor.setPower(power);
        if(LFMotorConnected)
            LFMotor.setPower(power);
    }

    private double degToEt(double deg, double encoderTicksPerRev){
        return deg/360*encoderTicksPerRev;
    }

    private void setLauncherPower(double power){
        lLauncher.setPower(power); rLauncher.setPower(power);
    }

    private void flapperDeg(double degrees){
        mFlapper((int) degToEt(degrees, flapperEncoderTicksPerRev));
    }
    private void flapperRot(double rotations){
        mFlapper((int) (rotations*flapperEncoderTicksPerRev));
    }
    private void mFlapper(int distance){
        flapperTarget += distance;
    }
    private void rightRot(double rotations) {
        right((int) (rotations * driveEncoderTicksPerRev));
    }
    private void leftRot(double rotations){
        left((int) (rotations * driveEncoderTicksPerRev));
    }
    private void bothRot(double rotations){
        both((int) (rotations * driveEncoderTicksPerRev));
    }

    private void rightDeg(double degrees){
        right((int) degToEt(degrees,driveEncoderTicksPerRev));
    }
    private void leftDeg(double degrees){
        left((int) degToEt(degrees,driveEncoderTicksPerRev));
    }
    private void bothDeg(double degrees){
        both((int) degToEt(degrees,driveEncoderTicksPerRev));
    }

    private void right(int distance){
        rightTarget += distance;
    }
    private void left(int distance){
        leftTarget += distance;
    }
    private void both(int distance){
        leftTarget += distance; rightTarget += distance;
    }
    public void initializeMotors() {
        Set<Map.Entry<String,DcMotor>> motors =  hardwareMap.dcMotor.entrySet();
        List<String> names = new LinkedList<>();
        for(Map.Entry<String,DcMotor> entry : motors){
            names.add(entry.getKey());
        }
        telemetry.addData("ConnectedMotors",names);
        if(names.contains(LFMotorName)){
            LFMotor = hardwareMap.dcMotor.get(LFMotorName);
            LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            LFMotorConnected = true;
        }else{
            LFMotorConnected = false;
        }
        if(names.contains(RFMotorName)){
            RFMotor = hardwareMap.dcMotor.get(RFMotorName);
            RFMotorConnected = true;
        }else{
            RFMotorConnected = false;
        }
        if(names.contains(LMMotorName)){
            LMMotor = hardwareMap.dcMotor.get(LMMotorName);
            LMMotorConnected = true;
        }else{
            LMMotorConnected = false;
        }
        if(names.contains(RMMotorName)){
            RMMotor = hardwareMap.dcMotor.get(RMMotorName);
            RMMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            RMMotorConnected = true;
        }else{
            RMMotorConnected = false;
        }
        telemetry.addData("LFMotorConnected (named "+LFMotorName+")",LFMotorConnected);
        telemetry.addData("RFMotorConnected (named "+RFMotorName+")",RFMotorConnected);
        telemetry.addData("LMMotorConnected (named "+LMMotorName+")",LMMotorConnected);
        telemetry.addData("RMMotorConnected (named "+RMMotorName+")",RMMotorConnected);
        telemetry.update();
    }
}
