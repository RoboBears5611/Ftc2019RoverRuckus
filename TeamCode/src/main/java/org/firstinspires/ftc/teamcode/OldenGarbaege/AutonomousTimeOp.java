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

@Autonomous(name = "Autonomous (RUN THIS ONE)" ,group = "Autonomous")
@Disabled
public class AutonomousTimeOp extends LinearOpMode {
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

    private final static double drivePower = 0.3;
    private final static int flapperMaxSpeed = 500;
    private int driveMargin = 50;


    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize Stuff
        initializeMotors();
        flapper = hardwareMap.dcMotor.get(FlapperName);
        lLauncher = hardwareMap.dcMotor.get(LLauncherName);
        rLauncher = hardwareMap.dcMotor.get(RLauncherName);

        flapper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flapper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        flapper.setMaxSpeed(flapperMaxSpeed);
//        flapper.setMaxSpeed(1000);

        //Move!
        //        sendStatus("Starting...");
//        bothRot(10);
//        updateFor(30000,true);
//        sendStatus("Finished updating section, running Thread.sleep(1500)");
//        telemetry.update();
//        Thread.sleep(1500);
//        telemetry.addData("I hoth delayed","once");
//        telemetry.update();
//        sendStatus("DONE!");

        sendStatus("Driving");
        drive(1900);
        cease();
        sendStatus("Firing Up Launcher");
        setLauncherPower(0.7);
        Thread.sleep(250);
        sendStatus("Rotating Flapper");
        mFlapper(-7350*2);
        updateFor(5000,true);
        sendStatus("Driving Forward Again");
        drive(2000);
        cease();
        sendStatus("Finishing") ;
        setLauncherPower(0);
        sendStatus("DONE!");
    }

    private void drive(long time) throws InterruptedException {
        rightDrivePower(0.75);
        //Slowed Down to compensate for weird motor drag.
        leftDrivePower(0.625);
        Thread.sleep(time);
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
        leftDrivePower(0);
        rightDrivePower(0);
        telemetry.addData("cease","CEASE O CAESAR");
        telemetry.update();
    }
    private void update(){
        if(Math.abs(flapper.getCurrentPosition()-flapperTarget)<=flapperMargin){
            flapper.setPower(0);
        }else{
            if(flapper.getCurrentPosition()<flapperTarget){
                flapper.setPower(1);
            }else if(flapper.getCurrentPosition()>flapperTarget){
                flapper.setPower(-1);
            }
        }
//        telemetry.addData("LMMotor Pos",LMMotor.getCurrentPosition());
//        telemetry.addData("LeftTarget",leftTarget);
//        if(Math.abs(LMMotor.getCurrentPosition()-leftTarget)>=driveMargin){
//
//            if(LMMotor.getCurrentPosition()>leftTarget) {
//                rightDrivePower(LMMotor.getCurrentPosition()-leftTarget<driveMargin*3?-0.15:-drivePower);
//                leftDrivePower(LMMotor.getCurrentPosition()-leftTarget<driveMargin*3?-0.15:-drivePower);
//            }else{
//                rightDrivePower(LMMotor.getCurrentPosition()-leftTarget<driveMargin*3?0.15:drivePower);
//                leftDrivePower(LMMotor.getCurrentPosition()-leftTarget<driveMargin*3?0.15:drivePower);
//            }
//        }else{
//            rightDrivePower(0);
//            rightDrivePower(0);
//        }
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
        lLauncher.setPower(power); rLauncher.setPower(-power);
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
//    private void rightRot(double rotations) {
//        right((int) (rotations * driveEncoderTicksPerRev));
//    }
//    private void leftRot(double rotations){
//        left((int) (rotations * driveEncoderTicksPerRev));
//    }
//    private void bothRot(double rotations){
//        both((int) (rotations * driveEncoderTicksPerRev));
//    }
//
//    private void rightDeg(double degrees){
//        right((int) degToEt(degrees,driveEncoderTicksPerRev));
//    }
//    private void leftDeg(double degrees){
//        left((int) degToEt(degrees,driveEncoderTicksPerRev));
//    }
//    private void bothDeg(double degrees){
//        both((int) degToEt(degrees,driveEncoderTicksPerRev));
//    }
//
//    private void right(int distance){
//        rightTarget += distance;
//    }
//    private void left(int distance){
//        leftTarget += distance;
//    }
//    private void both(int distance){
//        leftTarget += distance; rightTarget += distance;
//    }
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
            LMMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            LMMotorConnected = true;
        }else{
            LMMotorConnected = false;
        }
        if(names.contains(RMMotorName)){
            RMMotor = hardwareMap.dcMotor.get(RMMotorName);
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
