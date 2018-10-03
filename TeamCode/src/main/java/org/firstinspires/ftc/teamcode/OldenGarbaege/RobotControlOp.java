package org.firstinspires.ftc.teamcode.OldenGarbaege;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "RobotControlOp",group = "Robot Control")
@Disabled
public class RobotControlOp extends OpMode {
    private TacoDriveOp tacoDriver;
    private final static String lLiftServoName = "LLiftServo";
    private final static String rLiftServoName = "RLiftServo";
    private final static String liftMotorName = "LiftMotor";
    private Servo lLiftServo;
    private Servo rLiftServo;
    private final static boolean invertRLiftServo = false;
    private final static boolean invertLLiftServo = true;
    //Remember, this is at max 1/whatever you set, and it's added or subtracted from the position of the servo (a 0-1 value) every repetition of the program!! This means changes will be applied quickly!!
    private final static double liftServoChangeDivider = 120;
    private DcMotor liftMotor;
    private final static String lLauncherName = "LLauncher";
    private final static String rLauncherName = "RLauncher";
    private boolean launchersPowered = false;
    private DcMotor lLauncher;
    private DcMotor rLauncher;
    //Collector
    private final static String rubberFingersName = "RubberFingers";
    private DcMotor rubberFingers;
    private boolean rubberFingersPowered = false;
    //Feeder (don't question our naming conventions)
    private final static String flapperName = "Flapper";
    private DcMotor flapper;
    private double flapperTarget = 0;
    private final static double flapperMargin = 7;

//    private final static double launcherEncoderTicksPerRev = 120;
    private final static double flapperEncoderTicksPerRev = 120;

    //Launch Stuff//
    private LaunchState launchState = LaunchState.ready;
    //how far the flapper has to turn to get the chain it's attached to to rotate a complete rotation
    private final static double totalFlapperChainRotationDegrees = 7350;
    //how far the flapper has to turn to go from "ready" to "loaded" positions
//    private final static double readyToLoadedDegrees = 540;
    //how far the flapper has to turn to go from "loaded" to "fired" positions (a.k.a. how much push the ball needs to launch)
//    private final static double loadedToFiredDegrees = 20;
    private boolean _prevGamepad1A;
    private boolean _prevGamepad1B;
    private boolean _prevGamepad2B;
    private final static double targetLauncherSpeed = 0.7;
    private final static double launcherSpeedAdjustability = 0.4;
//    private final static double launcherAccelerationDelay = 500;
//    private final static double launcherDecelerationDelay = 250;
//    private final static long launcherFireDelay = 100;
//    private long accelOrDecelStartTime;
    //This is overstuffed with sendStatus()'s because I couldn't get the thing to work earlier.  That's been resolved, but the status updates remain, because, why cut it out?  It could come in handy.
    @Override
    public void init() {
        sendStatus("Initializing Drive");
        tacoDriver = new TacoDriveOp(hardwareMap,telemetry);
        sendStatus("Created TacoDrive Instance, Initializing");
        tacoDriver.init();
        sendStatus("Initialized Drive");
        lLiftServo = hardwareMap.servo.get(lLiftServoName);
        rLiftServo = hardwareMap.servo.get(rLiftServoName);
        sendStatus("Initialized drive motors and servos. initializing Lift Motor");
        liftMotor = hardwareMap.dcMotor.get(liftMotorName);
        sendStatus("Lift Motor Initialized");
        rubberFingers = hardwareMap.dcMotor.get(rubberFingersName);
        flapper = hardwareMap.dcMotor.get(flapperName);
        flapper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sendStatus("Rubber Fingers and Flapper Initialized");
        lLauncher = hardwareMap.dcMotor.get(lLauncherName);
//        lLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rLauncher = hardwareMap.dcMotor.get(rLauncherName);
//        rLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        sendStatus("Launchers Initialized");
    }

    private void sendStatus(String statusText){
        telemetry.addData("Status",statusText);
        telemetry.update();
    }

    private void sendMessage(String messageText){
        telemetry.addData("Message",messageText);
        telemetry.update();
    }

    public void start(){
        flapper.setPower(0);
        flapper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tacoDriver.start();
        lLiftServo.setPosition(invertLLiftServo?1:0);
        rLiftServo.setPosition(invertRLiftServo?1:0);
        telemetry.update();
        liftMotor.setPower(0);
        rubberFingers.setPower(0);
        _prevGamepad1A = gamepad1.a;
        _prevGamepad1B = gamepad1.b;
        flapper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //This is actually oddly slow...
//        flapper.setMaxSpeed(500);
    }

    private double dpsToEtps(double dps, double encoderTicksPerRev){
        return dps/360*encoderTicksPerRev;
    }

    private double rpmToEtps(double rpm, double encoderTicksPerRev){
        return (rpm*encoderTicksPerRev)/60;
    }

    private double rotToEt(double rot, double encoderTicksPerRev){
        return rot * encoderTicksPerRev;
    }

    private double millisToMinutes(long millis){
        return millis/60000;
    }

    private double degToEt(double deg, double encoderTicksPerRev){
        return deg/360*encoderTicksPerRev;
    }

//    private void setLauncherSpeed(double rpm){
//        int maxEtps = (int) rpmToEtps(rpm, launcherEncoderTicksPerRev);
//        telemetry.addData("MaxLauncherETPS",maxEtps);
//        lLauncher.setMaxSpeed(maxEtps);
//        rLauncher.setMaxSpeed(maxEtps);
//        //if rpm==0, turn off motors, else max them out so they (try to) reach max speed (which we just set)
//        double motorSpeed = rpm==0?0:1;
//        lLauncher.setPower(motorSpeed);
//        rLauncher.setPower(motorSpeed);
//    }

    private void setLauncherSpeed(double power){
        rLauncher.setPower(power);
        lLauncher.setPower(power);
    }

    @Override
    public void loop() {
        //PassedTime is calculated right off the bat for accuracy
//        performLauncherStuff();
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
        if(Math.abs(flapper.getCurrentPosition()-flapperTarget)<=flapperMargin*2){
//            flapper.setMaxSpeed(250);
        }else{
//            flapper.setMaxSpeed(500);
        }
        //flapper.setPower((gamepad2.x?1:0)+(gamepad2.y?-1:0));

        calcLauncherSpeeds();
        tacoDriver.loop(gamepad1,gamepad2);
        liftMotor.setPower(-gamepad2.right_stick_y);
        setLiftServos();
        setRubberFingersPowered();
        telemetry.addData("Status", "Running");
        telemetry.addData("LaunchState",launchState);
        telemetry.addData("Left Launcher Position",lLauncher.getCurrentPosition());
        telemetry.addData("Right Launcher Position",rLauncher.getCurrentPosition());
        telemetry.addData("Flapper Position",flapper.getCurrentPosition());
        telemetry.addData("Target Flapper Position",flapper.getTargetPosition());
        telemetry.addData("rLiftServo",rLiftServo.getPosition());
        telemetry.addData("lLiftServo",lLiftServo.getPosition());
        telemetry.addData("lLauncherPower",lLauncher.getPower());
        telemetry.addData("rLauncherPower",rLauncher.getPower());
        _prevGamepad1A = gamepad1.a;
        _prevGamepad1B = gamepad1.b;
        _prevGamepad2B = gamepad2.b;
    }

    private void calcLauncherSpeeds() {
        if(_prevGamepad2B!=gamepad2.b&&gamepad2.b){
            launchersPowered = !launchersPowered;
        }
        double adjustment = launcherSpeedAdjustability/(gamepad2.dpad_right||gamepad2.dpad_left?2:1);
        double launcherPower =  Range.clip(targetLauncherSpeed + (gamepad2.dpad_up?adjustment:0) - (gamepad2.dpad_down?-adjustment:0),0,1);
        setLauncherSpeed(launchersPowered?launcherPower:0);
    }

//    private void performLauncherStuff() {
//        // BEGIN USER LAUNCH STATE MANIPULATIONS //
//        //Set load and fire to true if their corresponding buttons (a and b) were just pushed.
//        boolean load = (_prevGamepad1A != gamepad1.a) && gamepad1.a;
//        boolean fire = (_prevGamepad1B != gamepad1.b) && gamepad1.b;
//
//        //If we're ready to load and Mr. User says "load", we load.
//        if(launchState == LaunchState.ready && load){
//            launchState = LaunchState.loading;
//            flapper.setTargetPosition(flapper.getCurrentPosition()+(int) degToEt(readyToLoadedDegrees, flapperEncoderTicksPerRev));
//            return;
//        }
//
//        //If we're ready to load but Mr. User says "fire", complain that we simply can't fire without first being loaded!!
//        if(launchState == LaunchState.ready && fire){
//            sendMessage("Trying to fire when not loaded!  Please load before firing.");
//        }
//
//
//        //If we're loaded but Mr. User says "load", there's obviously a need to retry loading.  So, set status to "revertingLoad" to un-load
//        if(launchState == LaunchState.loaded && load){
//            launchState = LaunchState.revertingLoad;
//            flapper.setTargetPosition(flapper.getCurrentPosition()- (int)degToEt(totalFlapperChainRotationDegrees - readyToLoadedDegrees, flapperEncoderTicksPerRev));
//            return;
//        }
//
//        //If we're loaded and Mr. User says "fire", get the launcher started on the "firing" sequence.
//        if(launchState == LaunchState.loaded && fire){
//            launchState = LaunchState.accelerating;
//            setLauncherSpeed(targetLauncherSpeed);
//            accelOrDecelStartTime = System.currentTimeMillis();
//            return;
//        }
//
//        //If status isn't ready or loaded, the controls are locked.  If the users thinks otherwise, inform he is in the wrong.
//        if((launchState != LaunchState.ready && launchState != LaunchState.loaded) && (gamepad1.a || gamepad1.b)){
//          sendMessage("Launcher Controls are locked because the current LaunchState is not ready or loaded.");
//        }
//        // END USER LAUNCHER STATE MANIPULATIONS //
//
//        // BEGIN LOADING LOGIC //
//
//        //The state is loading, but we're no longer loading, so set the status to loaded.
//        if(launchState == LaunchState.loading && !flapper.isBusy()){
//            launchState = LaunchState.loaded;
//        }
//
//        //If our status is "revertingLoad" but we aren't unloading anymore, set state to ready.
//        if(launchState == LaunchState.revertingLoad && !flapper.isBusy()){
//            launchState = LaunchState.ready;
//        }
//
//        // END LOADING LOGIC //
//
//        // BEGIN FIRING LOGIC //
//
//        //If we're accelerating, and the motors have reached desired speed, get shootin'!
//        if(launchState == LaunchState.accelerating&&System.currentTimeMillis() - accelOrDecelStartTime>=launcherAccelerationDelay){
//            launchState = LaunchState.firing;
//            flapper.setTargetPosition(flapper.getCurrentPosition()+(int)loadedToFiredDegrees);
//            return;
//        }
//
//
//        //If we're firing, but the flapper as already flapped into launch position, begin launcher deceleration (After delaying a short time for the ball to actually depart).
//        if(launchState == LaunchState.firing&&!flapper.isBusy()){
//            try{
//                wait(launcherFireDelay);
//            }catch(InterruptedException e){
//                Thread.currentThread().interrupt();
//            }
//            launchState = LaunchState.decelerating;
//            setLauncherSpeed(0);
//            accelOrDecelStartTime = System.currentTimeMillis();
//            return;
//        }
//
//        //If we're deceleration,and we've decelerated to under the threshold, start resetting flapper for another shot.
//        if(launchState == LaunchState.decelerating&&System.currentTimeMillis() - accelOrDecelStartTime>=launcherDecelerationDelay){
//            launchState = LaunchState.resetting;
//            flapper.setTargetPosition(flapper.getCurrentPosition() + (int)(totalFlapperChainRotationDegrees - readyToLoadedDegrees - loadedToFiredDegrees));
//            return;
//        }
//
//        //If we're resetting, but the flapper has already finished moving into the reset position, set the launch state to ready
//        if(launchState == LaunchState.resetting&&flapper.isBusy()){
//            launchState = LaunchState.ready;
//        }
//        // END FIRING LOGIC //
//    }

    private void setRubberFingersPowered() {
        //This could be simplified into a single logic statement, but simplification would make it miles from simple. This is far easily to understand.
        if(gamepad2.left_bumper){
            rubberFingersPowered = false;
        }
        if(gamepad2.right_bumper){
            rubberFingersPowered = true;
        }
        if(gamepad2.y){
            rubberFingers.setPower(-1);
        }else{
            rubberFingers.setPower(rubberFingersPowered ? 1 : 0);
        }
    }

    private void setLiftServos() {
        double change = (gamepad2.right_trigger - gamepad2.left_trigger)/ liftServoChangeDivider;
        double newLLiftServoPos = Range.clip(invertLLiftServo ? lLiftServo.getPosition() + change : lLiftServo.getPosition() - change,0,1);
        double newRLiftServoPos = Range.clip(invertRLiftServo ? rLiftServo.getPosition() + change  : rLiftServo.getPosition() - change,0,1);
        lLiftServo.setPosition(newLLiftServoPos);
        rLiftServo.setPosition(newRLiftServoPos);
    }
}

enum LaunchState{
    //loading and loaded refer to getting the ball ready to be shot.  Loading, logically, means the Flapper is getting the ball into the "loaded" position.
     loading,
    //Ball is locked and loaded and ready to be fired
    loaded,
    //Going back to ready Position from loaded position.  This may be needed if the loading process failed and needs to be retried.
    revertingLoad,
    //getting motor up to speed
    accelerating,
    //moving ball into launcher
    firing,
    //slowing motor down to allow flapper safe passage for reset.
    decelerating,
    //returning flapper to start position for next ball/ launch.
    resetting,
    //Default.  In start position and ready to shoot.
    ready
}