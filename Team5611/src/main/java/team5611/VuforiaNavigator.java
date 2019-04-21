package team5611;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import ftclib.FtcAndroidTone;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcSimpleDriveBase;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VuforiaNavigator {
    private TrcSimpleDriveBase driveBase;
    private TrcPidDrive pidDrive;
    private VuforiaVision vuforiaVision;

    TrcPidController encoderYPidCtrl = null; //Takes over default encoder logic, I guess for when you have really complicated drive systems.  We don't.  We don't actually use these
    TrcPidController gyroPidCtrl = null;
    FtcAndroidTone androidTone; //Makes beeping sounds to tell you when bad stuff is happening.


    float xTarget;
    float yTarget;
    float headingTarget;
    float currentHeadingTarget; //Varies from headingTarget when the robot is driving straight towards the target.

    TrcStateMachine<NavState> navStateMachine;
    TrcEvent event;
    public TrcEvent doneEvent;

    boolean targetVisible;
    Orientation lastOrientation = new Orientation();
    float[] lastPosition = new float[3];
    float lastX;
    float lastY;
    float lastHeading;

    boolean SKIP_TO_DONE = false; //NOTE:  For testing only

    enum NavState{
        TurnToTarget,
        DriveToTarget,
        TurnToHeading,
        Done
    }

    VuforiaNavigator(String InstanceName, TrcSimpleDriveBase driveBase, VuforiaVision vuforiaVision){
        this.driveBase = driveBase;
        this.vuforiaVision = vuforiaVision;

        navStateMachine = new TrcStateMachine<>(InstanceName+".navStateMachine");
        navStateMachine.setState(NavState.Done);
        event = new TrcEvent(InstanceName+".event");
        androidTone = new FtcAndroidTone("AndroidTone");
        //
        // Initialize PID drive.
        //
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD),
                RobotInfo.ENCODER_Y_TOLERANCE, this::driveYPidDistance);
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.GYRO_KP, RobotInfo.GYRO_KI, RobotInfo.GYRO_KD),
                RobotInfo.GYRO_TOLERANCE, this::drivePidHeading);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        gyroPidCtrl.setOutputRange(-RobotInfo.TURN_POWER_LIMIT, RobotInfo.TURN_POWER_LIMIT);


        pidDrive = new TrcPidDrive("pidDrive", driveBase,null, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        taskMgr.createTask(InstanceName+".targetTask",this::targetTask);
    }

    /**
 * @param x Target X in Inches
     *  @param y Target Y in Inches
 * @param heading Target heading in degrees
     */
    public void setTarget(float x, float y, float heading) {
        setTarget(x, y, heading, null);
    }

    /**
     *  @param x Target X in Inches
     *  @param y Target Y in Inches
     *  @param heading Target heading in degrees
     */
    public void setTarget(float x, float y, float heading, TrcEvent doneEvent){
        this.xTarget = x * (float)TrcUtil.MM_PER_INCH;
        this.yTarget = y * (float)TrcUtil.MM_PER_INCH;
        this.headingTarget = heading;
        currentHeadingTarget = headingTarget;
        this.doneEvent = doneEvent;

        if(distanceCloseEnough()){
            if(headingCloseEnough()){
                setNavState(NavState.Done);
            }else{
                setNavState(NavState.TurnToHeading);
            }
        }else{
            setNavState(NavState.TurnToTarget);
        }
    }

    private boolean distanceCloseEnough(){ return getLinearDistanceFromTarget() <= RobotInfo.VuforiaDrive_CloseEnoughMM;}
    private boolean headingCloseEnough(){ return Math.abs(currentHeadingTarget - lastHeading)< RobotInfo.VuforiaDrive_CloseEnoughDegrees;}

    public void headingTest_setTarget(float x, float y) {
        headingTest_setTarget(x, y, null);
    }

    public void headingTest_setTarget(float x, float y, TrcEvent doneEvent){
        xTarget = x;
        yTarget = y;
        SKIP_TO_DONE = true;
        this.doneEvent = doneEvent;
        setNavState(NavState.TurnToTarget);
    }

    public void distanceTest_increaseDistance(float distance) {
        distanceTest_increaseDistance(distance, null);
    }

    public void distanceTest_increaseDistance(float distance, TrcEvent doneEvent) {
        headingTarget = lastHeading;
        xTarget = lastX+distance*(float)Math.cos(lastHeading);
        yTarget = lastY+distance*(float)Math.sin(lastHeading);
        SKIP_TO_DONE = true;
        this.doneEvent = doneEvent;
        setNavState(NavState.DriveToTarget);
    }

    public void headingTest_setHeading(float heading) {
        headingTest_setHeading(heading, null);
    }

    public void headingTest_setHeading(float heading, TrcEvent doneEvent){
        headingTarget = heading;
        SKIP_TO_DONE = true;
        this.doneEvent = doneEvent;
        setNavState(NavState.TurnToHeading);
    }


    private double driveYPidDistance(){
        float dist = getLinearDistanceFromTarget();
        if(navStateMachine.getState()==NavState.DriveToTarget){
            if(dist>RobotInfo.VuforiaDrive_CloseEnoughMM) {
                return dist;
            }else{
                event.set(true);
                return 0;
            }
        }else{
            return 0; //Just stop trying to move.
        }
    }

    private float getLinearDistanceFromTarget(){
        return (float)Math.hypot(xTarget - lastX,yTarget - lastY);
    }

    private float drivePidHeading(){
        NavState state = navStateMachine.getState();
        if(state!=NavState.Done) {  //Even when in "DriveToTarget" mode, we want to hold the heading steady.
            if (!headingCloseEnough()) {
                return lastHeading;
            }else{
                if(state==NavState.TurnToHeading||state==NavState.TurnToTarget) event.set(true);
                return currentHeadingTarget;
            }
        }else{
            return currentHeadingTarget; //When we're close enough, stop trying to get closer.
        }
    }

    private void setNavState(NavState newNavState) {
        NavState nextState = NavState.Done;
        event.clear();

        if(SKIP_TO_DONE){  //For testing purposes, allows each state to be testing individually
            newNavState = NavState.Done;
        }

        //The "event", which moves things to the next state, is actually signalled by the "getPidHeading" and "getYPidDistance" functions

        switch(newNavState){
            case TurnToTarget:
                currentHeadingTarget = (float) Math.atan2(yTarget-lastY,xTarget-lastX);  //Get relative angle from robot to target
                nextState = NavState.DriveToTarget;
                break;
            case DriveToTarget:
                //Yes, I realize that the "y" target will be set to 0.  This is desired.
                    // When the state is changed to "DriveToTarget",
                    // ...The "getYPidDistance" function will start returning the bot's current distance from 0, such that
                    // ...with the target set to 0, we start driving forward.
                    // Outside of "DriveToTarget" mode, that function will always return 0, aka no driving forward or back.
                nextState = NavState.TurnToHeading;
                break;
            case TurnToHeading:
                currentHeadingTarget = headingTarget; //also uncomment these
                nextState = NavState.Done;
                break;
            case Done:
                break;
        }

        navStateMachine.setState(newNavState);
        if(newNavState!=NavState.Done) {
            pidDrive.setTarget(0,currentHeadingTarget,true,event); //In all states but "Done" this exactly is needed.
            navStateMachine.waitForSingleEvent(event, nextState);
        }else{
            event.set(true);
        }
    }

    private void targetTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode){
        OpenGLMatrix location = vuforiaVision.getRobotLocation();
        if(location!=null){
            lastOrientation = vuforiaVision.getLocationOrientation(location); //XYZ
            lastPosition = vuforiaVision.getLocationTranslation(location).getData();
        }
        lastX = lastPosition[0];
        lastY = lastPosition[1];
        lastHeading = lastOrientation.thirdAngle;

        if(navStateMachine.isReady()){
            switch(navStateMachine.getState()){
                case TurnToTarget:
                    break;
                case DriveToTarget:
                    break;
                case TurnToHeading:
                    break;
                case Done:
                    break;
            }
        }

    }
}
