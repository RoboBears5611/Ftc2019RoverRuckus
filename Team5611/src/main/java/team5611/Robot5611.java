/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team5611;

import android.widget.TextView;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import ftclib.FtcAndroidTone;
import ftclib.FtcBNO055Imu;
import ftclib.FtcDcMotor;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcRobotBattery;
import ftclib.FtcServo;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcGyro;
import trclib.TrcMecanumDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcSimpleDriveBase;
import trclib.TrcWarpSpace;

public class Robot5611 implements FtcMenu.MenuButtons
{
    static final boolean USE_SPEECH = true;
    static final boolean USE_VUFORIA = true;

    private static final String moduleName = "Robot5611";
    //
    // Global objects.
    //
    FtcOpMode opMode;
    HalDashboard dashboard;
    TrcDbgTrace tracer;
    FtcRobotBattery battery = null;
    VuforiaVision vuforiaVision = null;
    FtcAndroidTone androidTone;


    FtcDcMotor leftWheel = null;
    FtcDcMotor rightWheel = null;
    TrcSimpleDriveBase driveBase = null;

    TrcPidController encoderYPidCtrl = null;
    TrcPidController gyroPidCtrl = null;
    TrcPidDrive pidDrive = null;


    //Peripherals (extendoArm)
    FtcDcMotor ExtendoRotator;
    FtcDcMotor Extendor;

    DcMotor RoboLift;
    FtcDcMotor Collector;

    TrcRobot.RunMode RunMode;
    public Robot5611(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();
        tracer = FtcOpMode.getGlobalTracer();
        dashboard.setTextView(
                (TextView)((FtcRobotControllerActivity)opMode.hardwareMap.appContext).findViewById(R.id.textOpMode));
        RunMode=runMode;
        if(opMode.hardwareMap.voltageSensor.entrySet().isEmpty()){
            dashboard.displayPrintf(0,"No Battery Sensor Found");
        }else{
            battery = new FtcRobotBattery();
        }
        androidTone = new FtcAndroidTone("AndroidTone");

        if (USE_VUFORIA)
        {
            dashboard.displayPrintf(1,"Initializing Vuforia");
            int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            tracer.tracePrintf("CameraMonititorViewId:  %s",cameraViewId);
            dashboard.displayPrintf(3,"CamId %s",cameraViewId);
            vuforiaVision = new VuforiaVision(this, cameraViewId, VuforiaLocalizer.CameraDirection.FRONT,new OpenGLMatrix());//cameraViewId);
            dashboard.displayPrintf(1,"Initialized Vuforia");
        }

        dashboard.displayPrintf(2,"Loading up Peripherals.");

        leftWheel = new FtcDcMotor(RobotInfo.LeftMotorName);
        rightWheel = new FtcDcMotor(RobotInfo.RightMotorName);


        RoboLift = opMode.hardwareMap.dcMotor.get(RobotInfo.RoboLiftMotorName);
        switch(runMode){
            case AUTO_MODE:
                RoboLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RoboLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftWheel.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightWheel.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
                rightWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
                break;
            case DISABLED_MODE:
            case TELEOP_MODE:
            case TEST_MODE:
                RoboLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
                rightWheel.motor.setMode(RobotInfo.DRIVE_MOTOR_MODE);
                break;
            default:
                throw new IllegalArgumentException("Invalid runMode");
        }

        leftWheel.setInverted(false);
        rightWheel.setInverted(true);

        leftWheel.setBrakeModeEnabled(true);
        rightWheel.setBrakeModeEnabled(true);

        driveBase = new TrcSimpleDriveBase(leftWheel, rightWheel);
        driveBase.setPositionScales(RobotInfo.ENCODER_X_INCHES_PER_COUNT, RobotInfo.ENCODER_Y_INCHES_PER_COUNT);
        //
        // Initialize PID drive.
        //
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD),
                RobotInfo.ENCODER_Y_TOLERANCE, () -> driveBase.getYPosition());
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo.GYRO_KP, RobotInfo.GYRO_KI, RobotInfo.GYRO_KD),
                RobotInfo.GYRO_TOLERANCE, () -> driveBase.getHeading());
        gyroPidCtrl.setAbsoluteSetPoint(true);
        gyroPidCtrl.setOutputRange(-RobotInfo.TURN_POWER_LIMIT, RobotInfo.TURN_POWER_LIMIT);


        pidDrive = new TrcPidDrive("pidDrive", driveBase,null, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setStallTimeout(RobotInfo.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);

        ExtendoRotator = new FtcDcMotor(RobotInfo.ExtendoRotatorMotorName);
        Extendor = new FtcDcMotor(RobotInfo.ExtendorMotorName);


        Collector = new FtcDcMotor(RobotInfo.CollectorMotorName);
    }   //Robot5611

    void startMode(TrcRobot.RunMode runMode)
    {
        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(true);
        }

    }   //startMode

    void stopMode(TrcRobot.RunMode runMode){

        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(false);
        }
    }   //stopMode

    void tankDrive(double left, double right){
        dashboard.displayPrintf(10,"Left:  %5.2f Right:  %5.2f",left, right);
        driveBase.tankDrive(left, right);
    }
    void arcadeDrive(double y, double turn) {
        driveBase.arcadeDrive(y,turn);
    }
    void extendoArm(double extend, double rotate){
        Extendor.set(extend);
        ExtendoRotator.set(rotate);
    }
    public void collector(double collector) {
        Collector.set(collector);
    }

    double rotations;
    public void roboLiftInches(double inches) {

        rotations = inches;
        rotations *= 51.0/5.0; //51 teeth per 5 inches of rack
        rotations *= 1.0/25.0; //1 rotation per 25 teeth of pinion gear
        rotations *= 3.0; //3 rotations of motor per 1 rotation of pinion gear;
        rotations *= 1120.0; //280 encoder ticks per motor rotation
        roboLiftTicks((int)rotations);
    }
    public void roboLiftTicks(int ticks){
        int currentPosition = RoboLift.getTargetPosition();
        RoboLift.setTargetPosition(currentPosition+ticks);
        tracer.tracePrintf("EncoderTicks:  "+ticks);
    }
    public boolean roboLiftOnTarget() {
        dashboard.displayText(8,"TargetPos:  "+String.valueOf(RoboLift.getTargetPosition()));
        dashboard.displayText(9,"CurrentPos:  "+String.valueOf(RoboLift.getCurrentPosition()));
        return Math.abs(RoboLift.getTargetPosition()-RoboLift.getCurrentPosition())<5;
    }
    public void roboLift(double power){
        RoboLift.setPower(power);
    }

    void traceStateInfo(double elapsedTime, String stateName, double xDistance, double yDistance, double heading)
    {
        tracer.traceInfo(
                moduleName,
                "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f,volt=%5.2fV(%5.2fV)",
                elapsedTime, stateName,
                driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance,
                driveBase.getHeading(), heading,
                battery==null ? 0 : battery.getVoltage(), battery==null ? 0 : battery.getLowestVoltage());
    }   //traceStateInfo

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()
    {
        return opMode.gamepad1.dpad_up;
    }   //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return opMode.gamepad1.dpad_down;
    }   //isMenuDownButton

    @Override
    public boolean isMenuAltUpButton() {
        return false;
    }

    @Override
    public boolean isMenuAltDownButton() {
        return false;
    }

    @Override
    public boolean isMenuEnterButton()
    {
        return opMode.gamepad1.a;
    }   //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return opMode.gamepad1.dpad_left;
    }   //isMenuBackButton
}   //class Robot5611
