package team5611;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcDcMotor;
import ftclib.FtcGamepad;
import ftclib.FtcOpMode;
import trclib.TrcRobot;

@TeleOp(name="Vuforia Navigator Test", group="TeleOp")
public class FtcTeleOpVuforiaNavigationTest extends FtcOpMode {
    FtcDcMotor CollectorMotor;
    Robot5611 robot;
    float xSetting = 0f;
    float ySetting = 0f;
    float headingSetting = 0f;

    VuforiaNavigator.NavState testingState;

    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    boolean lastDpadLeft = false;
    boolean lastDpadRight = false;
    final static float adjustmentRate = 10;


    @Override
    public void initRobot() {
        robot = new Robot5611(TrcRobot.RunMode.TELEOP_MODE);
    }

    @Override
    public void runPeriodic(double elapsedTime) {
        if(gamepad1.right_bumper){
            switch(testingState){
                case TurnToTarget:
                    testingState = VuforiaNavigator.NavState.DriveToTarget;
                    break;
                case DriveToTarget:
                    testingState = VuforiaNavigator.NavState.TurnToHeading;
                    break;
                case TurnToHeading:
                    testingState = VuforiaNavigator.NavState.TurnToTarget;
                    break;
            }
        }

        boolean changed = false;
        switch(testingState){
            case TurnToTarget:
                if(gamepad1.dpad_up&&!lastDpadUp){
                    ySetting+=adjustmentRate;
                    changed = true;
                }
                if(gamepad1.dpad_down&&!lastDpadDown) {
                    ySetting -= adjustmentRate;
                    changed = true;
                }
                if(gamepad1.dpad_left&&!lastDpadLeft) {
                    xSetting -= adjustmentRate;
                    changed = true;
                }
                if(gamepad1.dpad_right&&!lastDpadRight){
                    xSetting += adjustmentRate;
                    changed = true;
                }
                if(changed){
                    robot.vuforiaNavigator.headingTest_setTarget(xSetting, ySetting);
                }
                telemetry.addData("x ", xSetting);
                telemetry.addData("y ", ySetting);
                break;
            case DriveToTarget:
                if(gamepad1.right_stick_y!=0)
                    robot.vuforiaNavigator.distanceTest_increaseDistance(gamepad1.right_stick_y);
                break;
            case TurnToHeading:
                headingSetting+=gamepad1.right_stick_x;
                if(gamepad1.right_stick_x!=0)
                    robot.vuforiaNavigator.headingTest_setHeading(headingSetting);
                telemetry.addData("heading ",headingSetting);
                break;
            case Done:
                break;
        }

        telemetry.addData("Currently Testing:  ", testingState);
        telemetry.addData("VuNav State", robot.vuforiaNavigator.navStateMachine.getState());


        lastDpadUp  = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
    }
}