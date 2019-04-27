package team5611;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcDcMotor;
import ftclib.FtcGamepad;
import ftclib.FtcOpMode;
import trclib.TrcDbgTrace;
import trclib.TrcPidController;
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

    TrcDbgTrace trace;

    double P = 0f;
    double I = 0f;
    double D = 0f;
    PID ActivePID = PID.P;
    boolean unappliedPIDChanges = false;

    enum PID {
        P,I,D
    }

    @Override
    public void initRobot() {
        robot = new Robot5611(TrcRobot.RunMode.TELEOP_MODE);
        testingState = testingState.TurnToTarget;
        trace = TrcDbgTrace.getGlobalTracer();
        TrcPidController.PidCoefficients pidCoefficients = robot.vuforiaNavigator.pidDrive.getTurnPidCtrl().getPidCoefficients();
        P = pidCoefficients.kP;
        I = pidCoefficients.kI;
        D = pidCoefficients.kD;
    }

    boolean lastGamepadB = false;
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
                robot.dashboard.displayPrintf(6,"x:  %3.1f",xSetting);
                robot.dashboard.displayPrintf(7,"y:  %3.1f",ySetting);
                break;
            case DriveToTarget:
                if(gamepad1.right_stick_y!=0)
                    robot.vuforiaNavigator.distanceTest_increaseDistance(gamepad1.right_stick_y);
                robot.dashboard.displayPrintf(6,"distance change:  %1.4f",gamepad1.right_stick_y);
                break;
            case TurnToHeading:
                headingSetting+=gamepad1.right_stick_x;
                if(gamepad1.right_stick_x!=0)
                    robot.vuforiaNavigator.headingTest_setHeading(headingSetting);
                robot.dashboard.displayPrintf(6,"heading:  %3.1f",headingSetting);

                break;
            case Done:
                break;
        }


        if(gamepad1.b&&!lastGamepadB){
            ActivePID = PID.values()[(ActivePID.ordinal()+1)%3];
        }
        double PIDdelta = -gamepad1.left_stick_y*0.05;
        if(PIDdelta!=0){
            trace.tracePrintf("Changing PID!!!");
            switch(ActivePID){
                case P:
                    P+=PIDdelta;
                    break;
                case I:
                    I+=PIDdelta;
                    break;
                case D:
                    D+=PIDdelta;
                    break;
            }
            unappliedPIDChanges = true;
        }
        if(unappliedPIDChanges){
            if(gamepad1.a){
                robot.vuforiaNavigator.vuforiaTurnPidCtrl.setPidCoefficients(new TrcPidController.PidCoefficients(P,I,D));
                robot.vuforiaNavigator.resetPID();
                unappliedPIDChanges = false;
                robot.dashboard.displayText(8,"");
            }
            else robot.dashboard.displayText(8,"Press 'A' to apply PID Changes");
        }

        robot.dashboard.displayPrintf(4,"Currently Testing:  %s", testingState);
        robot.dashboard.displayPrintf(5,"Robot State:  %s", robot.vuforiaNavigator.navStateMachine.getState());
        robot.dashboard.displayPrintf(9,"P:  %4.4f",P);
        robot.dashboard.displayPrintf(10,"I:  %4.4f",I);
        robot.dashboard.displayPrintf(11,"D:  %4.4f",D);

        lastDpadUp  = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastGamepadB = gamepad1.b;
    }

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode) {
        robot.dashboard.clearDisplay();
        robot.startMode(nextMode);
    }

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.stopMode(nextMode);
        printPerformanceMetrics(robot.tracer);
    }   //stopMode
}