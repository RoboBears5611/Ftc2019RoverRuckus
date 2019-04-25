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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Date;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import trclib.TrcRobot;

@Autonomous(name="Autonomous", group="5611Auto")
public class FtcAuto extends FtcOpMode
{
    private static final boolean USE_TRACELOG = true;

    enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    enum Strategy
    {
        SIMPLE_AUTO,
        ADJUSTED_AUTO,
        DO_NOTHING,
        FULL_AUTO
    }   //enum Strategy

    enum FieldSide{
        FRONT, BACK
    }

    private static final String moduleName = "FtcAuto";

    private Robot5611 robot;
    private TrcRobot.RobotCommand autoCommand = null;
    private Alliance alliance = Alliance.RED_ALLIANCE;
    private double delay = 0.0;
    private Strategy strategy = Strategy.DO_NOTHING;
    private double turnDegrees;
//    private double driveDistance = 0.0;
//    private double driveTime = 0.0;
//    private double drivePower = 0.0;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        robot = new Robot5611(TrcRobot.RunMode.AUTO_MODE);
        //
        // Choice menus.
        //
        doMenus();

        if (USE_TRACELOG)
        {
            robot.tracer.openTraceLog("/sdcard/FIRST/tracelog");
        }

        //
        // Strategies.
        //
        switch (strategy)
        {
            case FULL_AUTO:
                autoCommand = new CmdAutoFull(robot,delay);
                break;
            case SIMPLE_AUTO:
                autoCommand = new CmdAutoSimple_Original(robot, delay);
                break;
            case ADJUSTED_AUTO:
                autoCommand = new CmdAutoSimple(robot,delay);
                break;
//            case DISTANCE_DRIVE:
//                autoCommand = new CmdPidDrive(
//                        robot, robot.pidDrive, delay, 0.0, driveDistance*12.0, 0.0);
//                break;
//
//            case TIMED_DRIVE:
//                autoCommand = new CmdTimedDrive(robot, delay, driveTime, 0.0, drivePower, 0.0);
//                break;
            case DO_NOTHING:
            default:
                autoCommand = null;
                break;
        }
        robot.dashboard.displayText(7,"IT'S WAITING");
    }   //initRobot

    public void startMode(TrcRobot.RunMode runMode)
    {
        robot.tracer.traceInfo(moduleName, "%s: ***** Starting autonomous *****", new Date());
        robot.startMode(TrcRobot.RunMode.AUTO_MODE);
        robot.battery.setEnabled(true);
        robot.dashboard.clearDisplay();
    }   //startMode

    public void stopMode(TrcRobot.RunMode runMode)
    {
        robot.stopMode(TrcRobot.RunMode.AUTO_MODE);
        robot.battery.setEnabled(false);
        printPerformanceMetrics(robot.tracer);

        if (USE_TRACELOG)
        {
            robot.tracer.closeTraceLog();
        }
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoCommand != null)
        {
            autoCommand.cmdPeriodic(elapsedTime);
            robot.dashboard.displayText(6,"Running Auto Command");

        }
        robot.dashboard.displayText(5,"Running Command");
    }   //runContinuous

    private void doMenus()
    {
        //
        // Create menus.
        //
//        FtcChoiceMenu<MatchType> matchTypeMenu = new FtcChoiceMenu<>("Match type:", null, robot);
//        FtcValueMenu matchNumberMenu = new FtcValueMenu(
//                "Match number:", matchTypeMenu, robot,
//                1.0, 50.0, 1.0, 1.0, "%.0f");

        FtcChoiceMenu<Strategy> strategyMenu = new FtcChoiceMenu<>("Strategies:", null, robot);
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", null, robot);
        FtcValueMenu delayMenu = new FtcValueMenu(
                "Delay time:", null, robot,
                0.0, 30.0, 1.0, 0.0, " %.0f sec");
        FtcChoiceMenu<FieldSide> fieldSideMenu = new FtcChoiceMenu<>("Field Side:",null, robot);

        strategyMenu.addChoice("Full Autonomous",Strategy.FULL_AUTO,true, allianceMenu);
        strategyMenu.addChoice("Simple Autonomous",Strategy.SIMPLE_AUTO,true, delayMenu);
        strategyMenu.addChoice("Adjusted Simple Autonomous", Strategy.ADJUSTED_AUTO, false, delayMenu);
        strategyMenu.addChoice("Do nothing", Strategy.DO_NOTHING, false, delayMenu);

//        delayMenu.setChildMenu(strategyMenu);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true, fieldSideMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false, fieldSideMenu);

        fieldSideMenu.addChoice("Front", FieldSide.FRONT, true, delayMenu);
        fieldSideMenu.addChoice("Back", FieldSide.BACK, false, delayMenu);

        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(strategyMenu);
        //
        // Fetch choices.
        //
        delay = delayMenu.getCurrentValue();
        strategy = strategyMenu.getCurrentChoiceObject();
        alliance = allianceMenu.getCurrentChoiceObject();
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(1, "== Match ==");
        robot.dashboard.displayPrintf(2, "Auto Strategy: %s", strategyMenu.getCurrentChoiceText());
        robot.dashboard.displayPrintf(3, "Alliance=%s,Delay=%.0f sec", alliance.toString(), delay);
//        robot.dashboard.displayPrintf(4, "Drive: distance=%.0f ft,Time=%.0f,Power=%.1f",
//                                      driveDistance, driveTime, drivePower);
        telemetry.update();
    }   //doMenus

}   //class FtcAuto
