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

package team3543;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import ftclib.FtcGamepad;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcGameController;
import trclib.TrcRobot;

@TeleOp(name="TeleOp", group="3543TeleOp")
public class FtcTeleOp extends FtcOpMode implements TrcGameController.ButtonHandler
{
    private enum DriveMode
    {
        TANK_MODE,
        MECANUM_MODE,
    }   //enum DriveMode

    protected HalDashboard dashboard;
    protected Robot robot;

    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;

    private double drivePowerScale = 1.0;
    private boolean invertedDrive = false;
    private DriveMode driveMode = DriveMode.MECANUM_MODE;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        robot = new Robot(TrcRobot.RunMode.TELEOP_MODE);
        dashboard = robot.dashboard;
        //
        // Initializing Gamepads.
        //
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1, this);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2, this);
        driverGamepad.setYInverted(true);
        operatorGamepad.setYInverted(true);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode runMode)
    {
        dashboard.clearDisplay();
        robot.startMode(runMode);
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode runMode)
    {
        robot.stopMode(runMode);
        printPerformanceMetrics(robot.tracer);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // DriveBase subsystem.
        //
        switch(driveMode)
        {
            case TANK_MODE:
                double leftPower = driverGamepad.getLeftStickY(true)*drivePowerScale;
                double rightPower = driverGamepad.getRightStickY(true)*drivePowerScale;
                robot.driveBase.tankDrive(leftPower, rightPower, invertedDrive);
                dashboard.displayPrintf(1, "Tank:left=%.1f,right=%.1f,inv=%s",
                                        leftPower, rightPower, Boolean.toString(invertedDrive));
                break;

            case MECANUM_MODE:
                double x = driverGamepad.getLeftStickX(true)*drivePowerScale;
                double y = driverGamepad.getRightStickY(true)*drivePowerScale;
                double rot = (driverGamepad.getRightTrigger(true) -
                              driverGamepad.getLeftTrigger(true))*drivePowerScale;
                robot.driveBase.holonomicDrive(x, y, rot, invertedDrive);
                dashboard.displayPrintf(1, "Mecan:x=%.1f,y=%.1f,rot=%.1f,inv=%s",
                                        x, y, rot, Boolean.toString(invertedDrive));
                break;
        }

        double elevatorPower = operatorGamepad.getRightStickY(true);
        robot.elevator.setPower(elevatorPower);

        dashboard.displayPrintf(2, "DriveBase: x=%.2f,y=%.2f,heading=%.2f",
                robot.driveBase.getXPosition(),
                robot.driveBase.getYPosition(),
                robot.driveBase.getHeading());
        dashboard.displayPrintf(3, "Elevator: power=%.1f, pos=%.1f (%s,%s)",
                elevatorPower, robot.elevator.getPosition(),
                robot.elevator.isLowerLimitSwitchActive(), robot.elevator.isUpperLimitSwitchActive());
    }   //runPeriodic


    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    @Override
    public void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        dashboard.displayPrintf(
                7, "%s: %04x->%s", gamepad.toString(), button, pressed? "Pressed": "Released");
        if (gamepad == driverGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    if (pressed)
                        driveMode = DriveMode.MECANUM_MODE;
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (pressed)
                        driveMode = DriveMode.TANK_MODE;
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    drivePowerScale = pressed? 0.5: 1.0;
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    invertedDrive = pressed;
                    break;
            }
        }
        else if (gamepad == operatorGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    break;

                case FtcGamepad.GAMEPAD_BACK:
                    break;

                case FtcGamepad.GAMEPAD_START:
                    break;
            }
        }
    }   //buttonEvent

}   //class FtcTeleOp
