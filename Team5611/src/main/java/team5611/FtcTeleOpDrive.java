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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import ftclib.FtcGamepad;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcRobot;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;

@TeleOp(name="Basic Drive", group="TeleOp")
public class FtcTeleOpDrive extends FtcOpMode
{
    protected HalDashboard dashboard;
    protected Robot5611 robot;

    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    protected Controls controls;

    private double drivePowerScale = 1.0;
    private boolean invertedDrive = false;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //

        robot = new Robot5611(TrcRobot.RunMode.TELEOP_MODE);

        dashboard = robot.dashboard;
        dashboard.displayPrintf(2,"Robot5611 Object Created!");
        //
        // Initializing Gamepads.
        //
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2);
        controls = new DefaultControls(driverGamepad,operatorGamepad);
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
    public void runPeriodic(double elapsedTime) {
        OpenGLMatrix location = this.robot.vuforiaVision.getRobotLocation();
        if (location != null) {
            VectorF translation = robot.vuforiaVision.getLocationTranslation(location);
            robot.dashboard.displayPrintf(3, "Position:  x:  %4.2f, y:  %4.2f, z:  %4.2f", translation.get(0), translation.get(1), translation.get(2));
            Orientation orientation = robot.vuforiaVision.getLocationOrientation(location).toAxesOrder(XYZ);
            robot.dashboard.displayPrintf(4, "Orientation:  x:  %4.2f, y:  %4.2f, z:  %4.2f", orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle);
        }   //runPeriodic
        robot.drive(controls.forward(),controls.turn());
    }
}   //class FtcTeleOpDrive
