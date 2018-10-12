/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

package common;

import android.speech.tts.TextToSpeech;
import android.widget.TextView;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftclib.FtcAndroidTone;
import ftclib.FtcBNO055Imu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcRobotBattery;
import hallib.HalDashboard;
import team3543.R;
import trclib.TrcDbgTrace;
import trclib.TrcGyro;
import trclib.TrcRobot;
import trclib.TrcSimpleDriveBase;

public class Robot implements FtcMenu.MenuButtons
{
    public static final boolean USE_SPEECH = true;
    //
    // Global objects.
    //
    public String moduleName;
    public FtcOpMode opMode;
    public HalDashboard dashboard;
    public TrcDbgTrace tracer;
    public FtcRobotBattery battery = null;
    public FtcAndroidTone androidTone;
    public TextToSpeech textToSpeech = null;
    //
    // Sensors.
    //
    public FtcBNO055Imu imu = null;
    public TrcGyro gyro = null;
    public double targetHeading = 0.0;
    //
    // DriveBase subsystem.
    //
    public TrcSimpleDriveBase driveBase = null;

    public Robot(TrcRobot.RunMode runMode)
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
        battery = new FtcRobotBattery();
        androidTone = new FtcAndroidTone("AndroidTone");
        if (USE_SPEECH)
        {
            textToSpeech = FtcOpMode.getInstance().getTextToSpeech();
            textToSpeech.speak("Init starting", TextToSpeech.QUEUE_FLUSH, null);
        }
        //
        // Initialize sensors.
        //
        imu = new FtcBNO055Imu("imu");
        gyro = imu.gyro;
    }   //Robot

    public void startMode(TrcRobot.RunMode runMode)
    {
        //
        // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
        //
        gyro.resetZIntegrator();
        gyro.setEnabled(true);
        targetHeading = 0.0;
    }   //startMode

    public void stopMode(TrcRobot.RunMode runMode)
    {
        //
        // Disable the gyro integrator.
        //
        gyro.setEnabled(false);

        if (textToSpeech != null)
        {
            textToSpeech.stop();
            textToSpeech.shutdown();
        }
    }   //stopMode

    public void traceStateInfo(double elapsedTime, String stateName, double xDistance, double yDistance, double heading)
    {
        tracer.traceInfo(
                moduleName,
                "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f,volt=%5.2fV(%5.2fV)",
                elapsedTime, stateName,
                driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance,
                driveBase.getHeading(), heading,
                battery.getVoltage(), battery.getLowestVoltage());
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
    public boolean isMenuEnterButton()
    {
        return opMode.gamepad1.a;
    }   //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return opMode.gamepad1.dpad_left;
    }   //isMenuBackButton

}   //class Robot
