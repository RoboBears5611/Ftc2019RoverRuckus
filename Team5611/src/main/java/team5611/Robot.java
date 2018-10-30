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

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftclib.FtcDcMotor;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcRobotBattery;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;

public class Robot implements FtcMenu.MenuButtons
{
    static final boolean USE_SPEECH = true;
    static final boolean USE_VUFORIA = true;

    private static final String moduleName = "Robot";
    //
    // Global objects.
    //
    FtcOpMode opMode;
    HalDashboard dashboard;
    TrcDbgTrace tracer;
    FtcRobotBattery battery = null;
    VuforiaVision vuforiaVision = null;


    //Peripherals
//    FtcDcMotor TestLeftMotor;
//    FtcDcMotor TestRightMotor;

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
//        battery = new FtcRobotBattery();

        if (USE_VUFORIA)
        {
            dashboard.displayPrintf(1,"Initializing Vuforia");
            int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            tracer.tracePrintf("CameraMonititorViewId:  %s",cameraViewId);
            dashboard.displayPrintf(3,"CamId %s",cameraViewId);
            vuforiaVision = new VuforiaVision(this, -1);//cameraViewId);
            dashboard.displayPrintf(1,"Initialized Vuforia");
        }

        dashboard.displayPrintf(2,"Loading up Peripherals.");
        //Peripherals
//        TestLeftMotor = new FtcDcMotor("TestLeftMotor");
//        TestRightMotor = new FtcDcMotor("TestRightMotor");
//
//        TestLeftMotor.setInverted(true);
//        TestRightMotor.setInverted(false);
//
//        TestLeftMotor.setBrakeModeEnabled(false);
//        TestRightMotor.setBrakeModeEnabled(false);
    }   //Robot

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
