/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Date;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * the Adafruit RGB Sensor.  It assumes that the I2C
 * cable for the sensor is connected to an I2C port on the
 * Core Device Interface Module.
 *
 * It also assuems that the LED pin of the sensor is connected
 * to the digital signal pin of a digital port on the
 * Core Device Interface Module.
 *
 * You can use the digital port to turn the sensor's onboard
 * LED on or off.
 *
 * The op mode assumes that the Core Device Interface Module
 * is configured with a name of "dim" and that the Adafruit color sensor
 * is configured as an I2C device with a name of "sensor_color".
 *
 * It also assumes that the LED pin of the RGB sensor
 * is connected to the signal pin of digital port #5 (zero indexed)
 * of the Core Device Interface Module.
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "Autonomous (Color Sensor)", group = "Sensor")
public class AutonomousColorSensorOp extends LinearOpMode {

    private ColorSensor sensorRGB;
    private DeviceInterfaceModule cdim;
    private Servo SuperWacker3000; //The official name of the extendo color sensor ball wacker arm
    private double StateTransitionTime;
    private CSTState CurrentState = CSTState.Initialization;
    private TimeBasedMechanumDriveBase timeBasedMechanumDriveBase;



    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    private static final int LED_CHANNEL = 5;
    private static final double WackerRetractedPosition = 0.95;
    private static final double WackerExtendedPosition = 0;
//    private static final double WackerUpperSensorAdjustment = 0.1;
//    private static final double WackerLowerSensorAdjustment = 0;
//    private static final double WackerAdjustmentAmount = 0.01;

    @Override
    public void runOpMode() {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get("dim");



        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannel.Mode.OUTPUT);

        // get a reference to our ColorSensor object.
        sensorRGB = hardwareMap.colorSensor.get("sensor_color");
        SuperWacker3000 = hardwareMap.servo.get("SuperWacker3000");
        timeBasedMechanumDriveBase = new TimeBasedMechanumDriveBase(hardwareMap,telemetry);


        // turn the LED on in the beginning, just so user will know that the sensor is active.
        cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);

        SuperWacker3000.setPosition(WackerRetractedPosition);


        // wait for the start button to be pressed.
        waitForStart();

        SuperWacker3000.setPosition(WackerExtendedPosition);

        // loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())  {

            // convert the RGB values to HSV values.
            Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", sensorRGB.alpha());
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("Hue", hsvValues[0]);


            float hue = values[0];
            double blue = calculateAbsoluteAngleDifference(185,hue);
            boolean isBlue = blue  < 35;
            double red =  calculateAbsoluteAngleDifference(0,hue);
            boolean isRed = red < 18;
            boolean isClear = sensorRGB.alpha()<175;

            telemetry.addData("blueDistance",blue);
            telemetry.addData("isBlue",isBlue);
            telemetry.addData("redDistance",red);
            telemetry.addData("isRed",isRed);
            telemetry.addData("isClear",isClear);
            boolean isWantedColor = !isClear&&(isBlue || isRed);

            String ColorString;
            if(!isWantedColor){
                ColorString = "Neither";

            }else if(isBlue){
                ColorString = "Blue";
            }else{
                ColorString = "Red";
            }


            telemetry.addData("Current Color",ColorString);

            double StateTransDeltaMillis = new Date().getTime()-StateTransitionTime;

            //Execution State Machine
            switch(CurrentState){
                case Initialization:
                    ChangeState(CSTState.ExtendingWacker);
                    break;
                case ExtendingWacker:
                    if(StateTransDeltaMillis>750){
                        ChangeState(CSTState.DeterminingColor);
                    }
                    break;
                case DeterminingColor:
                    if(isWantedColor){
                        ChangeState(CSTState.DeterminedColor);
                    }else if(StateTransDeltaMillis>50){
//                        ChangeState(CSTState.AdjustingSensorUp);
                    }
                    break;
//                case DeterminingColorDown:
//                    if(isWantedColor) {
//                        ChangeState(CSTState.DeterminedColor);
//                    }else if(StateTransDeltaMillis>50){
//                        ChangeState(CSTState.AdjustingSensorDown);
//                    }
//                    break;
////                case AdjustingSensorUp:
//                    if(SuperWacker3000.getPosition()>=WackerUpperSensorAdjustment){
//                        ChangeState(CSTState.AdjustingSensorDown);
//                    }
//                    if(StateTransDeltaMillis>50) {
//                        ChangeState(CSTState.DeterminingColor);
//                    }
//                    break;
//                case AdjustingSensorDown:
//                    if(SuperWacker3000.getPosition()<=WackerLowerSensorAdjustment){
//                        ChangeState(CSTState.AdjustingSensorUp);
//                    }
//                    if(StateTransDeltaMillis>50){
//                        ChangeState(CSTState.DeterminingColorDown);
//                    }
//                    break;
                case DeterminedColor:
                    telemetry.addLine("Color Determined as "+ColorString);
                    if(isRed){
                        ChangeState(CSTState.DrivingForward);
                    }else if(isBlue){
                        ChangeState(CSTState.DrivingBackward);
                    }else{
                        telemetry.addLine("I think the fabric of my reality just tore.  Or somebody just messed with the code and now we're trying to knock off a color without actually having figured out which color is which.");
                    }
                    break;
                case DrivingForward:
                case DrivingBackward:
                    if(!timeBasedMechanumDriveBase.isBusy()){
                        ChangeState(CSTState.Complete);
                    }
                    break;
                case Complete:
                    telemetry.addData("Status","COMPLETE!!!");

                default:
                    telemetry.addLine("ERROR:  UNKNOWN STATE");
            }
            telemetry.addData("CurrentState",CurrentState);
            telemetry.update();
        }
    }
    private void ChangeState(CSTState newState){
        switch(newState){
            case ExtendingWacker:
                SuperWacker3000.setPosition(WackerExtendedPosition);
                break;
//            case AdjustingSensorDown:
//                SuperWacker3000.setPosition(SuperWacker3000.getPosition()-WackerAdjustmentAmount);
//                break;
//            case AdjustingSensorUp:
//                SuperWacker3000.setPosition(SuperWacker3000.getPosition()+WackerAdjustmentAmount);
//                break;
            case DrivingForward:
                timeBasedMechanumDriveBase.move(0,0.25f,0,250);
                break;
            case DrivingBackward:
                timeBasedMechanumDriveBase.move(0,-0.25f,0,250);
                break;
            default:
                break;  //If there are no special actions to be taken whilst transition, we can just transition and call it good
        }
        CurrentState = newState;
        StateTransitionTime = new Date().getTime();
    }

    private enum CSTState{  //ColorSensorTest States
        Initialization, DrivingForward, ExtendingWacker,DeterminingColor,
        //DeterminingColorDown, AdjustingSensorUp, AdjustingSensorDown,
        DrivingBackward, Complete, DeterminedColor

    }

    //Gets difference from 0 for red.  yikes.
    // put the targeted hue first
    private double calculateAbsoluteAngleDifference(double angleOne, double angleTwo){
        double angle = angleOne - angleTwo;

        return Math.abs(angle);
    }

}
