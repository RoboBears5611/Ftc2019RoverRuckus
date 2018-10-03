/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="ServoTester", group="TEST")  // @Autonomous(...) is the other common choice

public class ServoTester extends OpMode
{
    private Servo ActiveServo;
    private Servo[] Servos;
    private String[] ServoNames;
    private int activeIndex;
    private long lastUpdatetime;
    private int encoderSpeed = 5;
    private final static double encoderSpeedChangeRate = 0.5;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Looking for servos....");
        Set<Map.Entry<String,Servo>> servoset =  hardwareMap.servo.entrySet();
        ArrayList<String> names = new ArrayList<String>();
        ArrayList<Servo> servos = new ArrayList<Servo>();
        StringBuilder sb = new StringBuilder();
        for(Map.Entry<String,Servo> entry : servoset){
            String name = entry.getKey();
            names.add(name);
            servos.add(entry.getValue());
            sb.append(name);
            sb.append(",");
        }
        ServoNames = names.toArray(new String[names.size()]);
        Servos = servos.toArray(new Servo[servos.size()]);
        activeIndex = 0;
        telemetry.addLine("Connected Servos:  "+sb.toString());
        telemetry.addData("Status","Servos Found!  Awaiting commands...");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        boolean update = true;
        if(lastUpdatetime>System.currentTimeMillis()-250) {
            update = false;
        }else if(gamepad1.right_bumper){
            activeIndex++;
            telemetry.addData("Status","Incrementing servo");
        }else if(gamepad1.left_bumper){
            activeIndex--;
            telemetry.addData("Status","Decrementing servo");
        }else if(gamepad1.a) {
            activeIndex = 0;
            telemetry.addData("Status", "Returning to start of servo list");
        }else{
            update = false;
        }

        if(update){
            while(activeIndex>=ServoNames.length){
                activeIndex-=ServoNames.length;
            }
            lastUpdatetime = System.currentTimeMillis();


            ActiveServo = Servos[activeIndex];

            telemetry.addData("Active Servo",ServoNames[activeIndex]);
        }
        if(ActiveServo != null){
            ActiveServo.setPosition(gamepad1.right_stick_y/2+0.5);


            telemetry.addData("Active Servo Position:  ",ActiveServo.getPosition());
            telemetry.addData("Active Servo",ServoNames[activeIndex]);
        }
        telemetry.addData("Status",ActiveServo==null?
                "Awaiting specification of Servo (via LT and RT)"
                :"Currently Driving Servo with name '"+ServoNames[activeIndex]+"' to position "+ActiveServo.getPosition());

    }


}
