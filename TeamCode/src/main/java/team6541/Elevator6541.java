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

package team6541;

import ftclib.FtcDcMotor;
import ftclib.FtcDigitalInput;
import ftclib.FtcServo;
import trclib.TrcEvent;
import trclib.TrcPidActuator;
import trclib.TrcPidController;

public class Elevator6541
{
    private FtcDigitalInput upperLimitSwitch;
    private FtcDigitalInput lowerLimitSwitch;
    private FtcDcMotor elevatorMotor;
    private TrcPidController pidController;
    private TrcPidActuator pidElevator;
    private FtcServo hookServo;

    public Elevator6541()
    {
        upperLimitSwitch = new FtcDigitalInput("elevatorUpperLimit");
        lowerLimitSwitch = new FtcDigitalInput("elevatorLowerLimit");
        upperLimitSwitch.setInverted(true);
        lowerLimitSwitch.setInverted(true);

        elevatorMotor = new FtcDcMotor("elevatorMotor", lowerLimitSwitch, upperLimitSwitch);
        elevatorMotor.setBrakeModeEnabled(true);

        pidController = new TrcPidController("elevatorPidController",
                new TrcPidController.PidCoefficients(
                    Robot6541Info.ELEVATOR_KP, Robot6541Info.ELEVATOR_KI, Robot6541Info.ELEVATOR_KD),
                Robot6541Info.ELEVATOR_TOLERANCE, this::getPosition);
        pidElevator = new TrcPidActuator("pidElevator", elevatorMotor, lowerLimitSwitch, pidController,
                Robot6541Info.ELEVATOR_CAL_POWER, Robot6541Info.ELEVATOR_MIN_HEIGHT, Robot6541Info.ELEVATOR_MAX_HEIGHT);
        pidElevator.setPositionScale(Robot6541Info.ELEVATOR_INCHES_PER_COUNT, Robot6541Info.ELEVATOR_ZERO_OFFSET);

        hookServo = new FtcServo("hookServo");
    }

    public void zeroCalibrate()
    {
        pidElevator.zeroCalibrate();
    }

    public void setPower(double power)
    {
        pidElevator.setPower(power);
    }

    public void setPosition(double target, TrcEvent event, double timeout)
    {
        pidElevator.setTarget(target, event, timeout);
    }

    public double getPosition()
    {
        return pidElevator.getPosition();
    }

    public boolean isUpperLimitSwitchActive()
    {
        return upperLimitSwitch.isActive();
    }

    public boolean isLowerLimitSwitchActive()
    {
        return lowerLimitSwitch.isActive();
    }

    public void setHookPosition(double pos)
    {
        hookServo.setPosition(pos);
    }

    public void openHook()
    {
        hookServo.setPosition(Robot6541Info.HANGING_HOOK_OPEN_POSITION);
    }

    public void closeHook()
    {
        hookServo.setPosition(Robot6541Info.HANGING_HOOK_CLOSE_POSITION);
    }

}   //class Elevator6541
