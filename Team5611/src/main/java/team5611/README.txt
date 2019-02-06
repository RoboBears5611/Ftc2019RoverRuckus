Overview of the RoboBears codestuffs.

The root of the operation is "FtcTeleOpDrive", for TeleOp,  and "FtcAuto" for autonomous.  Both have interesting ways of implementing their logic.

"FtcTeleOpDrive" maps controls to the robot.  Just to make life exciting, those are separate things.
    * "Controls" is an *abstract* class, meaning it defines the requirements for many different variety of controls
      (abstract classes differ from a interfaces in that an the former can also do some helpful logic itself)
      -The point of the "Controls" class is to map controls from the actual gamepads to different functions, like "forward drive power"
      -Since we're using a special library, the gamepads are of the type "FtcGamepad", which is basically a harder-to-use wrapper of normal gamepads.
        -I really don't know why I used it.  You have to do binary logic to get to buttons.
            -if you want to change buttons, you have to look up the button in "FtcGamepad", find the bit offset, and put that number into the "<<" function.
        * "Default Controls" - Are the usual controls, as the name implies.  If you want to change any controls, this is where you do it.
    * "Robot5611" contains all the control logic for the robot (it is used in both TeleOp and Autonomous programs)
        -It contains initialization logic (like grabbing motors) and some helper functions (like tankDrive(double left, double right))
        *RobotInfo contains a bunch of static information for the robot, mostly motor names (you shouldn't mess with the PID stuff)

"FtcAuto" is really just a starter program.  It asks the user what it wants to do with a menu, then starts a 'command' to do that.
     - The menu system is through the TRC library we used.  It's cool if you want to learn it, you can change parameters (like how far to drive)
        on the fly (right before you run the programs), and then pass the value you get (say, the # of feet to drive) to the command.
    * "CmdAutoFull" this contains the actual autonomous logic for our autonomous.
        -it's implemented as a State Machine - basically, a main function is executed dozens of times a second, and what the function does changes based on
           the current 'state'.  If the starting state is 'delay', then the state machine can keep delaying until it has waited long enough and can change
           the state for the next execution
           -this state machine is different in that it uses a TRC library helper that supports 'events'.  You can get events from things like the TRC timer,
              and then tell the TRC state machine to do nothing (skip the state switch function) until that event is fired.
           -the code for each state is switched between by a 'switch' statement, which is wrapped in an 'if' to skip it if the TRC state machine says to.
              (i.e., it's waiting for an event)
    * "CmdTurn" is a bogus test program you shouldn't need to use.

"MotorTester" is a handy program to test motors in the configuration.
    - RB and LB switch between motors
    - Right Joystick powers the current motor
    - Dpad Up and Down change the "multiplier", which can be lowered to slow down the motors (a multiplier of 0.5 will halve the range the joystick operates in)
    - B switches to and from Encoder Mode
        - Encoder mode utilizes the encoders to set the motors to a definite position, which can be slowly changed with the right joystick
        - Dpad Up and Down change the rate with which the joystick changes the encoder's position (more encoder ticks per second)

"FtcTestVuforia" and "VuforiaVision" - are not used right now.


***APPENDIX OF DOOM:  Connecting remotely.
1.  Plug in the (turned-on) robot phone
2.  Open a Command Line
3.  Type "adb tcpip 5555"  (NOTE:  If you don't have 'adb' accessible from command line, you need to add it's containing folder to your PATH variable.  Look up how to do that)
    a.  Extra NOTE:  If it says "daemon not running, starting daemon" (or something close) when you run the above statement, run it again, so it says "restarting on port 5555"
4.  Connect via hotspot
    a.  Open the robot controller app on the phone
    b.  Go to "Program and Manage" from the hamburger menu
    c.  This will turn the hotspot on;  connect using the password on the page (you will not be able to use normal internet through this hotspot and you will get a little warning icon)
5.  Type "adb connect 192.168.49.1" on your command line
6.  You're connected, you can download like you would if you were plugged in (it just might be a little slower)
    a.  NOTE:  Don't use  "Instant run" to speed things up, though it will ask you to - in my experience, it just doesn't do anything, I don't know why.