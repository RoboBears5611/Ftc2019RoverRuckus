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

package team5611;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

        import ftclib.FtcVuforia;
        import trclib.TrcUtil;

        import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VuforiaVision
{
    private static final int IMAGE_WIDTH = 640;     //in pixels
    private static final int IMAGE_HEIGHT = 480;    //in pixels
    private static final int FRAME_QUEUE_CAPACITY = 2;
    //
    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here.
    //
    // Width of the FTC field (from the center point to the outer panels)
    private static final float FTC_FIELD_WIDTH_MM  = (12*6) * (float)TrcUtil.MM_PER_INCH;   //6 ft. in mm
    // Height of the center of the target image above the floor.
    private static final float TARGET_HEIGHT_MM = (6) * (float)TrcUtil.MM_PER_INCH;         //6 inches in mm

    private Robot5611 robot5611;
    private FtcVuforia vuforia;
    private VuforiaTrackable[] imageTargets = null;
    private OpenGLMatrix lastRobotLocation = null;

    public VuforiaVision(
            Robot5611 robot5611, int cameraViewId, VuforiaLocalizer.CameraDirection cameraDir, OpenGLMatrix phoneLocation)
    {
        final String VUFORIA_LICENSE_KEY =
                "AaXobyf/////AAABmf2229UqeE0Glf90ORMEc7+MsrR1FfF0gydPcpd" +
                        "8rUew1njTJbXa6Nc6s1+GJEKltYEXkRLE4kp8QWmKMVQoIw+e" +
                        "9zcQ6IqFfx5a4LItqtRY+j1gJStDQf8tui4Gjr2/hKPW3Z6CT/" +
                        "Xy2Aq45DcT+s3UYd83YtsNOW55gxqwTmKM0KmNnr+nk6Z7K9i0" +
                        "XtplQhF/HjZsOrBxCfPch/dQ+v/SrOIbuvPRVPUncu5+drw0vw" +
                        "kGG4lhM0vt/KPv7GEWYUChSJYHgXZ5+GkuQuWLTYVgkEKfriT+" +
                        "6S7Lx4XoHpyOgNSnxgg+F6UBWHzkTlQLDC40zfRwdGmWh8Z8ao" +
                        "y+AGK0ZKLQQZu4/40ytpXpTNQ2";
        final String TRACKABLE_IMAGES_FILE = "RoverRuckus";

        this.robot5611 = robot5611;
        vuforia = new FtcVuforia(VUFORIA_LICENSE_KEY, cameraViewId, cameraDir);
        vuforia.configVideoSource(IMAGE_WIDTH, IMAGE_HEIGHT, FRAME_QUEUE_CAPACITY);


        /*
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /*
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, FTC_FIELD_WIDTH_MM, TARGET_HEIGHT_MM)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));

        /*
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -FTC_FIELD_WIDTH_MM, TARGET_HEIGHT_MM)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));

        /*
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-FTC_FIELD_WIDTH_MM, 0, TARGET_HEIGHT_MM)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));

        /*
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(FTC_FIELD_WIDTH_MM, 0, TARGET_HEIGHT_MM)
                .multiplied(RobotInfo.VuforiaCameraRotation);

        FtcVuforia.TargetInfo[] imageTargetsInfo =
                {
                        new FtcVuforia.TargetInfo(0, "Blue-Rover", false, blueRoverLocationOnField),
                        new FtcVuforia.TargetInfo(1, "Red-Footprint", false, blueRoverLocationOnField),
                        new FtcVuforia.TargetInfo(2, "Front-Craters", false, frontCratersLocationOnField),
                        new FtcVuforia.TargetInfo(3, "Back-Space", false, backSpaceLocationOnField)
                };

        vuforia.addTargetList(TRACKABLE_IMAGES_FILE, imageTargetsInfo, phoneLocation);
        imageTargets = new VuforiaTrackable[imageTargetsInfo.length];
        for (int i = 0; i < imageTargets.length; i++)
        {
            imageTargets[i] = vuforia.getTarget(imageTargetsInfo[i].name);
        }

//        FtcVuforia.TargetInfo[] objectTargetsInfo =
//        {
//                new FtcVuforia.TargetInfo(0, "Team-Marker", true, null)
//        };
//
//        vuforia.addTargetList(TRACKABLE_OBJECTS_FILE, objectTargetsInfo, null);
    }   //VuforiaVision

    public void setEnabled(boolean enabled)
    {
        vuforia.setTrackingEnabled(enabled);
    }   //setEnabled

    public OpenGLMatrix getRobotLocation()
    {
        OpenGLMatrix robotLocation = null;
        boolean targetVisible = false;

        for (int i = 0; i < imageTargets.length; i++)
        {
            if (vuforia.isTargetVisible(imageTargets[i]))
            {
                targetVisible = true;
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix location = vuforia.getRobotLocation(imageTargets[i]);
                if (location != null)
                {
                    lastRobotLocation = location;
                }
                break;
            }
        }

        if (targetVisible)
        {
            robotLocation = lastRobotLocation;
        }

        return robotLocation;
    }   //getRobotLocation

    public VectorF getLocationTranslation(OpenGLMatrix location)
    {
        final String funcName = "getLocationTranslation";

        VectorF translation = location.getTranslation();
        // express position (translation) of robot in inches.
        robot5611.tracer.traceInfo(funcName, "Translation: x=%6.2f, y=%6.2f, z=%6.2f",
                translation.get(0)/TrcUtil.MM_PER_INCH,
                translation.get(1)/TrcUtil.MM_PER_INCH,
                translation.get(2)/TrcUtil.MM_PER_INCH);
        return translation;
    }   //getLocationTranslation

    public Orientation getLocationOrientation(OpenGLMatrix location)
    {
        final String funcName = "getLocationOrientation";

        Orientation orientation = Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);
        // express the rotation of the robot in degrees.
        robot5611.tracer.traceInfo(funcName, "Orientation: roll=%6.2f, pitch=%6.2f, heading=%6.2f",
                orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle);
        return orientation;
    }   //getLocationOrientation
}   //class VuforiaVision
