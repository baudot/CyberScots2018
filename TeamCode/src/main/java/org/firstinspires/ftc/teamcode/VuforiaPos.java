/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * This is NOT an opmode.
 *
 * This class can't be used to define all the specific hardware for a single robot.
 * In this case that robot is a Ragbot.
 */
public class VuforiaPos
{
    float mmPerInch = 25.4f;

    public void vuforiaInit() {

        String VUFORIA_KEY = "AcRNMjL/////AAAAGYx9swjI4EM3gOz2yIkuui5Eo0LMsIsAxmD+X+Lz2Eox41tmaut+zNhNGm68NGyXSnmYIwcWSIVz/fOZf+ht++8XJScyjQv/BDbKKOOEZ3//KzhxFkS93SlQ3OKX+KhDdtv/USecJsSYAMY/A77pOHu10H6SXHGC2fTuCa1+mzp6rEpugFFC0JxcTJSyFTx3IMvH4BPU98zZbTbb8bnVl1usz84xusFUTKGua19+lvZ1gBwfe/SltwgQZEmzTrQPT7K8cnu0obpmxspet8k5FHbqeJvzXV9PMK1wd2+wYygYQMeJCOrg/ZIE/fRODW4sgDIt6L85XMehoidJ3aE2csreAsiSaQsFgnYe4H07XwDi";

        // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
        // We will define some constants and conversions here

        float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
        float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

        // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
        // Valid choices are:  BACK or FRONT
        final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

        boolean targetVisible = false;

        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */


        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;


        VuforiaLocalizer vuforia;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
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

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT  = 0;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 311;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        targetsRoverRuckus.activate();
    }

    private LocRot getLocRot() {

        OpenGLMatrix lastLocation = null;
        // check all the trackable target to see which one (if any) is visible.
        boolean targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                //telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible && lastLocation != null) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

            LocRot lr = new LocRot();
            lr.location = translation;
            lr.rotation = rotation;

            return lr;
        }
        else {
            return null;
        }

    }

    static final double LOCROT_TIMEOUT = 500; //Time until the last known location is no longer working (in milliseconds)

    private List<VuforiaTrackable> allTrackables;

    private HardwareRagbotNoArm robot;

    private HardwareMap hwMap;

    private Telemetry telemetry;

    private ElapsedTime averageTime = new ElapsedTime();

    static final double TIME_TO_AVERAGE = 500;

    ElapsedTime timeSinceLastLocRot = new ElapsedTime();

    boolean lrIsValid = false;
    LocRot lastlr = null;

    public void updatePosition() {
        LocRot locrot = getLocRot();

        if (locrot != null) { // A poster is visible (sometimes doesn't work even if a poster is visible)
            lastlr = locrot;
            timeSinceLastLocRot.reset();
        }else {
            //do something if no poster is seen
        }
        lrIsValid = timeSinceLastLocRot.milliseconds() < LOCROT_TIMEOUT; //If the location and rotation is still valid, the time since it was last found is under the timeout
    }

    public VuforiaPos(HardwareRagbotNoArm robot, Telemetry telemetry){
        this.robot = robot;
        this.hwMap = robot.hardwareMap;
        this.telemetry = telemetry;
    }

    public void move(double forward, double turn) {
        double leftPower = forward;
        double rightPower = forward;
        leftPower += turn;
        rightPower -= turn;
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        robot.frontLeftDrive.setPower(leftPower);
        robot.backLeftDrive.setPower(leftPower);
        robot.frontRightDrive.setPower(rightPower);
        robot.backRightDrive.setPower(rightPower);
    }

    public void init() {
        vuforiaInit();
    }

    public void driveToPoint(double xDestination, double yDestination, LinearOpMode opmode) {
        LocRot startLocRot = lastlr;
        averageTime.reset();
        while (averageTime.milliseconds() < TIME_TO_AVERAGE && opmode.opModeIsActive()) {
            updatePosition();

            if (lastlr != null/* && lrIsValid*/) {
                if (startLocRot == null) {
                    startLocRot = lastlr;
                }
                telemetry.addLine("We think lastlr is not null and is valid");
                telemetry.update();
                //startLocRot.location.put(0,  (startLocRot.location.get(0) + lastlr.location.get(0))/2);
                //startLocRot.location.put(1,  (startLocRot.location.get(1) + lastlr.location.get(1))/2);
                //startLocRot.location.put(2,  (startLocRot.location.get(2) + lastlr.location.get(2))/2);

                //startLocRot.rotation.firstAngle = (startLocRot.rotation.firstAngle + lastlr.rotation.firstAngle) / 2;
                //startLocRot.rotation.secondAngle = (startLocRot.rotation.secondAngle + lastlr.rotation.secondAngle) / 2;
                //startLocRot.rotation.thirdAngle = (startLocRot.rotation.thirdAngle + lastlr.rotation.thirdAngle) / 2;
                robot.stopMoving();
            }else {
                telemetry.addLine("OOOOOF: No poster found!");
                telemetry.update();
                averageTime.reset();
                robot.move(0.0, -0.5);
            }
        }

        robot.stopMoving();

        telemetry.addLine("Done with getting position");
        telemetry.update();
        opmode.sleep(3000);

        if (lastlr != null) {
            telemetry.addLine("Wrapping up locrot handling.");
            telemetry.update();

            double dx = startLocRot.location.get(0) - xDestination;
            double dy = startLocRot.location.get(1) - yDestination;

            telemetry.addData("dx: ", dx);
            telemetry.addData("dy: ", dy);
            telemetry.update();
            opmode.sleep(5000);
            //double dx = 0.1;
            //double dy = 0.1;

            if (dy == 0) {
                dy = 1;
            }

            double angleToTurn = Math.atan(dx/dy);

            if (dy < 0) {
                angleToTurn += Math.PI;
            }

            double fractionToTurn = angleToTurn / (Math.PI * 2);

            telemetry.addData("Angle to turn: ", angleToTurn);
            telemetry.addData("Fraction of whole circle: " , fractionToTurn);
            telemetry.update();
            opmode.sleep(5000);

            robot.encoderDrive(HardwareRagbotNoArm.TURN_SPEED, HardwareRagbotNoArm.FULL_CIRCLE_INCHES * fractionToTurn, HardwareRagbotNoArm.FULL_CIRCLE_INCHES * -fractionToTurn, 3, opmode);

            double driveDistance = Math.sqrt((dx * dx) + (dy * dy)) / mmPerInch;

            telemetry.addLine("Done turning");
            telemetry.addData("Distance (in): ", driveDistance);
            telemetry.update();
            opmode.sleep(4000);

            robot.encoderDrive(HardwareRagbotNoArm.DRIVE_SPEED, driveDistance, driveDistance, 5, opmode);
        }else {
            telemetry.addLine("Something is really wrong, lastlr is null");
            telemetry.update();
        }
    }
}