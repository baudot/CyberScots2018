package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Sampling Test", group = "Cyber Scots")
//@Disabled
public class samplingTest extends LinearOpMode {
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    HardwareRagbotNoArm robot = new HardwareRagbotNoArm();
    static final double left_whip_up_pos = 0;
    static final double left_whip_side_pos = 0.15;
    static final double center_whip_up_pos = 0;
    static final double center_whip_side_pos = 0.3;
    static final double right_whip_up_pos = 0;
    static final double right_whip_side_pos = 0.4;

    public void sampling() {
        //Find the cube
        telemetry.addData("Red", robot.sensorColor.red());
        telemetry.addData("Green", robot.sensorColor.green());
        telemetry.addData("Blue ", robot.sensorColor.blue());
        robot.sensorColor.enableLed(true);
        if ((robot.sensorColor.red() + robot.sensorColor.green() + robot.sensorColor.blue() > 75)){
            telemetry.addData("Cube", "Found Cube");robot.frontLeftDrive.setPower(1);
            robot.frontLeftDrive.setPower(1);
            robot.frontRightDrive.setPower(1);
            robot.backLeftDrive.setPower(1);
            robot.backRightDrive.setPower(1);
            sleep(250);
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);

        } else {
            telemetry.addLine("CALABUNGA!!!");
            robot.frontLeftDrive.setPower(1);
            robot.frontRightDrive.setPower(-1);
            robot.backLeftDrive.setPower(1);
            robot.backRightDrive.setPower(-1);
            sleep(250);
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);
            telemetry.addLine("JACK_IS_YOUR_GOD!!!");
        }
        telemetry.update();
    }

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        sampling();
    }
}