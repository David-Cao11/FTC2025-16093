package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.references.XCYBoolean;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp
@Config
public class TestArmPID extends LinearOpMode {
    public static int referenceAngle = 45;
    public static int position = 200;
//    public static double kS = 0;
    public static double kCos = 0;
//    public static double kV = 0;
//    public static double kA = 0;
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

// NOTE: motors have internal PID control

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure superstructure = new SuperStructure(
                this,
                () -> {
                    logic_period();
                    drive_period();
                });
        NewMecanumDrive drive =new NewMecanumDrive( );
//        ArmFeedforward feedforward = new ArmFeedforward(kS, kCos, kV, kA);


        //XCYBoolean testMove = new XCYBoolean(()->gamepad1.b);
        XCYBoolean testArm = new XCYBoolean(()->gamepad1.a);
        drive.setUp(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.update();
        superstructure.resetSlide();

        waitForStart();

        superstructure.setSlidePosition(0);
        Runnable update = ()->{drive.update();superstructure.update();XCYBoolean.bulkRead();};

        while (opModeIsActive()) {
            superstructure.setSlidePosition(SSValues.SLIDE_MIN);
            double power = Math.cos(referenceAngle) * kCos;

            if(testArm.toTrue()){
                superstructure.setArmPosition(position, power);
            }

//            superstructure.setArm(feedforward.calculate(1,2,2));

            telemetry_M.addData("arm:", superstructure.getArmPosition());
            telemetry.addData("slideL: ",superstructure.getSlideLeftPosition());
            telemetry.addData("slideR: ",superstructure.getSlideRightPosition());
            telemetry.addData("Touch Sensor",superstructure.getTouchSensorPressed());
            telemetry_M.addData("armPower: ",superstructure.getArmPower());
            telemetry_M.addData("Arm Error",superstructure.getArmPosition() - superstructure.getArmTargetPosition());
            telemetry_M.update();
            update.run();
        }
    }

    private void drive_period() {
        //there's nothing here
    }

    private void logic_period() {
        XCYBoolean.bulkRead();
        telemetry.update();
    }

}