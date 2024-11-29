package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.BarkMecanumDrive;
import org.firstinspires.ftc.teamcode.references.SSValues;
import org.firstinspires.ftc.teamcode.uppersystems.Action;
import org.firstinspires.ftc.teamcode.uppersystems.ArmAction;
import org.firstinspires.ftc.teamcode.uppersystems.ClawAction;
import org.firstinspires.ftc.teamcode.uppersystems.GrabAction;
import org.firstinspires.ftc.teamcode.uppersystems.SlideAction;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;
import org.firstinspires.ftc.teamcode.uppersystems.WristAction;

@Config
public abstract class AutoMaster extends LinearOpMode {

    public static final int POSITIVE = 1;
    public static final int NEGATIVE = -1;
    public static final int RED = -1;
    public static final int BLUE = 1;

    protected int startSide;
    protected int side_color;

    private BarkMecanumDrive drive;
    protected SuperStructure upper;
    private Runnable update;
    Pose2d startPos;


    protected void initHardware(Pose2d p) throws InterruptedException{
        //TODO check if this start pose is correct (10% chance not correct)
        startPos = p;
        //TODO measure these because these are 100% not correct

        telemetry.addLine("init: drive");
        telemetry.update();
        drive = new BarkMecanumDrive(hardwareMap);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(startPos);
        drive.update();
        drive.setSimpleMoveTolerance(2,2,Math.toRadians(10));

        update = ()->{
            drive.update();
            telemetry.update();
        };

        telemetry.addLine("init: superstructure");
        telemetry.update();
        upper = new SuperStructure(
                this,
                () -> {
                }, SSValues.AUTO_ARM_OFFSET);

        update = ()->{
            drive.update();
            telemetry.update();
            upper.update();
        };

        drive.setUpdateRunnable(update);

        upper.resetSlide();
        upper.setGrabPos(SSValues.GRAB_DEFAULT);
        upper.setWristPos(SSValues.WRIST_DEFAULT);
        upper.setSlidesByP(SSValues.SLIDE_MIN, 0.93);//Maybe we should test this!
        upper.setArmByP(SSValues.ARM_DEFAULT, 0.5);
        upper.setClawLeftPos(SSValues.CLAW_LEFT_CLOSE);
        upper.setClawRightPos(SSValues.CLAW_RIGHT_CLOSE);

        telemetry.addLine("init: trajectory");
        telemetry.update();


    }

    ///////////////////////////////////SUPERSTRUCTURE///////////////////////////////////////////////
    protected void reset(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        // Sequence actions based on last sequence
        if (upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_FAR || upper.getPreviousSequence() == SuperStructure.Sequences.INTAKE_NEAR || upper.getPreviousSequence() == SuperStructure.Sequences.CUSTOM_INTAKE) {
            upper.setGrabPos(SSValues.GRAB_CLOSED);
            Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 100));
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 10));
        } else if (upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_BASKET || upper.getPreviousSequence() == SuperStructure.Sequences.HANG || upper.getPreviousSequence() == SuperStructure.Sequences.LOW_BASKET) {
            upper.setGrabPos(SSValues.GRAB_DEFAULT);
            Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 50));
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
            Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 50));
            Action.actions.add(new ArmAction(upper, SSValues.ARM_DEFAULT, 300));
        }else if(upper.getPreviousSequence() == SuperStructure.Sequences.HIGH_CHAMBER){
            Action.actions.add(new WristAction(upper, SSValues.WRIST_DEFAULT, 100));
            Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 300));
            Action.actions.add(new ArmAction(upper, SSValues.ARM_DEFAULT,200));
        }
        Action.buildSequence(update);
    }

    protected void highChamberAim(){
        upper.switchSequence(SuperStructure.Sequences.HIGH_CHAMBER);
        Action.actions.add(new WristAction(upper, SSValues.WRIST_HIGH_CHAMBER));
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_AIM,50));
        Action.buildSequence(update);
    }

    protected void highChamberPlace(){
        Action.actions.add(new WristAction(upper, SSValues.WRIST_ABOVE_SAMPLES));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_HIGH_CHAMBER_PLACE,70));
        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_OPEN, SSValues.CLAW_RIGHT_OPEN,70));
        Action.buildSequence(update);
    }


    ///////////////////////////////////BLUE//////////////////////////////////////////

    protected void moveToBlueChamberAim(){
        drive.setSimpleMoveTolerance(3,3, Math.toRadians(5));
        drive.setSimpleMovePower(0.87);
        drive.moveTo(new Pose2d(0, 45, Math.toRadians(90)), 100);
    }

    protected void moveToBlueChamberPlace(int xOffset){
        drive.setSimpleMoveTolerance(2, 2, Math.toRadians(5));
        drive.setSimpleMovePower(0.87);
        drive.moveTo(new Pose2d(0+xOffset, 34, Math.toRadians(90)), 100);

    }

    protected void pushTwoBlueSamples(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(3,3, Math.toRadians(10));
        drive.setSimpleMovePower(0.87);
        drive.moveTo(new Pose2d(-35, 36, Math.toRadians(90)), 0);
        drive.moveTo(new Pose2d(-35, 15, Math.toRadians(90)), 0);
        drive.moveTo(new Pose2d(-45, 10, Math.toRadians(180)), 100);
        drive.moveTo(new Pose2d(-45, 54, Math.toRadians(180)), 0);
        drive.moveTo(new Pose2d(-45, 15, Math.toRadians(180)), 0);
        drive.moveTo(new Pose2d(-55, 15, Math.toRadians(180)), 0);
        drive.moveTo(new Pose2d(-55, 54, Math.toRadians(180)), 0);
        drive.moveTo(new Pose2d(-54, 48, Math.toRadians(-120)), 200);
    }

    protected void intakeLastBlueSample(){
        upper.switchSequence(SuperStructure.Sequences.INTAKE_NEAR);
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        upper.startIntake();
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,400));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_AUTO_INTAKE_LAST,20,0.5));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,1800));//This is just a timer
        Action.buildSequence(update);
        upper.stopIntake();
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE,300));//This is just a timer
        Action.buildSequence(update);
    }

    protected void placeLastBlueSampleAtHP(){
        upper.switchSequence(SuperStructure.Sequences.RUN);
        drive.setSimpleMoveTolerance(1,1, Math.toRadians(5));
        drive.setSimpleMovePower(0.87);
        drive.moveTo(new Pose2d(-40, 55, Math.toRadians(-90)), 100);
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP,50));
        Action.actions.add(new GrabAction(upper, SSValues.GRAB_OPEN, 800));//1200
        Action.buildSequence(update);
        drive.moveTo(new Pose2d(-40, 60, Math.toRadians(-90)), 100);
    }

    protected void prepareToClawIntakeBlueSpecimen(int yOffset){
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_MIN, 10));
        Action.buildSequence(update);
        drive.moveTo(new Pose2d(-40, 60+yOffset, Math.toRadians(-90)), 300);
    }

    protected void clawBlueSampleUp(){
        Action.actions.add(new ClawAction(upper, SSValues.CLAW_LEFT_CLOSE, SSValues.CLAW_RIGHT_CLOSE, 400));
        Action.actions.add(new SlideAction(upper, SSValues.SLIDE_SLIGHTLY_LONGER, 30));
        Action.buildSequence(update);
    }



    ///////////////////////////////////////////////TESTS////////////////////////////////////////////

    protected void autoUpperTest(){
        Action.actions.add(new ArmAction(upper, SSValues.ARM_UP));
        Action.actions.add(new WristAction(upper, SSValues.WRIST_INTAKE, 5000));
        upper.setIntake(SSValues.CONTINUOUS_SPIN);
        delay(5000);
        Action.buildSequence(update);
    }

    protected void autoResetArmTest(){
        Action.actions.add(new ArmAction(upper, SSValues.ARM_DEFAULT));
        Action.buildSequence(update);
    }


    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
            idle();
            update.run();
        }
    }
}