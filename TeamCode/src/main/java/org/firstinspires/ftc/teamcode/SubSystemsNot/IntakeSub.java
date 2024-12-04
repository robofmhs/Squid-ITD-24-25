//package org.firstinspires.ftc.teamcode.SubSystemsNot;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.sfdev.assembly.state.StateMachine;
//import com.sfdev.assembly.state.StateMachineBuilder;
//
//class StateMachines {
//    public enum Intake {
//        NEUTRAL, //parallel to ground
//        EXTEND,
//        INTAKE,
//        TRANSFER,
//    }
//
//    public static StateMachine getIntakeMachine(IntakeSub servos) {
//        return new StateMachineBuilder()
//                .state(Intake.NEUTRAL)
//                .onEnter( () -> {
//                    servos.IntakeDiffy1.setPosition(IntakeSub.IntakeDiffy1_NeutralPos);
//                    servos.IntakeDiffy2.setPosition(IntakeSub.IntakeDiffy2_NeutralPos);
//                    servos.IntakeSlide1.setPosition(IntakeSub.IntakeSlide_TurtlePos);
//                    servos.IntakeSlide2.setPosition(IntakeSub.IntakeSlide_TurtlePos);
//                })
//                .transition( () -> servos.gamepad1.x)
//
//                .state(Intake.EXTEND)
//                .onEnter( () -> {
//                    servos.IntakeSlide1.setPosition(IntakeSub.IntakeSlide_IntakePos);
//                    servos.IntakeSlide2.setPosition(IntakeSub.IntakeSlide_IntakePos);
//                })
//                .transitionTimed(0.5)
//
//                .state(Intake.INTAKE)
//                .onEnter( () -> {
//                    servos.IntakeDiffy1.setPosition(IntakeSub.IntakeDiffy1_IntakePos);
//                    servos.IntakeDiffy2.setPosition(IntakeSub.IntakeDiffy2_IntakePos);
//                })
//                .loop( () -> {
//                    if(servos.gamepad1.a) {
//                        servos.IntakeGripper.setPosition(IntakeSub.IntakeClose);
//                    }
//                })
//                .transition( () -> servos.gamepad1.x, Intake.NEUTRAL)
//
//                .state(Intake.TRANSFER)
//                .onEnter( () -> {
//                    servos.IntakeDiffy1.setPosition(IntakeSub.IntakeDiffy1_TransferPos);
//                    servos.IntakeDiffy2.setPosition(IntakeSub.IntakeDiffy2_TransferPos);
//                })
//                .transition( () -> IntakeSub.transfer_complete)
//
//                .build();
//    }
//
////    public enum Gunner {
////        TURTLE,
////        SPECINTAKE,
////        SPECOUTTAKE,
////        TRANSFERGUN,
////        BASKET
////    }
////    public static StateMachine getGunnerMachine(IntakeSub servos) {
////        return new StateMachineBuilder()
////                .state(Gunner.TURTLE)
////                .onEnter( () -> {
////                    servos.
////                })
////                .transition( () -> servos.gamepad1.x)
////
////                .build();
////    }
//}
//
//
//@TeleOp
//@Config
//public class IntakeSub extends LinearOpMode {
//    public Servo IntakeGripper;
//    public Servo IntakeDiffy1;
//    public Servo IntakeDiffy2;
//    public Servo IntakeSlide1;
//    public Servo IntakeSlide2;
//
//    public static boolean transfer_complete = true;
//
//    public static double IntakeOpen = 0;
//    public static double IntakeClose = .3;
//
//    public static double IntakeSlideTarget;
//
//    public static double IntakeSlide_TurtlePos;
//
//    public static double IntakeSlide_IntakePos;
//
//    public static double IntakeDiffy1_NeutralPos;
//    public static double IntakeDiffy2_NeutralPos;
//
//    public static double IntakeDiffy1_IntakePos;
//    public static double IntakeDiffy2_IntakePos;
//
//    public static double IntakeDiffy1_TransferPos;
//    public static double IntakeDiffy2_TransferPos;
//
//    public void setIntakeSlide(double x){
//        IntakeSlide1.setPosition(x);
//        IntakeSlide2.setPosition(x);
//    }
//    public void rotateDiffy(double x){
//        IntakeDiffy1.setPosition(IntakeDiffy1.getPosition()-x);
//        IntakeDiffy2.setPosition(IntakeDiffy2.getPosition()+x);
//    }
//    public void turnDiffy(double x){
//        IntakeDiffy1.setPosition(IntakeDiffy1.getPosition()+x);
//        IntakeDiffy2.setPosition(IntakeDiffy2.getPosition()+x);
//    }
//    public void setDiffy(double x){
//        IntakeDiffy1.setPosition(x);
//        IntakeDiffy2.setPosition(x);
//    }
//
//    public enum Global {
//        TURTLE,
//        IDLE, //parallel to ground
//        GLOBETRANSFER,
//        OCCUPIED,
//    }
//
//
//
//    public void gripper(Servo servo1, double pos1,double pos2){
//        if (servo1.getPosition() >= pos1 && servo1.getPosition() < pos1+.01) {
//            servo1.setPosition(pos2);
//
//        } else if (servo1.getPosition() >= pos2) {
//            servo1.setPosition(pos1);
//        } else {
//            servo1.setPosition(pos2);
//        }
//    }
//    @Override
//    public void runOpMode() throws InterruptedException {
//        IntakeGripper = hardwareMap.get(Servo.class,"IntakeGripper");
//        IntakeDiffy1 = hardwareMap.get(Servo.class,"IntakeDiffy1");
//        IntakeDiffy2 = hardwareMap.get(Servo.class,"IntakeDiffy2");
//        IntakeSlide1 = hardwareMap.get(Servo.class,"IntakeSlide1");
//        IntakeSlide2 = hardwareMap.get(Servo.class,"IntakeSlide2");
//
//        IntakeDiffy2.setDirection(Servo.Direction.REVERSE);
//        IntakeSlide2.setDirection(Servo.Direction.REVERSE);
//
//        public static StateMachine getGlobalMachine(IntakeSub servos) {
//            return new StateMachineBuilder()
//                    .state(Global.IDLE)
//                    .onEnter( () -> {
//
//                    })
//                    .loop( () -> {
//
//                    } ) //enter normal controls
//                    .transition( () -> servos.gamepad1.x)
//
//                    .state(Intake.EXTEND)
//                    .onEnter( () -> {
//                        servos.IntakeSlide1.setPosition(IntakeSub.IntakeSlide_IntakePos);
//                        servos.IntakeSlide2.setPosition(IntakeSub.IntakeSlide_IntakePos);
//                    })
//                    .transitionTimed(0.5)
//
//                    .state(Intake.INTAKE)
//                    .onEnter( () -> {
//                        servos.IntakeDiffy1.setPosition(IntakeSub.IntakeDiffy1_IntakePos);
//                        servos.IntakeDiffy2.setPosition(IntakeSub.IntakeDiffy2_IntakePos);
//                    })
//                    .loop( () -> {
//                        if(servos.gamepad1.a) {
//                            servos.IntakeGripper.setPosition(IntakeSub.IntakeClose);
//                        }
//                    })
//                    .transition( () -> servos.gamepad1.x, Intake.NEUTRAL)
//
//                    .state(Intake.TRANSFER)
//                    .onEnter( () -> {
//                        servos.IntakeDiffy1.setPosition(IntakeSub.IntakeDiffy1_TransferPos);
//                        servos.IntakeDiffy2.setPosition(IntakeSub.IntakeDiffy2_TransferPos);
//                    })
//                    .transition( () -> IntakeSub.transfer_complete)
//
//                    .build();
//        }
//        waitForStart();
//        while (opModeIsActive()){
//
//            if(gamepad1.a){
//                gripper(IntakeGripper,IntakeOpen,IntakeClose);
//                while (gamepad1.a){}
//            }
//        }
//    }
//}
