package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Modo Mecanum aprimorado")
public class MecanumAP extends OpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private static final double SLOW_MODE_FACTOR = 0.5;
    private static final double DEADZONE = 0.1;

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
    }

    @Override
    public void loop() {
        double vertical = applyDeadzone(-gamepad1.left_stick_y);
        double horizontal = applyDeadzone(gamepad1.right_stick_x);
        double giro = applyDeadzone(gamepad1.left_stick_x);

        // Modo lento
        if (gamepad1.right_bumper) {
            vertical *= SLOW_MODE_FACTOR;
            horizontal *= SLOW_MODE_FACTOR;
            giro *= SLOW_MODE_FACTOR;
        }

        // Ajuste da curva exponencial para controle mais preciso
        vertical = Math.signum(vertical) * Math.pow(Math.abs(vertical), 2);
        horizontal = Math.signum(horizontal) * Math.pow(Math.abs(horizontal), 2);
        giro = Math.signum(giro) * Math.pow(Math.abs(giro), 2);

        double motorE1Pot = vertical + horizontal + giro;
        double motorD1Pot = vertical - horizontal - giro;
        double motorE2Pot = vertical - horizontal + giro;
        double motorD2Pot = vertical + horizontal - giro;

        // Normalização dos valores para o intervalo -1 a 1
        double[] powers = {motorE1Pot, motorD1Pot, motorE2Pot, motorD2Pot};
        normalizePower(powers);

        leftFront.setPower(powers[0]);
        leftBack.setPower(powers[1]);
        rightFront.setPower(powers[2]);
        rightBack.setPower(powers[3]);

        // Telemetria para monitoramento
        telemetry.addData("Motor E1 Power", powers[0]);
        telemetry.addData("Motor D1 Power", powers[1]);
        telemetry.addData("Motor E2 Power", powers[2]);
        telemetry.addData("Motor D2 Power", powers[3]);
        telemetry.update();
    }

    // Método para aplicar deadzone
    private double applyDeadzone(double value) {
        return (Math.abs(value) > DEADZONE) ? value : 0;
    }

    // Método para normalizar potências
    private void normalizePower(double[] powers) {
        double maxPower = Math.abs(powers[0]);
        for (double power : powers) {
            if (Math.abs(power) > maxPower) maxPower = Math.abs(power);
        }
        if (maxPower > 1) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= maxPower;
            }
        }
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
