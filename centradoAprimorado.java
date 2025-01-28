package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Modo Mecanum aprimorado com redução e telemetria")
public class centradoAprimorado extends OpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private static final double SLOW_MODE_FACTOR = 0.5;  // Fator para modo lento
    private static final double DEADZONE = 0.1;         // Limite para ignorar valores pequenos
    private static final double REDUCAO_DISTANCIA = 0.5; // Distância em que a redução ativa inicia
    private static final double REDUCAO_ANGULO = 10;     // Ângulo (em graus) para desacelerar na aproximação
    private double velocidadeAtual = 1.0;               // Fator de redução de velocidade

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
    }

    @Override
    public void loop() {
        // Leitura dos controles
        double vertical = applyDeadzone(-gamepad1.left_stick_y);
        double horizontal = applyDeadzone(gamepad1.right_stick_x);
        double giro = applyDeadzone(gamepad1.left_stick_x);

        // Modo lento com botão
        if (gamepad1.right_bumper) {
            vertical *= SLOW_MODE_FACTOR;
            horizontal *= SLOW_MODE_FACTOR;
            giro *= SLOW_MODE_FACTOR;
        }

        // Aplicação da redução progressiva baseada na entrada
        velocidadeAtual = calcularReducao(vertical, horizontal, giro);

        // Aplicação da curva exponencial para controle mais preciso
        vertical = Math.signum(vertical) * Math.pow(Math.abs(vertical), 2);
        horizontal = Math.signum(horizontal) * Math.pow(Math.abs(horizontal), 2);
        giro = Math.signum(giro) * Math.pow(Math.abs(giro), 2);

        // Ajuste para incluir redução ativa
        vertical *= velocidadeAtual;
        horizontal *= velocidadeAtual;
        giro *= velocidadeAtual;

        // Cálculo das potências dos motores
        double motorE1Pot = vertical + horizontal + giro;
        double motorD1Pot = vertical - horizontal - giro;
        double motorE2Pot = vertical - horizontal + giro;
        double motorD2Pot = vertical + horizontal - giro;

        // Normalização dos valores para o intervalo -1 a 1
        double[] powers = {motorE1Pot, motorD1Pot, motorE2Pot, motorD2Pot};
        normalizePower(powers);

        // Envio de potências para os motores
        leftFront.setPower(powers[0]);
        leftBack.setPower(powers[1]);
        rightFront.setPower(powers[2]);
        rightBack.setPower(powers[3]);

        // Telemetria para monitoramento
        telemetry.addData("Motor E1 Power", powers[0]);
        telemetry.addData("Motor D1 Power", powers[1]);
        telemetry.addData("Motor E2 Power", powers[2]);
        telemetry.addData("Motor D2 Power", powers[3]);
        telemetry.addData("Velocidade Atual", velocidadeAtual);
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

    // Método para calcular a redução de velocidade com base no movimento
    private double calcularReducao(double vertical, double horizontal, double giro) {
        // Simula uma redução baseada na proximidade de destino e ângulo
        double magnitude = Math.sqrt(vertical * vertical + horizontal * horizontal); // Distância
        double fatorDistancia = (magnitude < REDUCAO_DISTANCIA) ? magnitude / REDUCAO_DISTANCIA : 1.0;

        // Reduz giro perto do alvo
        double fatorGiro = (Math.abs(giro) < Math.toRadians(REDUCAO_ANGULO)) ? 0.5 : 1.0;

        // Retorna o menor fator como limite da velocidade
        return Math.min(fatorDistancia, fatorGiro);
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
