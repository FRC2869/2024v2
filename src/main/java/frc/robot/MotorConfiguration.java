package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

public class MotorConfiguration {
    private double P;
    public double getP() {
        return P;
    }

    public void setP(double p) {
        P = p;
    }

    private double I;
    public double getI() {
        return I;
    }

    public void setI(double i) {
        I = i;
    }

    private double D;
    public double getD() {
        return D;
    }

    public void setD(double d) {
        D = d;
    }

    private double Iz;
    public double getIz() {
        return Iz;
    }

    public void setIz(double iz) {
        Iz = iz;
    }

    private double F;
    private double maxOutput;
    private double minOutput;
    private double gearRatio = 1;
    private double position;
    private int currentLimit;
    private boolean idleCoast;
    private boolean inverted;
    private double openLoopRampRate;

    public double getOpenLoopRampRate() {
        return this.openLoopRampRate;
    }

    public void setOpenLoopRampRate(double openLoopRampRate) {
        this.openLoopRampRate = openLoopRampRate;
    }

    public double getF() {
        return this.F;
    }

    public void setF(double F) {
        this.F = F;
    }

    public double getMaxOutput() {
        return this.maxOutput;
    }

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public double getMinOutput() {
        return this.minOutput;
    }

    public void setMinOutput(double minOutput) {
        this.minOutput = minOutput;
    }

    public double getGearRatio() {
        return this.gearRatio;
    }

    public void setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }

    public double getPosition() {
        return this.position;
    }

    public void setPosition(double position) {
        this.position = position;
    }

    public int getCurrentLimit() {
        return this.currentLimit;
    }

    public void setCurrentLimit(int currentLimit) {
        this.currentLimit = currentLimit;
    }

    public boolean isIdleCoast() {
        return this.idleCoast;
    }

    public void setIdleCoast(boolean idleCoast) {
        this.idleCoast = idleCoast;
    }

    public boolean isInverted() {
        return this.inverted;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }
    
    public MotorConfiguration(double P, double I, double D, double Iz, double F, double maxOutput, double minOutput, double gearRatio, double position, int currentLimit, boolean idleCoast, boolean inverted, double openLoopRampRate){
        this.P = P;
        this.I = I;
        this.D = D;
        this.Iz = Iz;
        this.F = F;
        this.maxOutput = maxOutput;
        this.minOutput = minOutput;
        this.gearRatio = gearRatio;
        this.position = position;
        this.currentLimit = currentLimit;
        this.idleCoast = idleCoast;
        this.inverted = inverted;
        this.openLoopRampRate = openLoopRampRate;
    }

    public MotorConfiguration(double maxOutput, double minOutput, int currentLimit, boolean idleCoast, boolean inverted){
        this.maxOutput = maxOutput;
        this.minOutput = minOutput;
        this.currentLimit = currentLimit;
        this.idleCoast = idleCoast;
        this.inverted = inverted;
    }

    public static RelativeEncoder configureMotor(CANSparkBase motor, MotorConfiguration config){
        motor.restoreFactoryDefaults();
        var pid = motor.getPIDController();
        pid.setP(config.getP());
        pid.setI(config.getI());
        pid.setD(config.getD());
        pid.setIZone(config.getIz());
        pid.setFF(config.getF());
        pid.setOutputRange(config.getMinOutput(), config.getMaxOutput());
        var encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(config.getGearRatio());
        encoder.setPosition(config.getPosition());
        motor.setSmartCurrentLimit(config.getCurrentLimit());
        motor.setInverted(config.isInverted());
        motor.setIdleMode(config.isIdleCoast() ? IdleMode.kCoast : IdleMode.kBrake);
        motor.setOpenLoopRampRate(config.getOpenLoopRampRate());
        motor.burnFlash();
        return encoder;
    }

    public static TalonFXConfiguration configureMotor(TalonFX motor, MotorConfiguration config){
        var c = motor.getConfigurator();
        var configuration = new TalonFXConfiguration();
        configuration.CurrentLimits.SupplyCurrentLimit = config.getCurrentLimit();
        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.MotorOutput.Inverted = config.isInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        configuration.MotorOutput.NeutralMode = config.isIdleCoast() ?  NeutralModeValue.Coast : NeutralModeValue.Brake;
        configuration.MotorOutput.PeakForwardDutyCycle = config.getMaxOutput();
        configuration.MotorOutput.PeakReverseDutyCycle = config.getMinOutput();
        configuration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = config.getOpenLoopRampRate();
        configuration.Slot0.kP = config.getP();
        configuration.Slot0.kI = config.getI();
        configuration.Slot0.kD = config.getD();
        configuration.Slot0.kG = config.getF();
        configuration.Feedback.SensorToMechanismRatio = config.getGearRatio();
        motor.setPosition(config.getPosition());
        c.apply(configuration);
        return configuration;
    }
}
