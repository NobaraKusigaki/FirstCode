// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private Joystick joy = new Joystick(0);
    private VictorSPX m_Right1 = new VictorSPX(2);
    private VictorSPX m_Right2 = new VictorSPX(1);
    private VictorSPX m_Left1 = new VictorSPX(7);
    private VictorSPX m_Left2 = new VictorSPX(3);


    boolean bntA, bntB, bntX, bntY;
    double spdbutton = 1;
    double Y1, X1, X2, Y2;
    double seno, seno2, mag, mag2;
    int pov;
    double mL = 0;
    double mR = 0;
    boolean analog1 = true;
    boolean analog2 = true;
    double g_L = 0;
    double g_R = 0;

    @Override
    public void robotInit() {
        m_Right2.follow(m_Right1);
        m_Left2.follow(m_Left1);

        m_Right1.setInverted(true);
        m_Right2.setInverted(true);

        m_Left1.configNeutralDeadband(0.04);
        m_Right1.configNeutralDeadband(0.04);

        m_Left1.setNeutralMode(NeutralMode.Brake);
        m_Right1.setNeutralMode(NeutralMode.Brake);

    }

    @Override
    public void teleopPeriodic() {
        m_Left1.set(ControlMode.PercentOutput, mL);
        m_Right1.set(ControlMode.PercentOutput, mR);

        pov = joy.getPOV(0);

        bntA = joy.getRawButton(1);
        bntB = joy.getRawButton(2);
        bntX = joy.getRawButton(3);
        bntY = joy.getRawButton(4);

        g_R = joy.getRawAxis(2);
        g_L = -joy.getRawAxis(3);

        Y1 = -joy.getRawAxis(1);
        X1 = joy.getRawAxis(0);
        Y2 = -joy.getRawAxis(5);
        X2 = joy.getRawAxis(4);

        mag = Math.hypot(X1, Y1);
        mag2 = Math.hypot(X2, Y2);

        spdbutton = calcbutton(bntA, bntB, bntX);
        activeAnalog(analog1, analog2, Y1, X1, X2, Y2);

        if (pov != -1)
            calcpov(pov);

        if (g_R != 0)
            calctrigger(g_R, spdbutton);

        if (g_L != 0)
            calctrigger2(g_L, spdbutton);

        if (mag >= 0.04)
            calcAnalog1(analog1, X1, Y1, mag, seno);

        if (mag2 >= 0.04)
            calcAnalog2(analog2, X2, Y2, mag2, seno2);

        SmartDashboard();

    }
public void SmartDashboard(){
    SmartDashboard.putBoolean("botao (A)  ativado", bntA);
    SmartDashboard.putBoolean(" botao  (B)  ativado", bntB);
    SmartDashboard.putBoolean(" botao  (X) ativado", bntX);
    SmartDashboard.putNumber("speedbutton", spdbutton);
    SmartDashboard.putBoolean("analog1", analog1);
    SmartDashboard.putNumber("motor_R1", mR);
    SmartDashboard.putNumber("motor_L1", mL);
    SmartDashboard.putBoolean("analog2", analog2);
    SmartDashboard.putNumber("trigger_R", g_R);
    SmartDashboard.putNumber("trigger_L", -g_L);
    SmartDashboard.putNumber("pov", pov);

}
    public double calcbutton(boolean bntA, boolean bntB, boolean bntX) {
        if (joy.getRawButton(1)) {
            spdbutton = 0.50;
        } else if (joy.getRawButton(2)) {
            spdbutton = 0.25;
        } else if (joy.getRawButton(3)) {
            spdbutton = 1;

        }
        return spdbutton;

    }

    public void activeAnalog(boolean analog1, boolean analog2, double X1, double Y1, double X2, double Y2) {
        if (mag != 0) {
            analog1 = true;
            analog2 = false;
            calcAnalog1(analog1, X1, Y1, X2, Y2);
        }
        if (mag2 != 0) {
            analog2 = true;
            analog1 = false;
            calcAnalog2(analog2, X2, Y2, X2, Y2);
        }
        if (analog1 == false && analog2 == false) {
            mL = 0.0;
            mR = 0.0;
        }
    }

    public void calcAnalog1(boolean analog1, double X1, double Y1, double mag, double seno) {
        if (analog1) {
            seno = Y1 / mag;

            // QUADRANTE 1
            if (X1 >= 0 && Y1 >= 0) {
                mL = mag * spdbutton;
                mR = (2 * seno - 1) * mag * spdbutton;
            }
            // QUADRANTE 2
            else if (X1 < 0 && Y1 >= 0) {
                mL = (2 * seno - 1) * mag * spdbutton;
                mR = mag * spdbutton;
            }
            // QUADRANTE 4
            else if (X1 < 0 && Y1 < 0) {
                mL = (2 * seno + 1) * mag * spdbutton;
                mR = -mag * spdbutton;
            }
            // QUADRANTE 3
            else if (X1 >= 0 && Y1 < 0) {
                mL = -mag * spdbutton;
                mR = (2 * seno + 1) * mag * spdbutton;
            }

        }
    }

    public void calcAnalog2(boolean analog2, double X2, double Y2, double mag2, double seno2) {
        if (analog2) {
            seno2 = Y2 / mag2;
            // QUADRANTE 1
            if (X2 >= 0 && Y2 >= 0) {
                mR = (-2 * seno2 + 1) * mag2 * spdbutton;
                mL = -mag2 * spdbutton;
            }
            // QUADRANTE 4
            if (X2 < 0 && Y2 < 0) {
                mR = (-2 * seno2 - 1) * mag2 * spdbutton;
                mL = mag2 * spdbutton;
            }
            // QUADRANTE 3
            else if (X2 >= 0 && Y2 < 0) {
                mR = mag2 * spdbutton;
                mL = (-2 * seno2 - 1) * mag2 * spdbutton;
            }
            // QUADRANTE 2
            else if (X2 < 0 && Y2 >= 0) {
                mR = -mag2 * spdbutton;
                mL = (-2 * seno2 + 1) * mag2 * spdbutton;
            }

        }
    }

    public void calctrigger(double g_R, double spdbutton) {
        if (g_R != 0) {
            g_R *= spdbutton;
        }
        if (X1 >= 0) {
            mR = g_R * (1 - X1) * spdbutton;
            mL = g_R * spdbutton;
        } else if (X1 < 0) {
            mR = g_R * spdbutton;
            mL = g_R * (1 + X1) * spdbutton;
        }

    }

    public void calctrigger2(double g_L, double spdbutton) {
        if (g_L != 0) {
            g_L *= spdbutton;
        }
        if (X1 >= 0) {
            mR = g_L * spdbutton;
            mL = g_L * (1 - X1) * spdbutton;
        } else if (X1 < 0) {
            mR = g_L * (1 + X1) * spdbutton;
            mL = g_L * spdbutton;
        }

    }

    public void calcpov(int pov) {
        switch (pov) {
            case 0:
                mL = 0.40;
                mR = 0.40;
                break;

            case 45:
                mL = 0.40;
                mR = 0;
                break;

            case 90:
                mL = 0.40;
                mR = -0.40;
                break;

            case 135:
                mL = 0;
                mR = -0.40;
                break;

            case 180:
                mL = -0.40;
                mR = -0.40;
                break;

            case 225:
                mL = -0.40;
                mR = 0;
                break;

            case 270:
                mL = -0.40;
                mR = 0.40;
                break;

            case 315:
                mL = 0;
                mR = 0.40;
                break;

            default:
            case -1:
                mL = 0;
                mR = 0;

        }

    }

}