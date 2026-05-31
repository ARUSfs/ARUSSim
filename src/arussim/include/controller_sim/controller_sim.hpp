#pragma once
#include <vector>

#include <iostream>
#include <cmath>
#include "kalman_filter.hpp"


class ControllerSim
{
    public:
        ControllerSim(){
            vx_ = 0;
            vy_ = 0;
            r_ = 0;

            ax_ = 0;
            ay_ = 0;
            
            input_acc_ = 0;

            delta_ = 0;
            delta_v_ = 0;
        }

        void init(double dt, bool use_GSS){
            dt_ = dt;
            est_.use_GSS_ = use_GSS;
            initialize_speed_estimator();
        }
        
        std::vector<double> get_torque_cmd(double input_acc_, double target_r) {
            get_estimation();
            get_torque_vectoring(input_acc_, target_r);
            get_traction_control();
            get_power_limitation(kMaxPower);
            return {torque_cmd_.fl_, torque_cmd_.fr_, torque_cmd_.rl_, torque_cmd_.rr_};
        }

        void set_sensors(double ax, double ay, double r, double delta, double w_fl, double w_fr, double w_rl, double w_rr, double vx, double vy){
            ax_ = ax;
            ay_ = ay;
            r_ = r;
            delta_ = delta;
            wheel_speed_.fl_ = w_fl;
            wheel_speed_.fr_ = w_fr;
            wheel_speed_.rl_ = w_rl;
            wheel_speed_.rr_ = w_rr;
            // GSS
            vx_= vx;
            vy_ = vy;
        }

        double vx_;
        double vy_;
        double r_;
        double ax_;
        double ay_;
        double delta_;
        double delta_v_;
        double input_acc_;

        struct {
            double fl_ = 0;
            double fr_ = 0;
            double rl_ = 0;
            double rr_ = 0;
        } wheel_speed_;

        struct {
            double fl_ = 0;
            double fr_ = 0;
            double rl_ = 0;
            double rr_ = 0;
        } torque_cmd_;
        
        struct {
            double fl_ = 0;
            double fr_ = 0;
            double rl_ = 0;
            double rr_ = 0;
        } tire_loads_;

        struct {
            double fl_ = 0;
            double fr_ = 0;
            double rl_ = 0;
            double rr_ = 0;
        } slip_ratio_;

        double dt_;

        void initialize_speed_estimator();
        void get_estimation();
        void get_torque_vectoring(double input_acc_, double target_r);
        void get_traction_control();
        void get_power_limitation(float PW_max);
        friend void calculate_tire_loads(ControllerSim& controller_sim);

        // PROVISIONAL: lo dejo por ahora por las diferencias entre este cálculo de sr entre ARUSSim y vdc-ART25
        friend void calculate_slip_ratio(ControllerSim& controller_sim);

        friend VectorXd calculate_steering(ControllerSim& controller_sim);
        friend void control_allocation(double target_mz, double total_fx_cmd,ControllerSim& controller_sim);
        friend double get_target_yawrate(ControllerSim& controller_sim);
        friend double get_target_mz(double actual, double target,ControllerSim& controller_sim);
        friend void calculate_torque_feedforward(double torque_tc[], ControllerSim& controller_sim);
        friend void calculate_pid_control(double torque_tc[], ControllerSim& controller_sim);
        friend void limit_power(float P_max[], ControllerSim& controller_sim);
        friend double estimate_power(ControllerSim& controller_sim);

        // Getters for vehicle parameters
        double get_kMass() { return kMass; }
        double get_kHCog() { return kHCog; }
        double get_kTrackWidth() { return kTrackWidth; }
        double get_kWheelBase() { return kWheelBase; }
        double get_kStaticLoadFront() { return kStaticLoadFront; }
        double get_kMassDistributionRear() { return kMassDistributionRear; }
        double get_kLf() { return kLf; }
        double get_kLr() { return kLr; }
        double get_kAirDensity() { return kAirDensity; }
        double get_kCLA() { return kCLA; }
        double get_kCDA() { return kCDA; }
        double get_kCOPx() { return kCOPx; }
        double get_kCOPy() { return kCOPy; }
        bool get_kTorqueVectoring() { return kTorqueVectoring; }
        double get_kTireDynRadius() { return kTireDynRadius; }
        double get_kAckermann() { return kAckermann; }
        double get_kTorqueMin() { return kTorqueMin; }
        double get_kTorqueMax() { return kTorqueMax; }
        double get_kTVKp() { return kTVKp; }
        double get_kTCKp() {return kTCKp; } 
        double get_kGearRatio() { return kGearRatio; }
        double get_kTargetSlipRatio() { return kTargetSlipRatio; }
        const auto& get_tire_loads() const { return tire_loads_; }
        const auto get_pac_param() const { return pac_param_; }

    private:
        // Vehicle parameters
        double kMass = 260.0;
        double kMassDistributionRear = 0.54;
        double kWheelBase = 1.535;
        double kTrackWidth = 1.22;
        double kHCog = 0.26;
        double kLf = kWheelBase*kMassDistributionRear;
        double kLr = kWheelBase*(1-kMassDistributionRear);
        double kIzz = 180;

        double kTireDynRadius = 0.225;
        double kTireInertia = 0.4;
        double kGearRatio = 12.48;
        double kAckermann = 0.6;

        double kRollingResistance = 100;
        double kCDA = 1.97;
        double kCLA = 4.75;
        double kCOPx = 0.4604; //longitudinal distribution (rear)
        double kCOPy = 0.517;
        double kAirDensity = 1.225;

        double kG = 9.81;

        double kStaticLoadFront = (1 - kMassDistributionRear) * kMass * kG / 2;
        double kStaticLoadRear = kMassDistributionRear * kMass * kG / 2;

        struct {
            double Dlon = 1.3976;
            double Clon = 1.9503;
            double Blon = 17.49;
        } pac_param_;

        // Estimation
        struct {
            // Estimation filter
            KalmanFilter v_filter_;

            // Previous wheelspeed
            VectorXd w_prev_ = VectorXd::Zero(4);

            // Tire slip
            VectorXd sr_ = VectorXd::Zero(4);
            VectorXd sa_ = VectorXd::Zero(4);

            // TMEasy parameters
            double dMu_x_0_ = 42.3777;
            double Mu_x_M_ = 1.3645;
            double Mu_x_S_ = 1.2999;
            double s_x_M_ = 0.0948;
            double s_x_S_ = 0.3780;

            double dMu_y_0_ = 31.3798;
            double Mu_y_M_ = 1.4862;
            double Mu_y_S_ = 1.3301;
            double s_y_M_ = 0.1105;
            double s_y_S_ = 0.1103; 

            // GSS usage
            bool use_GSS_ = false;

            // State, control vector and measurement dimensions
            int n_ = 3;
            int m_ = 2;
            int p_GSS_ = 3;
            int p_TME_ = 9;  
            
            // State, state covariance and control vector
            VectorXd x_ = VectorXd::Zero(n_);
            MatrixXd P_ = 0.1 * MatrixXd::Identity(n_,n_);
            VectorXd u_ = VectorXd::Zero(m_);

            // Model, control and model covariance matrices
            MatrixXd M_ = MatrixXd::Zero(n_,n_);
            MatrixXd B_ = MatrixXd::Zero(n_,m_);
            MatrixXd Q_ = MatrixXd::Zero(n_,n_);

            // Observation and measurement covariance matrices
            MatrixXd H_GSS_ = MatrixXd::Identity(p_GSS_,n_);
            MatrixXd R_GSS_ = MatrixXd::Zero(p_GSS_,p_GSS_);
            MatrixXd H_TME_ = MatrixXd::Zero(p_TME_,n_);
            MatrixXd R_TME_ = MatrixXd::Zero(p_TME_,p_TME_);

            // Other parameters
            double vx_threshold_ = 1.;
            double eps_ = 0.0001;
            double cf_ = 26000.;
            double cr_ = 24000.;
        } est_;

        // Torque vectoring
        double kTVKp = 800;
        bool kTorqueVectoring = true;

        // Traction control
        double kTargetSlipRatio = 0.03;
        double kTCKp = 400;

        // Power limitation
        double kMaxPower = 50000;

        double kTorqueMax = 21*12.48;
        double kTorqueMin = -21*12.48;       
        
};

