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

            if(vx_ > 0.1) pid_init_ = true;

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

        struct {
            double fl_ = 0;
            double fr_ = 0;
            double rl_ = 0;
            double rr_ = 0;
        } slip_angle_;

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
        friend double calculate_tire_force(double slip_angle, double slip_ratio, double fz, ControllerSim& controller_sim);
        friend void limit_power(float P_max[], ControllerSim& controller_sim);
        friend double estimate_power(ControllerSim& controller_sim);

        // Getters for vehicle parameters
        double get_kMass() { return kMass; }
        double get_kHCog() { return kHCog; }
        double get_kTrackWidth() { return kTrackWidth; }
        double get_kWheelBase() { return kWheelBase; }
        double get_kStaticLoadFront() { return kStaticLoadFront; }
        double get_kStaticLoadRear() { return kStaticLoadRear; }
        double get_kMassDistributionRear() { return kMassDistributionRear; }
        double get_kLf() { return kLf; }
        double get_kLr() { return kLr; }
        double get_kAirDensity() { return kAirDensity; }
        double get_kCLA() { return kCLA; }
        double get_kCDA() { return kCDA; }
        double get_kCOPx() { return kCOPx; }
        double get_kCOPy() { return kCOPy; }
        bool get_kTorqueVectoring() { return kTorqueVectoring; }
        bool get_RWD() { return kRWD; }
        double get_kTireDynRadius() { return kTireDynRadius; }
        double get_kTireInertia() { return kTireInertia; }
        double get_kAckermann1() { return kAckermann1; }
        double get_kAckermann2() { return kAckermann2; }
        double get_kTorqueMin() { return kTorqueMin; }
        double get_kTorqueMax() { return kTorqueMax; }
        double get_kTVKp() { return kTVKp; }
        double get_kTCKp() {return kTCKp; } 
        double get_kTCKi() {return kTCKi; }
        double get_kTCKd() {return kTCKd; }
        bool get_kTractionControl() { return kTractionControl; }
        double get_kGearRatio() { return kGearRatio; }
        double get_kTargetSlipRatio() { return kTargetSlipRatio; }
        const auto& get_tire_loads() const { return tire_loads_; }
        const auto get_pac_param() const { return pac_param_; }

    private:
        // Vehicle parameters
        double kMass = 264.07;  // Mass of the car + LiDAR
        double kMassDistributionRear = 0.5689;
        double kWheelBase = 1.535;
        double kTrackWidth = 1.22;
        double kHCog = 0.27;
        double kLf = kWheelBase*kMassDistributionRear;
        double kLr = kWheelBase*(1-kMassDistributionRear);
        double kIzz = 190;

        double kTireDynRadius = 0.23;
        double kTireInertia = 0.4;
        double kGearRatio = 12.48;

        double kAckermann1 = 0.1175;
        double kAckermann2 = 0.9724;

        double kRollingResistance = 100;
        double kCDA = 1.5;
        double kCLA = 3.6;
        double kCOPx = 0.5795; //longitudinal distribution (rear)
        double kCOPy = 0.31;
        double kAirDensity = 1.225;

        double kG = 9.81;

        double kStaticLoadFront = (1 - kMassDistributionRear) * kMass * kG / 2;
        double kStaticLoadRear = kMassDistributionRear * kMass * kG / 2;

        struct {
            double Dlat = -1.3323;
            double Clat = 1.7230;
            double Blat = 12.7;
            double Elat = 0.4035;

            double Dlon = 1.1976;
            double Clon = 1.9503;
            double Blon = 17.49;
            double Elon = 0.999;

            double Gx1 = 25000;
            double bx = 0.2367;
            double a = 937330;
            double c = 0.1689;
            double Gy1 = 38.21;
            double by = 0.5365;
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
            bool use_GSS_ = true;

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
        double kTVKp = 600;
        bool kTorqueVectoring = true;
        bool kRWD = true;

        // Traction control
        double kTargetSlipRatio = 0.1;
        double kTCKp = 30;
        double kTCKi = 1.5;
        double kTCKd = 0;
        double kTractionControl = false;
        bool pid_init_ = false;

        // Power limitation
        double kMaxPower = 50000;

        double kTorqueMax = 21*kGearRatio;
        double kTorqueMin = -21*kGearRatio;       
        
};

