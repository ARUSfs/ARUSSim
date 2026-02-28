#include <cmath>
#include <algorithm>
#include <iostream>
#include "arussim/csv_generator.hpp"
#include <fstream>
#include "controller_sim/controller_sim.hpp"
class VehicleDynamics
{
    public:
        VehicleDynamics();
        void update_simulation(double input_delta, std::vector<double> input_torque, double dt);

        double x_;
        double y_;
        double yaw_;
        double vx_;
        double vy_;
        double r_;
        double ax_;
        double ay_;
        double delta_;
        double delta_fl_;
        double delta_fr_;
        double delta_v_;

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
            double alpha_fl_ = 0;
            double alpha_fr_ = 0;
            double alpha_rl_ = 0;
            double alpha_rr_ = 0;

            double lambda_fl_ = 0;
            double lambda_fr_ = 0;
            double lambda_rl_ = 0;
            double lambda_rr_ = 0;
        } tire_slip_;
        
        double input_delta_;
        double input_acc_;
        double dt_;

        void write_csv_row();
        void set_csv_generator(std::shared_ptr<CSVGenerator> csvGen) {
            csv_generator_vehicle_dynamics_ = csvGen;
        }
    
    private:

        double kMass = 264.07;  // Mass of the car + LiDAR
        double kNsMassF = 17.0;
        double kNsMassR = 24.302;
        double kSMass = kMass - kNsMassF - kNsMassR;
        double kIzz = 190;

        double kMassDistributionRear = 0.5689;
        double kSMassF = kSMass * (1-kMassDistributionRear);
        double kSMassR = kSMass * kMassDistributionRear;

        double kWheelBase = 1.535;
        double kTrackWidth = 1.22;
        double kLf = kWheelBase*kMassDistributionRear;
        double kLr = kWheelBase*(1-kMassDistributionRear);

        double kHCog = 0.27;
        double kHCogNsF = 0.23;
        double kHCogNsR = 0.23;
        double kHRollCenterF = 0.033;
        double kHRollCenterR = 0.097;
        double kHRollAxis = kHRollCenterF + (kHRollCenterR - kHRollCenterF) * kLf / kWheelBase;

        double kWheelRateF = 105000 / std::pow(1.02,2); // spring_stiffness (N/mm) / motion_ratio ^ 2
        double kWheelRateR = 105000 / std::pow(1.02,2);
        double kRollStiffnessF = 0.5 * std::pow(kTrackWidth,2) * 0.01745 * kWheelRateF;
        double kRollStiffnessR = 0.5 * std::pow(kTrackWidth,2) * 0.01745 * kWheelRateR;
        double kRollStiffness = kRollStiffnessF + kRollStiffnessR;

        double kAckermann1 = 0.1175;
        double kAckermann2 = 0.9724;

        double kTireDynRadius = 0.23;
        double kTireInertia = 0.4;

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

        double kRollingResistance = 100;
        double kCDA = 1.5;
        double kCLA = 3.6;
        double kCOPx = 0.5795; //longitudinal distribution (rear)
        double kCOPy = 0.31;
        double kAirDensity = 1.225;

        double x_dot_{0.0}, y_dot_{0.0}, vx_dot_{0.0}, vy_dot_{0.0}, r_dot_{0.0};
        double w_fl_dot_{0.0}, w_fr_dot_{0.0}, w_rl_dot_{0.0}, w_rr_dot_{0.0};
        double delta_dot_{0.0}, delta_v_dot_{0.0};

        double kG = 9.81;

        

        double kStaticLoadFront = (1 - kMassDistributionRear) * kMass * kG / 2;
        double kStaticLoadRear = kMassDistributionRear * kMass * kG / 2;

        struct {
            double fl_ = 0;
            double fr_ = 0;
            double rl_ = 0;
            double rr_ = 0;
        } tire_loads_;

        struct Tire_force {
            double fy;
            double fx;
        };

        Tire_force force;

        // Steering dynamics
        double kCoefDelta = 306.3;
        double kCoefV = 25.69;
        double kCoefInput = 307;
        double kSteeringAMax = 3.0;
        double kSteeringVMax = 2.3;

        void calculate_dynamics();
        void integrate_dynamics();

        double calculate_fx(Tire_force force_fl, Tire_force force_fr, Tire_force force_rl, Tire_force force_rr);

        void calculate_tire_loads();
        void calculate_ackermann();
        void calculate_tire_slip();
        Tire_force calculate_tire_forces(double slip_angle, double slip_ratio, double tire_load);
        void kinematic_correction();

        std::shared_ptr<CSVGenerator> csv_generator_vehicle_dynamics_;
};