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
        void update_simulation(double input_delta, std::vector<double>, double dt);
        void set_torque_vectoring(bool value);

        double x_;
        double y_;
        double yaw_;
        double vx_;
        double vy_;
        double r_;
        double ax_;
        double ay_;
        double delta_;
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
        
        double input_delta_;
        double input_acc_;
        double dt_;

        void write_csv_row();
        void set_csv_generator(std::shared_ptr<CSVGenerator> csvGen) {
            csv_generator_vehicle_dynamics_ = csvGen;
        }
    
    private:

        double kMass = 270.0;
        double kMassDistributionRear = 0.55;
        double kWheelBase = 1.533;
        double kTrackWidth = 1.22;
        double kHCog = 0.28;
        double kLf = kWheelBase*kMassDistributionRear;
        double kLr = kWheelBase*(1-kMassDistributionRear);
        double kIzz = 180;

        double kTireDynRadius = 0.202;
        double kTireInertia = 0.5;

        struct {
            double Dlat = -1.537;
            double Clat = 1.54;
            double Blat = 9.0;

            double Dlon = 1.38;
            double Clon = 1.5;
            double Blon = 12.4;

            double kAlphaP = 0.1809;
            double kLambdaP = 0.1397;
        } pac_param_;

        double kRollingResistance = 100;
        double kCDA = 1;
        double kCLA = 3.5;
        double kCOPx = 0.5; //longitudinal distribution (rear)
        double kCOPy = kHCog;
        double kAirDensity = 1.1;

        double x_dot_{0.0}, y_dot_{0.0}, vx_dot_{0.0}, vy_dot_{0.0}, r_dot_{0.0};
        double w_fl_dot_{0.0}, w_fr_dot_{0.0}, w_rl_dot_{0.0}, w_rr_dot_{0.0};
        double delta_dot_{0.0}, delta_v_dot_{0.0};

        double kG = 9.81;

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

        // Control parameters
        double kTVKp = 1000;
        bool kTorqueVectoring = true;
        double kTorqueMax = 252;
        double kTorqueMin = -252;

        // Steering dynamics
        double kCoefDelta = 306.3;
        double kCoefV = 25.69;
        double kCoefInput = 307;
        double kSteeringAMax = 125;
        double kSteeringVMax = 125;

        void calculate_dynamics();
        void integrate_dynamics();

        double calculate_fx(Tire_force force_fl, Tire_force force_fr, Tire_force force_rl, Tire_force force_rr);

        void calculate_tire_loads();
        void calculate_tire_slip();
        Tire_force calculate_tire_forces(double slip_angle, double slip_ratio, double tire_load);
        void kinematic_correction();
        void update_torque_cmd();

        std::shared_ptr<CSVGenerator> csv_generator_vehicle_dynamics_;
};