#include <cmath>
#include <algorithm>
#include <iostream>

class VehicleDynamics
{
    public:
        VehicleDynamics();
        void update_simulation(double input_delta, double input_acc, double dt);

        double x_;
        double y_;
        double yaw_;
        double vx_;
        double vy_;
        double r_;
        double ax_;
        double ay_;
        double delta_;

        struct {
            double fl_ = 0;
            double fr_ = 0;
            double rl_ = 0;
            double rr_ = 0;
        } wheel_speed_;
        
        double input_delta_;
        double input_acc_;
        double dt_;
    
    private:

        double kMass = 270.0;
        double kMassDistributionRear = 0.52;
        double kWheelBase = 1.535;
        double kTrackWidth = 1.22;
        double kHCog = 0.28;
        double kLf = kWheelBase*kMassDistributionRear;
        double kLr = kWheelBase*(1-kMassDistributionRear);
        double kIzz = 180;

        double kTireDynRadius = 0.202;
        double kTireInertia = 0.5;
        double kCamberStiffnessLat = -100;
        double kCamberStiffnessLong = 9;

        struct {
            double Dlat = -1.537;
            double Clat = 1.54;
            double Blat = 9.0;

            double Dlon = 1.38;
            double Clon = 1.5;
            double Blon = 12.4;
        } pac_param_;

        double kMinFx = -3500;
        double kMaxFx = 2000;

        double kRollingResistance = 100;
        double kCDA = 1;
        double kCLA = 3.5;
        double kCOPx = 0.5; //longitudinal distribution (rear)
        double kCOPy = kHCog;
        double kAirDensity = 1.1;

        double x_dot_{0.0}, y_dot_{0.0}, vx_dot_{0.0}, vy_dot_{0.0}, r_dot_{0.0};
        double w_fl_dot_{0.0}, w_fr_dot_{0.0}, w_rl_dot_{0.0}, w_rr_dot_{0.0};

        double kG = 9.81;

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

        void calculate_dynamics();
        void integrate_dynamics();

        double calculate_fx(Tire_force force_fl, Tire_force force_fr, Tire_force force_rl, Tire_force force_rr);

        void calculate_tire_loads();
        void calculate_tire_slip();
        Tire_force calculate_tire_forces(double slip_angle, double slip_ratio, double tire_load);
        void kinematic_correction();
        void update_torque_cmd();
};