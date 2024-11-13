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

        double kMinFx = -3500;
        double kMaxFx = 2000;

        double kRollingResistance = 100;
        double kCDA = 1;
        double kAirDensity = 1.1;

        double kCamberStiffness = -100;

        double x_dot_{0.0}, y_dot_{0.0}, vx_dot_{0.0}, vy_dot_{0.0}, r_dot_{0.0};

        double kG = 9.81;

        struct {
            double alpha_fl_ = 0;
            double alpha_fr_ = 0;
            double alpha_rl_ = 0;
            double alpha_rr_ = 0;
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

        double calculate_fx();

        void calculate_tire_loads();
        void calculate_tire_slip();
        Tire_force calculate_tire_forces(double slip_angle, double tire_load);
        void kinematic_correction();
        void calculate_wheel_speed();
};