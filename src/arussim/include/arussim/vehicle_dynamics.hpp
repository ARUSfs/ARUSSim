#include <cmath>
#include <algorithm>

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
        
        double input_delta_;
        double input_acc_;
        double dt_;
    
    private:

        double kMass = 270.0;
        double kMassDistributionRear = 0.52;
        double kWheelBase = 1.535;
        double kLf = kWheelBase*kMassDistributionRear;
        double kLr = kWheelBase*(1-kMassDistributionRear);
        double kIzz = 180;

        double kMinFx = -3500;
        double kMaxFx = 2000;

        double kRollingResistance = 100;
        double kCDA = 1;
        double kAirDensity = 1.1;

        double kCamberStiffness = -100000;

        double x_dot_{0.0}, y_dot_{0.0}, vx_dot_{0.0}, vy_dot_{0.0}, r_dot_{0.0};

        struct {
            double alpha_front_;
            double alpha_rear_;
        } tire_slip_;

        void calculate_dynamics();
        void integrate_dynamics();
        double calculate_fx();
        void calculate_tire_slip();
        void calculate_tire_forces(double &fy_front, double &fy_rear);
};