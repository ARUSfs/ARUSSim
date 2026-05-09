#include <cmath>
#include <algorithm>
#include <iostream>
#include "arussim/csv_generator.hpp"
#include <fstream>
class VehicleDynamics
{
    public:
        VehicleDynamics();
        void update_simulation(double input_delta, std::vector<double> input_torque, double dt);
        void set_parameters(std::map<std::string, double> &params);

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
        
        struct {
            double fl_ = 0;
            double fr_ = 0;
            double rl_ = 0;
            double rr_ = 0;
        } tire_loads_;
        
        double input_delta_;
        double input_acc_;
        double dt_;

        void write_csv_row();
        void set_csv_generator(std::shared_ptr<CSVGenerator> csvGen) {
            csv_generator_vehicle_dynamics_ = csvGen;
        }
    
    private:
        
        double kMass;
        double kNsMassF;
        double kNsMassR;
        double kSMass;
        double kIzz;

        double kMassDistributionRear;
        double kSMassF;
        double kSMassR;

        double kWheelBase;
        double kTrackWidth;
        double kLf;
        double kLr;

        double kHCog;
        double kHCogNsF;
        double kHCogNsR;
        double kHRollCenterF;
        double kHRollCenterR;
        double kHRollAxis;

        double kSpringStiffnessF;
        double kSpringStiffnessR;
        double kMotionRatioF;
        double kMotionRatioR;        
        double kWheelRateF;
        double kWheelRateR;
        double kRollStiffnessF;
        double kRollStiffnessR;
        double kRollStiffness;

        double kAckermann1;
        double kAckermann2;

        double kTireDynRadius;
        double kTireInertia_F;
        double kTireInertia_R;

        double kGearRatio;

        struct {
            // Normal load
            double Fz0;

            // Longitudinal coefficients
            double D1_x;
            double D2_x;
            double Cx;
            double Bx;
            double Ex;

            // Lateral coefficients
            double D1_y;
            double D2_y;
            double Cy;
            double By;
            double Ey;

            // Shifts
            double SH;
            double SV;

            // Combined slip longitudinal
            double rB1_x;
            double rB2_x;
            double rC1_x;
            double rE1_x;

            // Combined slip lateral
            double rB1_y;
            double rB2_y;
            double rC1_y;
            double rSh;

            // Scaling / weighting factors
            double rGx1;
            double rBx;
            double rAx;
            double rCx;

            double rGy1;
            double rBy;

            // Model selector
            int comb_model;

        } pac_param_;

        double kRollingResistance;
        double kCDA;
        double kCLA;
        double kCOPx; //longitudinal distribution (rear)
        double kCOPy;
        double kAirDensity;

        double x_dot_{0.0}, y_dot_{0.0}, vx_dot_{0.0}, vy_dot_{0.0}, r_dot_{0.0};
        double w_fl_dot_{0.0}, w_fr_dot_{0.0}, w_rl_dot_{0.0}, w_rr_dot_{0.0};
        double delta_dot_{0.0}, delta_v_dot_{0.0};

        double kG;

        double kStaticLoadFront;
        double kStaticLoadRear;

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