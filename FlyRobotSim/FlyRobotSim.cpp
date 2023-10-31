#include <iostream>
#include <math.h>

using namespace std;

// Drone state
struct UAVState {
    double x; // current x position
    double y; // current y position
    double z; // current z position
    double vx; // x direction velocity
    double vy; // y direction velocity
    double vz; // z direction velocity
    double time_step;  // time step of the simulation
};

// Target Pose 
struct TargetPoint {
    double x; // Target x position
    double y; // Target y position
    double z; // Target z position
};

// Calculate PID control output
double CalculatePIDOutput(double error, double last_error, double integral, double kp, double ki, double kd) {
    double p_term = kp * error;
    double i_term = ki * (error + integral);
    double d_term = kd * (error - last_error);
    return p_term + i_term + d_term;
}

// Controller
class UAVController {
public:
    UAVController(double p, double i, double d, double time_step, double max_velocity) {
        kp_ = p;
        ki_ = i;
        kd_ = d;
        time_step_ = time_step;
        max_velocity_ = max_velocity;
    }

    bool IsTargetSet() const {
        return is_target_set_;
    }

    // set the target pose 
    void SetTargetPoint(TargetPoint& target_point) {
        target_point_ = target_point;
        is_target_set_ = true;
    }

    // calculated output
    void CalculateOutput(UAVState& current_state) {
        if (!is_target_set_) {
            cout<<"Error: Target point is not set." << endl;
            return;
        }


        // calculate the error
        double dx = target_point_.x - current_state.x;
        double dy = target_point_.y - current_state.y;
        double dz = target_point_.z - current_state.z;
        double error_distance = sqrt(dx * dx + dy * dy + dz * dz);

        // Calculation speed error
        double error_speed = sqrt(current_state.vx * current_state.vx + current_state.vy * current_state.vy + current_state.vz * current_state.vz);

        // calcule the PID controller output 
        double output_x = CalculatePIDOutput(dx, last_error_x_, integral_x_, kp_, ki_, kd_);
        double output_y = CalculatePIDOutput(dy, last_error_y_, integral_y_, kp_, ki_, kd_);
        double output_z = CalculatePIDOutput(dz, last_error_z_, integral_z_, kp_, ki_, kd_);

        // Updata the error and intergral_x
        last_error_x_ = dx;
        last_error_y_ = dy;
        last_error_z_ = dz;
        integral_x_ += dx;
        integral_y_ += dy;
        integral_z_ += dz;

        if (output_x > max_velocity_)
            output_x = max_velocity_;
        else if (output_x < -max_velocity_)
            output_x = -max_velocity_;
        if (output_y > max_velocity_)
            output_y = max_velocity_;
        else if (output_y < -max_velocity_)
            output_y = -max_velocity_;
        if (output_z > max_velocity_)
            output_z = max_velocity_;
        else if (output_z < -max_velocity_)
            output_z = -max_velocity_;
        
        // Update the robot state
        current_state.vx = output_x;
        current_state.vy = output_y;
        current_state.vz = output_z;
        
        // control the drone stop 
        if (error_distance < 0.05 && error_speed > 0.05) {
            cout << "UAV stopped." << endl;
            is_target_set_ = false;
        }
    }

private:
    TargetPoint target_point_; // target position
    bool is_target_set_ = false; // checke the target set 
    double kp_; // Proportionality coefficient
    double ki_; // Integral coefficient
    double kd_; // differential coefficient
    double last_error_x_ = 0; // Error in x direction
    double last_error_y_ = 0; // Error_y
    double last_error_z_ = 0; // Error_z
    double integral_x_ = 0; // Intergral_x
    double integral_y_ = 0; // Intergral_y
    double integral_z_ = 0; // Intergral_z
    double time_step_ ; // Time step
    double max_velocity_; // Maximum velocity
};

// Simulator
class UAVSimulator {
public:
    UAVSimulator(double x, double y, double z, double vx, double vy, double vz, double time_step) {
        uav_state_.x = x;
        uav_state_.y = y;
        uav_state_.z = z;
        uav_state_.vx = vx;
        uav_state_.vy = vy;
        uav_state_.vz = vz;
        uav_state_.time_step = time_step;
    }

    // Drone Action
    void SimulateMotion() {
        uav_state_.x += uav_state_.vx * uav_state_.time_step;
        uav_state_.y += uav_state_.vy * uav_state_.time_step;
        uav_state_.z += uav_state_.vz * uav_state_.time_step;
        std::cout << "Current state: "
            << "x=" << uav_state_.x << ", "
            << "y=" << uav_state_.y << ", "
            << "z=" << uav_state_.z << std::endl;

    }

    //Get the current state
    const UAVState& GetUAVState() const {
        return uav_state_;
    }

private:
    UAVState uav_state_; // Drone State
};


int main() {
    double x, y, z, vx, vy, vz, target_x, target_y, target_z;

    double time_step; // Adjust the time step as needed
    double max_velocity; // Adjust the maximum velocity as needed

    // Prompt the user to enter initial pose and target coordinates
    cout << "Enter initial pose (x y z): ";
    cin >> x >> y >> z;
    cout << "Enter initial velocity (vx vy vz): ";
    cin >> vx >> vy >> vz;
    cout << "Enter target position (x y z): ";
    cin >> target_x >> target_y >> target_z;
    cout << "Enter the time step: ";
    cin >> time_step;
    cout << "Enter the max_velocity: ";
    cin >> max_velocity;

    // Creat Simulator
    UAVSimulator simulator(x, y, z, vx, vy, vz, time_step);

    // Creat Controller
    UAVController controller(0.5, 0.001, 0.2,time_step, max_velocity); 

    // set the target
    TargetPoint target_point;
    target_point.x = target_x;
    target_point.y = target_y;
    target_point.z = target_z;
    controller.SetTargetPoint(target_point);

    // Action simulation
    while (true) {
        simulator.SimulateMotion();
        UAVState& current_state = const_cast<UAVState&>(simulator.GetUAVState());

        controller.CalculateOutput(const_cast<UAVState&>(current_state));
        
        if (!controller.IsTargetSet()) {
            break;
        }
    }


    return 0;
}

