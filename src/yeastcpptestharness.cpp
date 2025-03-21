#include <iostream>
#include <memory>
#include <fstream>
#include <thread>
#include <chrono>

#include <yeastcppwpilibdrivecontroller/wpilibdrivecontroller.hpp>
#include <yeastcppwpilibodometryprovider/wpilibodometryprovider.hpp>
#include <yeastcpppathplannertrajectoryfollower/pathplannertrajectoryfollower.hpp>

#include <ntcore/networktables/NetworkTableInstance.h>

std::shared_ptr<yeast_motion::WPILibDriveController> controller;
std::shared_ptr<yeast_motion::WPILibOdometryProvider> odometry;
std::shared_ptr<yeast_motion::PathPlannerTrajectoryFollower> pathfollower;

float sim_time = 0;
float dt = 0.01;
yeast_motion::OdometrySample start_pose;

yeast_motion::MotionState motion_state;

void init()
{
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Initializing" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    start_pose.pose.translation.x = 3.738521252012985;
    start_pose.pose.translation.y = 5.679469373296802;
    start_pose.pose.rotation.theta = -60.0 * M_PI / 180.0;
    start_pose.pose_valid = true;
    start_pose.velocity.x = 1.5;
    start_pose.velocity_valid = true;

    nlohmann::json controller_config;
    {
        controller_config["MotorConfig"] = nlohmann::json::array();

        std::vector<std::pair<float, float>> wheel_positions = { {.3,.3}, {.3,-.3}, {-.3,.3}, {-.3,-.3} };
        {
            int i = 0;
            for (auto& [x, y] : wheel_positions)
            {
                controller_config["MotorConfig"].push_back(nlohmann::json::object());
                controller_config["MotorConfig"][i]["x"] = x;
                controller_config["MotorConfig"][i]["y"] = y;
                i++;
            }
        }

        controller.reset (new yeast_motion::WPILibDriveController(controller_config));
    }

    {
        nlohmann::json odometry_config;
        odometry_config = controller_config;
        odometry.reset (new yeast_motion::WPILibOdometryProvider(odometry_config));
        odometry->reset (start_pose);
        motion_state.measurement = odometry->get();
        std::cout << "Motion State: " << std::endl << motion_state.to_json().dump(2) << std::endl;
    }

    {
        pathfollower.reset (new yeast_motion::PathPlannerTrajectoryFollower());

        {
            std::ifstream f("settings.json");
            nlohmann::json config = nlohmann::json::parse(f);
            pathfollower->set_config(config);
        }

        {
            std::ifstream f("EvilPath.json");
            nlohmann::json path = nlohmann::json::parse(f);
            pathfollower->begin(path, motion_state);
        }
    }
}

void simulate()
{
    static std::vector<yeast_motion::SwerveModuleStatus> module_statuses;

    if (module_statuses.empty())
    {
        for (int i = 0; i < 4; i++)
        {
            yeast_motion::SwerveModuleStatus module_status;
            module_status.position = 0;
            module_status.speed = 0;
            module_status.theta = 0;
            module_statuses.push_back(module_status);
        }
    }

    auto command = pathfollower->follow(motion_state);
    auto follower_status = pathfollower->status();

    if (follower_status.finished)
    {
        command.velocity.x = 0;
        command.velocity.y = 0;
        command.velocity.omega = 0;
    }

    controller->update_motor_status(module_statuses);
    controller->drive(command);
    auto module_commands = controller->get_command();

    for (size_t i = 0; i < 4; i++)
    {
        module_statuses[i].speed = module_commands[i].speed;
        module_statuses[i].position += module_commands[i].speed * dt;
        module_statuses[i].theta = module_commands[i].theta;
    }

    yeast_motion::Rotation2D drive_theta;
    drive_theta.theta = motion_state.measurement.pose.rotation.theta + command.velocity.omega * dt;

    motion_state.measurement = odometry->update(module_statuses, drive_theta);
}

void print_time (float sim_time)
{
    static int whole_time = 0;
    if ((int) sim_time > whole_time)
    {
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Seconds: " << whole_time << std::endl;
        std::cout << "Pose: " << motion_state.measurement.pose.to_json().dump(2) << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        whole_time = (int) sim_time;
    }
}

void cleanup()
{
    pathfollower.reset();
    controller.reset();
    odometry.reset();
}

// void add_absolute_pose()
// {
//     static int whole_time = 0;
//     if ((int) sim_time > whole_time)
//     {
//         yeast_motion::AbsolutePoseEstimate absolute_pose;
//         absolute_pose.pose.rotation.theta = 0;
//         absolute_pose.pose.translation.x = 10;
//         absolute_pose.pose.translation.y = 2;
//         absolute_pose.timestamp = wpi::math::MathSharedStore::GetTimestamp().value();

//         odometry->provide_absolute_position_estimate(absolute_pose);

//         std::cout << "Fusing absolute pose" << std::endl;
//         whole_time = (int) sim_time;
//     }
// }

int main(int argc, char *argv[])
{
    init();
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Running" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    print_time(sim_time + 1);

    while (!pathfollower->status().finished)
    {
        simulate();
        sim_time += dt;
        std::this_thread::sleep_for(std::chrono::milliseconds((int64_t) (dt * 1000)));
        // if (sim_time > 10.0)
        // {
        //     add_absolute_pose();
        // }
        print_time(sim_time + 1);
        
    }


    print_time(sim_time);
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Total Time: " << sim_time << std::endl;
    std::cout << "End Pose: " << motion_state.measurement.pose.to_json().dump(2) << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    cleanup();
}