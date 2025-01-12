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

yeast_motion::MotionState motion_state;

void init()
{
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Initializing" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

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
    }

    {
        pathfollower.reset (new yeast_motion::PathPlannerTrajectoryFollower());

        {
            std::ifstream f("settings.json");
            nlohmann::json config = nlohmann::json::parse(f);
            pathfollower->set_config(config);
        }

        {
            std::ifstream f("Path1.json");
            nlohmann::json path = nlohmann::json::parse(f);
            pathfollower->begin(path);
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

int main(int argc, char *argv[])
{
    init();
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Running" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    while (sim_time < 10.0)
    {
        print_time(sim_time + 1);
        simulate();
        sim_time += dt;
        std::this_thread::sleep_for(std::chrono::milliseconds((int64_t) (dt * 1000)));
    }


    print_time(sim_time);
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "End Pose: " << motion_state.measurement.pose.to_json().dump(2) << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    cleanup();
}