#include <iostream>
#include <fstream>
#include <memory>

#include <yeastcppwpilibdrivecontroller/wpilibdrivecontroller.hpp>
#include <yeastcppwpilibodometryprovider/wpilibodometryprovider.hpp>
#include <yeastcpppathplannertrajectoryfollower/pathplannertrajectoryfollower.hpp>


std::shared_ptr<yeast_motion::WPILibDriveController> controller;
std::shared_ptr<yeast_motion::WPILibOdometryProvider> odometry;
std::shared_ptr<yeast_motion::PathPlannerTrajectoryFollower> pathfollower;

float sim_time = 0;
float dt = 0.01;

yeast_motion::OdometrySample pose;

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

    yeast_motion::MotionCommand command;
    command.velocity_valid = true;
    command.velocity.x = 3;

    controller->update_motor_status(module_statuses);
    controller->drive(command);
    auto module_commands = controller->get_command();

    yeast_motion::Rotation2D drive_theta;
    drive_theta.theta = 0;

    pose = odometry->update(module_statuses, drive_theta);
}

void print_time (float sim_time)
{
    static int whole_time = 0;
    if ((int) sim_time > whole_time)
    {
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Seconds: " << whole_time << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        whole_time = (int) sim_time;
    }
}

int main(int argc, char *argv[])
{
    init();
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "Runnin" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    while (sim_time < 10.0)
    {
        print_time(sim_time);
        simulate();
        sim_time += dt;
    }

    print_time(sim_time);
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "End Pose: " << pose.to_json().dump(2) << std::endl;
    std::cout << "----------------------------------------" << std::endl;

}