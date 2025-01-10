#include <iostream>
#include <memory>

#include <yeastcppwpilibdrivecontroller/wpilibdrivecontroller.hpp>
#include <yeastcppwpilibodometryprovider/wpilibodometryprovider.hpp>


std::shared_ptr<yeast_motion::WPILibDriveController> controller;
std::shared_ptr<yeast_motion::WPILibOdometryProvider> odometry;

float sim_time = 0;
float dt = 0.1;

yeast_motion::OdometrySample pose;

void init()
{
    std::cout << "Initializing" << std::endl;
    nlohmann::json controller_config;
    nlohmann::json odometry_config;

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

    odometry_config = controller_config;

    controller.reset (new yeast_motion::WPILibDriveController(controller_config));
    odometry.reset (new yeast_motion::WPILibOdometryProvider(odometry_config));
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

    std::cout << "ChassisCommand: " << command.to_json() << std::endl;

    controller->update_motor_status(module_statuses);
    controller->drive(command);
    auto module_commands = controller->get_command();

    int print_line = 0;

    for (auto command : module_commands)
    {
        std::cout << "Command:" << print_line << command.to_json() << std::endl;
        print_line ++;
    }

    print_line = 0;
    for (int i = 0; i < module_statuses.size(); i++)
    {
        module_statuses[i].position += module_commands[i].speed * dt;
        module_statuses[i].speed = module_commands[i].speed;
        module_statuses[i].theta = module_commands[i].theta;
        std::cout << "Status:" << print_line << module_statuses[i].to_json() << std::endl;
        print_line ++;
    }


    yeast_motion::Rotation2D drive_theta;
    drive_theta.theta = 0;

    pose = odometry->update(module_statuses, drive_theta);
}

int main(int argc, char *argv[])
{
    init();
    std::cout << "Runnin" << std::endl;

    while (sim_time < 1.0)
    {
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "Step: " << sim_time << std::endl;

        simulate();
        sim_time += dt;
        std::cout << "----------------------------------------" << std::endl;

    }

    std::cout << "----------------------------------------" << std::endl;
    std::cout << "End Pose: " << pose.to_json().dump(2) << std::endl;
    std::cout << "----------------------------------------" << std::endl;

}