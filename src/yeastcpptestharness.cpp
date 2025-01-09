#include <iostream>
#include <memory>

#include <yeastcppwpilibdrivecontroller/wpilibdrivecontroller.hpp>

nlohmann::json object;

std::shared_ptr<yeast_motion::WPILibDriveController> controller (new yeast_motion::WPILibDriveController(object));

int main(int argc, char *argv[])
{
    std::cout << "Runnin" << std::endl;
}