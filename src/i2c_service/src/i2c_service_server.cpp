#include <memory>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <rclcpp/rclcpp.hpp>
#include "i2c_service/srv/i2c_service.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class I2CServiceServer : public rclcpp::Node {
public:
    I2CServiceServer() : Node("i2c_service_server") {
        service_ = this->create_service<your_package_name::srv::I2CService>(
            "i2c_service", std::bind(&I2CServiceServer::handle_request, this, _1, _2));
    }

private:
    rclcpp::Service<your_package_name::srv::I2CService>::SharedPtr service_;
    const char *i2c_device_ = "/dev/i2c-1";

    void handle_request(
        const std::shared_ptr<your_package_name::srv::I2CService::Request> request,
        std::shared_ptr<your_package_name::srv::I2CService::Response> response)
    {
        int file = open(i2c_device_, O_RDWR);
        if (file < 0) {
            response->success = false;
            return;
        }

        if (ioctl(file, I2C_SLAVE, request->address) < 0) {
            response->success = false;
            close(file);
            return;
        }

        if (request->read) {
            uint8_t reg = request->register_;
            if (write(file, &reg, 1) != 1) {
                response->success = false;
                close(file);
                return;
            }

            std::vector<uint8_t> buffer(request->read_length);
            if (read(file, buffer.data(), request->read_length) != request->read_length) {
                response->success = false;
            } else {
                response->success = true;
                response->response = buffer;
            }
        } else {
            std::vector<uint8_t> out_data;
            out_data.push_back(request->register_);
            out_data.insert(out_data.end(), request->data.begin(), request->data.end());
            if (write(file, out_data.data(), out_data.size()) != static_cast<ssize_t>(out_data.size())) {
                response->success = false;
            } else {
                response->success = true;
            }
        }

        close(file);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<I2CServiceServer>());
    rclcpp::shutdown();
    return 0;
}
