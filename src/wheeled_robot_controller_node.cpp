#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

#include <libi2c/i2c.h>

using std::placeholders::_1;

typedef union float_t {
    int32_t data_i;
    float data_f;
} float_t;

class FloatManip {
    public:
        FloatManip() {
            data_.data_f = 0.0;
        }

        float read_from_buf(uint8_t* bufptr, int idx) {
            data_.data_i = 0.0;
            data_.data_i |= ((uint32_t)bufptr[idx + 0] << 24);
            data_.data_i |= ((uint32_t)bufptr[idx + 1] << 16);
            data_.data_i |= ((uint32_t)bufptr[idx + 2] <<  8);
            data_.data_i |= ((uint32_t)bufptr[idx + 3] <<  0);
            return data_.data_f;
        }
        
        void write_to_buf(uint8_t* bufptr, int idx, float data) {
            data_.data_f = data;
            bufptr[idx + 0] = (uint8_t)((data_.data_i & 0xFF000000) >> 24);
            bufptr[idx + 1] = (uint8_t)((data_.data_i & 0x00FF0000) >> 16);
            bufptr[idx + 2] = (uint8_t)((data_.data_i & 0x0000FF00) >>  8);
            bufptr[idx + 3] = (uint8_t)((data_.data_i & 0x000000FF) >>  0);
        }
    private:
        float_t data_;
};

class WheeledRobotController : public rclcpp::Node {
    public:
        WheeledRobotController() :
            Node("wheeled_robot_controller"),
            target_linear_velocity_(0.0),
            target_angular_velocity_(0.0)
        {
            int bus;

            /* Open i2c bus /dev/i2c-0 */
            if ((bus = i2c_open("/dev/i2c-1")) == -1) {
                RCLCPP_ERROR(this->get_logger(), "i2c error failed");
            }
            memset(&device, 0, sizeof(device));

            /* 24C04 */
            device.bus = bus;	/* Bus 0 */
            device.addr = 0x08;	/* Slave address is 0x50, 7-bit */
            device.iaddr_bytes = 1;	/* Device internal address is 1 byte */
            device.page_bytes = 32; /* Device are capable of 16 bytes per page */
            RCLCPP_INFO(this->get_logger(), "i2c initialization done.");
            char i2c_dev_desc[128];
            RCLCPP_INFO(this->get_logger(), "%s\n", i2c_get_device_desc(&device, i2c_dev_desc, sizeof(i2c_dev_desc)));

            subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10,
                std::bind(&WheeledRobotController::topic_callback, this, _1)
            );

            using namespace std::chrono_literals;
            timer_ = this->create_wall_timer(
                100ms,
                std::bind(&WheeledRobotController::timer_callback, this)
            );
        }

        ~WheeledRobotController()
        {
            i2c_close(device.bus);
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            //RCLCPP_INFO(this->get_logger(), "linear: '%f', angular: '%f'", msg->linear.x, msg->angular.z);
            target_linear_velocity_ = msg->linear.x;
            target_angular_velocity_ = msg->angular.z;

            auto saturate = [](double v, double mi, double ma) {
                return std::min(std::max(mi, v), ma);
            };

            saturate(target_linear_velocity_, 0.0, 1.0);
            saturate(target_angular_velocity_, -1.0, 1.0);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "timer_callback: ");

            // send target linear/angular velocity
            ssize_t ret = 0;
            unsigned char buf[256];
            size_t buf_size = 8;
            
            floatManip_.write_to_buf(buf, 0, target_linear_velocity_);
            floatManip_.write_to_buf(buf, 4, target_angular_velocity_);
            RCLCPP_INFO(this->get_logger(), "target>");
            RCLCPP_INFO(this->get_logger(), "vel_lin: %f", target_linear_velocity_);
            RCLCPP_INFO(this->get_logger(), "vel_ang: %f", target_angular_velocity_);
            
            ret = i2c_write(&device, 0x0, buf, buf_size);
            if (ret == -1 || (size_t)ret != buf_size) {
                fprintf(stderr, "Write i2c error!\n");
                exit(-4);
            }

            // read data from I2C
            uint8_t float_buf_[8];
            ret = i2c_read(&device, 0x0, float_buf_, 8);
            if (ret == -1) {
                fprintf(stderr, "Read i2c error!\n");
                exit(-4);
            }
            
            // decode data from buffer
            double vel_lin = floatManip_.read_from_buf(float_buf_, 0);
            double vel_ang = floatManip_.read_from_buf(float_buf_, 4);

            RCLCPP_INFO(this->get_logger(), "current>");
            RCLCPP_INFO(this->get_logger(), "vel_lin: %f", vel_lin);
            RCLCPP_INFO(this->get_logger(), "vel_ang: %f", vel_ang);
        }

        I2CDevice device;
        FloatManip floatManip_;
        double target_linear_velocity_, target_angular_velocity_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<WheeledRobotController>());

    rclcpp::shutdown();
    
    return 0;
}