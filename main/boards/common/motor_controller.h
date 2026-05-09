#ifndef __MOTOR_CONTROLLER_H__
#define __MOTOR_CONTROLLER_H__

#include "mcp_server.h"
#include <driver/gpio.h>
#include <driver/ledc.h>

#define MOTOR_LEDC_TIMER      LEDC_TIMER_2
#define MOTOR_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_FREQ_HZ    1000
#define MOTOR_LEDC_RESOLUTION LEDC_TIMER_8_BIT
#define MOTOR_LEDC_MAX_DUTY   255

class MotorController {
private:
    gpio_num_t in1_, in2_, in3_, in4_;
    ledc_channel_t ch_a_, ch_b_;
    int speed_a_ = 0;
    int speed_b_ = 0;

    void ConfigGpio(gpio_num_t pin) {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&cfg));
        gpio_set_level(pin, 0);
    }

    void ConfigPwm(ledc_channel_t ch, gpio_num_t en_pin) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num   = en_pin,
            .speed_mode = MOTOR_LEDC_MODE,
            .channel    = ch,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = MOTOR_LEDC_TIMER,
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    }

    void SetMotor(gpio_num_t in1, gpio_num_t in2, ledc_channel_t ch, int speed) {
        int abs_speed = speed < 0 ? -speed : speed;
        if (speed > 0) {
            gpio_set_level(in1, 1);
            gpio_set_level(in2, 0);
        } else if (speed < 0) {
            gpio_set_level(in1, 0);
            gpio_set_level(in2, 1);
        } else {
            gpio_set_level(in1, 0);
            gpio_set_level(in2, 0);
        }
        uint32_t duty = (uint32_t)(abs_speed * MOTOR_LEDC_MAX_DUTY / 100);
        ledc_set_duty(MOTOR_LEDC_MODE, ch, duty);
        ledc_update_duty(MOTOR_LEDC_MODE, ch);
    }

public:
    MotorController(gpio_num_t in1, gpio_num_t in2, gpio_num_t ena,
                    gpio_num_t in3, gpio_num_t in4, gpio_num_t enb)
        : in1_(in1), in2_(in2), in3_(in3), in4_(in4),
          ch_a_(LEDC_CHANNEL_1), ch_b_(LEDC_CHANNEL_2)
    {
        ledc_timer_config_t timer_cfg = {
            .speed_mode      = MOTOR_LEDC_MODE,
            .duty_resolution = MOTOR_LEDC_RESOLUTION,
            .timer_num       = MOTOR_LEDC_TIMER,
            .freq_hz         = MOTOR_LEDC_FREQ_HZ,
            .clk_cfg         = LEDC_AUTO_CLK,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

        ConfigGpio(in1_); ConfigGpio(in2_);
        ConfigGpio(in3_); ConfigGpio(in4_);
        ConfigPwm(ch_a_, ena);
        ConfigPwm(ch_b_, enb);

        auto& mcp = McpServer::GetInstance();

        mcp.AddTool("self.motor_a.set_speed",
            "Set motor A speed. Positive = forward, negative = reverse, 0 = stop. Range: -100 to 100.",
            PropertyList({Property("speed", kPropertyTypeInteger, 0, -100, 100)}),
            [this](const PropertyList& props) -> ReturnValue {
                speed_a_ = props["speed"].value<int>();
                SetMotor(in1_, in2_, ch_a_, speed_a_);
                return true;
            });

        mcp.AddTool("self.motor_b.set_speed",
            "Set motor B speed. Positive = forward, negative = reverse, 0 = stop. Range: -100 to 100.",
            PropertyList({Property("speed", kPropertyTypeInteger, 0, -100, 100)}),
            [this](const PropertyList& props) -> ReturnValue {
                speed_b_ = props["speed"].value<int>();
                SetMotor(in3_, in4_, ch_b_, speed_b_);
                return true;
            });

        mcp.AddTool("self.motor_a.stop", "Stop motor A immediately",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                speed_a_ = 0;
                SetMotor(in1_, in2_, ch_a_, 0);
                return true;
            });

        mcp.AddTool("self.motor_b.stop", "Stop motor B immediately",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                speed_b_ = 0;
                SetMotor(in3_, in4_, ch_b_, 0);
                return true;
            });

        mcp.AddTool("self.motor.get_state",
            "Get current speed of both motors (-100 to 100)",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return std::string("{\"motor_a\": ") + std::to_string(speed_a_) +
                       ", \"motor_b\": " + std::to_string(speed_b_) + "}";
            });
    }
};

#endif // __MOTOR_CONTROLLER_H__
