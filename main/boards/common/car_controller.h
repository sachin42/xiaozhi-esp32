#ifndef __CAR_CONTROLLER_H__
#define __CAR_CONTROLLER_H__

#include "mcp_server.h"
#include <driver/gpio.h>
#include <driver/ledc.h>

#define CAR_LEDC_TIMER      LEDC_TIMER_2
#define CAR_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define CAR_LEDC_FREQ_HZ    1000
#define CAR_LEDC_RESOLUTION LEDC_TIMER_8_BIT
#define CAR_LEDC_MAX_DUTY   255

// Motor A = left wheel, Motor B = right wheel
class CarController {
private:
    gpio_num_t l_in1_, l_in2_;
    gpio_num_t r_in1_, r_in2_;
    ledc_channel_t ch_l_, ch_r_;
    int speed_ = 60;

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
            .speed_mode = CAR_LEDC_MODE,
            .channel    = ch,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = CAR_LEDC_TIMER,
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
        uint32_t duty = (uint32_t)(abs_speed * CAR_LEDC_MAX_DUTY / 100);
        ledc_set_duty(CAR_LEDC_MODE, ch, duty);
        ledc_update_duty(CAR_LEDC_MODE, ch);
    }

    void SetLeft(int speed)  { SetMotor(l_in1_, l_in2_, ch_l_, speed); }
    void SetRight(int speed) { SetMotor(r_in1_, r_in2_, ch_r_, speed); }

    // Apply speed override: if spd>0 update speed_, else use current speed_
    int ResolveSpeed(int spd) {
        if (spd > 0) speed_ = spd;
        return speed_;
    }

public:
    CarController(gpio_num_t in1, gpio_num_t in2, gpio_num_t ena,
                  gpio_num_t in3, gpio_num_t in4, gpio_num_t enb)
        : l_in1_(in1), l_in2_(in2), r_in1_(in3), r_in2_(in4),
          ch_l_(LEDC_CHANNEL_1), ch_r_(LEDC_CHANNEL_2)
    {
        ledc_timer_config_t timer_cfg = {
            .speed_mode      = CAR_LEDC_MODE,
            .duty_resolution = CAR_LEDC_RESOLUTION,
            .timer_num       = CAR_LEDC_TIMER,
            .freq_hz         = CAR_LEDC_FREQ_HZ,
            .clk_cfg         = LEDC_AUTO_CLK,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

        ConfigGpio(l_in1_); ConfigGpio(l_in2_);
        ConfigGpio(r_in1_); ConfigGpio(r_in2_);
        ConfigPwm(ch_l_, ena);
        ConfigPwm(ch_r_, enb);

        auto& mcp = McpServer::GetInstance();

        mcp.AddTool("self.car.move_forward",
            "REQUIRED: Call this tool to physically move the car forward. "
            "Optional speed 1-100 overrides current speed. 0 means use current speed. "
            "When this tool is called, acknowledge in 5 words or less. No elaboration.",
            PropertyList({Property("speed", kPropertyTypeInteger, 0, 0, 100)}),
            [this](const PropertyList& props) -> ReturnValue {
                int s = ResolveSpeed(props["speed"].value<int>());
                SetLeft(s);
                SetRight(s);
                return true;
            });

        mcp.AddTool("self.car.move_backward",
            "REQUIRED: Call this tool to physically move the car backward. "
            "Optional speed 1-100 overrides current speed. 0 means use current speed. "
            "When this tool is called, acknowledge in 5 words or less. No elaboration.",
            PropertyList({Property("speed", kPropertyTypeInteger, 0, 0, 100)}),
            [this](const PropertyList& props) -> ReturnValue {
                int s = ResolveSpeed(props["speed"].value<int>());
                SetLeft(-s);
                SetRight(-s);
                return true;
            });

        mcp.AddTool("self.car.turn_left",
            "REQUIRED: Call this tool to physically turn the car left (tank turn). "
            "Optional speed 1-100 overrides current speed. 0 means use current speed. "
            "When this tool is called, acknowledge in 5 words or less. No elaboration.",
            PropertyList({Property("speed", kPropertyTypeInteger, 0, 0, 100)}),
            [this](const PropertyList& props) -> ReturnValue {
                int s = ResolveSpeed(props["speed"].value<int>());
                SetLeft(-s);
                SetRight(s);
                return true;
            });

        mcp.AddTool("self.car.turn_right",
            "REQUIRED: Call this tool to physically turn the car right (tank turn). "
            "Optional speed 1-100 overrides current speed. 0 means use current speed. "
            "When this tool is called, acknowledge in 5 words or less. No elaboration.",
            PropertyList({Property("speed", kPropertyTypeInteger, 0, 0, 100)}),
            [this](const PropertyList& props) -> ReturnValue {
                int s = ResolveSpeed(props["speed"].value<int>());
                SetLeft(s);
                SetRight(-s);
                return true;
            });

        mcp.AddTool("self.car.stop",
            "REQUIRED: Call this tool to immediately stop all motors. "
            "Must be called to stop the car — the car will keep moving otherwise. "
            "When this tool is called, acknowledge in 5 words or less. No elaboration.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                SetLeft(0);
                SetRight(0);
                return true;
            });

        mcp.AddTool("self.car.set_speed",
            "Set the default speed for move and turn commands. Range: 1 to 100. "
            "Does NOT start or stop movement — use move/turn tools for that. "
            "When this tool is called, acknowledge in 5 words or less. No elaboration.",
            PropertyList({Property("speed", kPropertyTypeInteger, 60, 1, 100)}),
            [this](const PropertyList& props) -> ReturnValue {
                speed_ = props["speed"].value<int>();
                uint32_t duty = (uint32_t)(speed_ * CAR_LEDC_MAX_DUTY / 100);
                ledc_set_duty(CAR_LEDC_MODE, ch_l_, duty);
                ledc_update_duty(CAR_LEDC_MODE, ch_l_);
                ledc_set_duty(CAR_LEDC_MODE, ch_r_, duty);
                ledc_update_duty(CAR_LEDC_MODE, ch_r_);
                return true;
            });

        mcp.AddTool("self.car.get_state",
            "Get current speed setting",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return std::string("{\"base_speed\": ") + std::to_string(speed_) + "}";
            });
    }
};

#endif // __CAR_CONTROLLER_H__
