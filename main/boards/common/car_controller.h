#ifndef __CAR_CONTROLLER_H__
#define __CAR_CONTROLLER_H__

#include "mcp_server.h"
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "ultrasonic.h"

#define CAR_TAG "CarController"
#define CAR_OBSTACLE_POLL_MS     200
#define CAR_DEFAULT_THRESHOLD_CM 20
#define CAR_MAX_DISTANCE_CM      200

// Motor A = left wheel (IN1/IN2), Motor B = right wheel (IN3/IN4)
// EN pins are jumpered to 5V — full speed, no PWM needed
class CarController {
private:
    gpio_num_t l_in1_, l_in2_;
    gpio_num_t r_in1_, r_in2_;

    ultrasonic_sensor_t sensor_;
    uint32_t threshold_cm_ = CAR_DEFAULT_THRESHOLD_CM;
    bool guard_enabled_ = true;
    bool moving_backward_ = false;

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

    // dir: 1=forward, -1=backward, 0=stop
    void SetMotor(gpio_num_t in1, gpio_num_t in2, int dir) {
        if (dir > 0) {
            gpio_set_level(in1, 1);
            gpio_set_level(in2, 0);
        } else if (dir < 0) {
            gpio_set_level(in1, 0);
            gpio_set_level(in2, 1);
        } else {
            gpio_set_level(in1, 0);
            gpio_set_level(in2, 0);
        }
    }

    void SetLeft(int dir)  { SetMotor(l_in1_, l_in2_, dir); }
    void SetRight(int dir) { SetMotor(r_in1_, r_in2_, dir); }

    void HardStop() {
        SetLeft(0);
        SetRight(0);
    }

    static void ObstacleTask(void* arg) {
        auto* car = static_cast<CarController*>(arg);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(CAR_OBSTACLE_POLL_MS));
            if (!car->guard_enabled_ || car->moving_backward_) continue;

            uint32_t dist_cm = 0;
            esp_err_t err = ultrasonic_measure_cm(&car->sensor_, CAR_MAX_DISTANCE_CM, &dist_cm);
            if (err == ESP_OK && dist_cm < car->threshold_cm_) {
                ESP_LOGD(CAR_TAG, "Obstacle at %lu cm — stopping", dist_cm);
                car->HardStop();
            }
        }
    }

public:
    CarController(gpio_num_t in1, gpio_num_t in2,
                  gpio_num_t in3, gpio_num_t in4,
                  gpio_num_t trig, gpio_num_t echo)
        : l_in1_(in1), l_in2_(in2), r_in1_(in3), r_in2_(in4)
    {
        ConfigGpio(l_in1_); ConfigGpio(l_in2_);
        ConfigGpio(r_in1_); ConfigGpio(r_in2_);

        sensor_ = { .trigger_pin = trig, .echo_pin = echo };
        ESP_ERROR_CHECK(ultrasonic_init(&sensor_));

        xTaskCreate(ObstacleTask, "obstacle", 4096, this, 5, nullptr);

        auto& mcp = McpServer::GetInstance();

        mcp.AddTool("self.car.move_forward",
            "REQUIRED: Call this tool to physically move the car forward. "
            "When this tool is called, acknowledge in 5 words or less. No elaboration.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                moving_backward_ = false;
                SetLeft(1); SetRight(1);
                return true;
            });

        mcp.AddTool("self.car.move_backward",
            "REQUIRED: Call this tool to physically move the car backward. "
            "When this tool is called, acknowledge in 5 words or less. No elaboration.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                moving_backward_ = true;
                SetLeft(-1); SetRight(-1);
                return true;
            });

        mcp.AddTool("self.car.turn_left",
            "REQUIRED: Call this tool to physically turn the car left (tank turn). "
            "When this tool is called, acknowledge in 5 words or less. No elaboration.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                moving_backward_ = false;
                SetLeft(-1); SetRight(1);
                return true;
            });

        mcp.AddTool("self.car.turn_right",
            "REQUIRED: Call this tool to physically turn the car right (tank turn). "
            "When this tool is called, acknowledge in 5 words or less. No elaboration.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                moving_backward_ = false;
                SetLeft(1); SetRight(-1);
                return true;
            });

        mcp.AddTool("self.car.stop",
            "REQUIRED: Call this tool to immediately stop all motors. "
            "Must be called to stop the car — car keeps moving otherwise. "
            "When this tool is called, acknowledge in 5 words or less. No elaboration.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                moving_backward_ = false;
                HardStop();
                return true;
            });

        mcp.AddTool("self.car.get_distance",
            "Measure distance to nearest obstacle in centimeters using ultrasonic sensor. "
            "When this tool is called, acknowledge in 5 words or less. No elaboration.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                uint32_t dist_cm = 0;
                esp_err_t err = ultrasonic_measure_cm(&sensor_, CAR_MAX_DISTANCE_CM, &dist_cm);
                if (err == ESP_OK) {
                    return std::string("{\"distance_cm\": ") + std::to_string(dist_cm) + "}";
                }
                return std::string("{\"error\": \"sensor timeout\"}");
            });

        mcp.AddTool("self.car.set_obstacle_threshold",
            "Set auto-stop distance in cm. Car stops automatically when obstacle is closer than this. "
            "Set to 0 to disable auto-stop. Default is 20 cm. "
            "When this tool is called, acknowledge in 5 words or less. No elaboration.",
            PropertyList({Property("threshold_cm", kPropertyTypeInteger, CAR_DEFAULT_THRESHOLD_CM, 0, CAR_MAX_DISTANCE_CM)}),
            [this](const PropertyList& props) -> ReturnValue {
                threshold_cm_ = (uint32_t)props["threshold_cm"].value<int>();
                guard_enabled_ = threshold_cm_ > 0;
                return true;
            });

        mcp.AddTool("self.car.get_state",
            "Get obstacle guard status and threshold",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return std::string("{\"guard_enabled\": ") +
                       (guard_enabled_ ? "true" : "false") +
                       ", \"threshold_cm\": " + std::to_string(threshold_cm_) + "}";
            });
    }
};

#endif // __CAR_CONTROLLER_H__
