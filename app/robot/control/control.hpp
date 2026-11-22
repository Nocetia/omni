#pragma once

#include "attitude.hpp"
#include "motor.hpp"
#include "remoter.hpp"
#include "pid.hpp"
#include "msg.hpp"
#include "configs/config_types.hpp"

#include <cstdint>
#include <limits>

namespace robot
{
    struct motor_feedback
    {
        motor::state state = motor::state::offline;
        motor::feedback_t fdb{};
    };

    struct motor_command
    {
        // current/voltage keep the driver's raw units; mode must be supported by the motor.
        motor::mode mode = motor::mode::relax;
        motor::command_t cmd{};
    };

    namespace vision
    {
        struct feedback
        {
            bool detect_color = false;
            float bullet_speed = 0.0f; // m/s
            uint16_t bullet_count = 0; // confirmed shots, not operator fire requests
            // The input producer checks color, bullet-speed and shot-count availability/freshness.
            bool valid = false;
        };

        struct target
        {
            bool fire = false;
            // rad, rad/s, rad/s^2; use the same frame as the gimbal attitude.
            float yaw = 0.0f;
            float yaw_vel = 0.0f;
            float yaw_acc = 0.0f;
            float pitch = 0.0f;
            float pitch_vel = 0.0f;
            float pitch_acc = 0.0f;

            uint32_t sequence = 0;
            uint32_t received_ms = 0;
            bool valid = false;
        };
    } // namespace vision

    namespace gimbal
    {
        enum class mode : uint8_t
        {
            off,
            manual,
            autoaim,
        };

        struct command
        {
            gimbal::mode mode = gimbal::mode::off;
            // Manual angular rates, rad/s.
            float yaw_rate = 0.0f;
            float pitch_rate = 0.0f;
            // Per-frame displacements, rad; the producer must not replay cached mouse input.
            float yaw_delta = 0.0f;
            float pitch_delta = 0.0f;
            bool roll_enabled = false; // fn1 request; roll actuator behaviour is not implemented yet.
        };

        struct feedback
        {
            motor_feedback yaw{};
            motor_feedback pitch{};
            motor_feedback roll{};
        };

        struct output
        {
            motor_command yaw{};
            motor_command pitch{};
            motor_command roll{};
        };

        struct status
        {
            bool ready = false;
            bool aimed = false;//电控误差小
            // Accepted, fresh vision target permits firing; not the operator's fire request.
            bool fire_allowed = false;//视觉开火
        };

        struct result
        {
            output command{};
            status state{};
        };

        struct axis_ref
        {
            float pos = 0.0f; // rad
            float vel = 0.0f; // rad/s
            float acc = 0.0f; // rad/s^2
            // Control laws that predict targets must also respect these position bounds.
            float min_pos = -std::numeric_limits<float>::infinity();
            float max_pos = std::numeric_limits<float>::infinity();
        };

        struct axis_state
        {
            float pos = 0.0f; // rad, same frame and wrap convention as axis_ref
            float vel = 0.0f; // rad/s
        };

        class axis_control
        {
        public:
            virtual ~axis_control() = default;

            virtual motor_command update(const axis_ref& ref, const axis_state& state, float dt_s) = 0;
            virtual void reset(const axis_state& state) = 0;
        };

        class controller
        {
        public:
            // Non-owning references; each axis controller must outlive this controller.
            controller(axis_control& yaw, axis_control& pitch, const cfg::gimbal_params& config);

            // Caller supplies finite positive dt_s; this module owns IMU/vision freshness.
            result update(const command& command, const imu::attitude::state& attitude,
                const vision::target& vision, const feedback& feedback, uint32_t now_ms, float dt_s);
            void reset(const imu::attitude::state& attitude);

        private:
            cfg::gimbal_params config_;
            axis_control& yaw_;
            axis_control& pitch_;
            axis_ref yaw_ref_{};
            axis_ref pitch_ref_{};
            gimbal::mode mode_ = gimbal::mode::off;
            bool valid_ = false;
        };

        class pid_axis final : public axis_control
        {
        public:
            explicit pid_axis(const cfg::axis_params& config);

            motor_command update(const axis_ref& ref, const axis_state& state, float dt_s) override;
            void reset(const axis_state& state) override;

        private:
            cfg::axis_params config_;
            ::control::pid pos_;
            ::control::pid vel_;
            bool valid_ = false;
        };
    } // namespace gimbal

    namespace shooter
    {
        struct command
        {
            bool enabled = false;
            bool fire = false; // one-cycle event in manual, held request in continuous mode
            bool continuous = false;
            bool aim_required = false;
            bool reverse = false;
        };

        struct feedback
        {
            motor_feedback fric_left{};
            motor_feedback fric_right{};
            motor_feedback trigger{};

            float heat = 0.0f;
            float heat_limit = 0.0f;
            float cooling_rate = 0.0f;  // heat units/s, supplied by referee
            uint32_t heat_sequence = 0; // changes only when a new heat observation arrives
            // The caller checks freshness of both heat and heat-limit packets each cycle.
            bool heat_valid = false;
        };

        struct output
        {
            motor_command fric_left{};
            motor_command fric_right{};
            motor_command trigger{};
        };

        class controller
        {
        public:
            explicit controller(const cfg::shooter_params& config);

            // Caller supplies finite positive dt_s and fresh motor feedback.
            output update(const command& command, const feedback& feedback, const gimbal::status& aim,
                float dt_s);
            void reset(const feedback& feedback);

        private:
            cfg::shooter_params config_;
            bool valid_ = false;
            bool continuous_ = false;
            bool aim_required_ = false;
            bool active_ = false;
            bool blocked_ = false;
            float shot_start_ = 0.0f;
            float shot_time_ = 0.0f;
            float heat_estimate_ = 0.0f;
            uint32_t heat_sequence_ = 0;
            bool heat_seen_ = false;
        };
    } // namespace shooter

    namespace chassis
    {
        struct command
        {
            bool enabled = false;
            // Chassis frame: x forward, y left, positive wz counter-clockwise viewed from above.
            float vx = 0.0f; // m/s
            float vy = 0.0f; // m/s
            float wz = 0.0f; // rad/s
        };

        struct feedback
        {
            motor_feedback wheel_lf{};
            motor_feedback wheel_lb{};
            motor_feedback wheel_rf{};
            motor_feedback wheel_rb{};
        };

        struct output
        {
            motor_command wheel_lf{};
            motor_command wheel_lb{};
            motor_command wheel_rf{};
            motor_command wheel_rb{};
        };

        class controller
        {
        public:
            explicit controller(const cfg::chassis_params& config);

            output update(const command& command, const feedback& feedback);

        private:
            cfg::chassis_params config_;
            bool valid_ = false;
        };
    } // namespace chassis

    namespace remote
    {
        struct input
        {
            remotes::reading value{};
            bool online = false;
            // Increment only on decoded frames, not on reads of the cached value.
            uint32_t sequence = 0;
            uint32_t received_ms = 0;
        };

        struct command
        {
            bool armed = false;
            gimbal::command gimbal{};
            shooter::command shooter{};
            chassis::command chassis{};
            // Cumulative events survive latest-value delivery between different task rates.
            float mouse_yaw = 0.0f;
            float mouse_pitch = 0.0f;
            uint32_t shots = 0;
            uint32_t session = 0;
            uint32_t received_ms = 0;
        };

        class controller
        {
        public:
            explicit controller(const cfg::remote_params& config);

            command update(const input& input, float dt_s);
            void reset(const input& input);

        private:
            cfg::remote_params config_;
            remotes::reading previous_{};
            uint32_t sequence_ = 0;
            bool online_ = false;
            bool armed_ = false;
            bool fric_enabled_ = false;
            bool autoaim_ = false;
            bool roll_enabled_ = false;
            bool wheel_ready_ = false;
            float mouse_yaw_ = 0.0f;
            float mouse_pitch_ = 0.0f;
            uint32_t shots_ = 0;
            uint32_t session_ = 0;
            float keyboard_vx_ = 0.0f;
            float keyboard_vy_ = 0.0f;
            bool valid_ = false;
        };
    } // namespace remote

    // Input producers own snapshots; both control tasks own their own controller instances.
    struct gimbal_input
    {
        uint32_t now_ms = 0;
        float dt_s = 0.0f;
        uint32_t received_ms = 0; // motor snapshot acquisition time
        remote::command remote{};
        imu::attitude::state attitude{};
        vision::target vision{};
        vision::feedback vision_feedback{};
        gimbal::feedback gimbal{};
        shooter::feedback shooter{};
    };

    struct gimbal_output
    {
        gimbal::output gimbal{};
        shooter::output shooter{};
        gimbal::status aim{};
        bool roll_requested = false;
    };

    struct chassis_input
    {
        uint32_t now_ms = 0;
        float dt_s = 0.0f;
        uint32_t received_ms = 0; // wheel snapshot acquisition time
        remote::command remote{};
        chassis::feedback chassis{};
    };

    class gimbal_control
    {
    public:
        gimbal_control(gimbal::axis_control& yaw, gimbal::axis_control& pitch,
            const cfg::control_params& config);

        gimbal_output update(const gimbal_input& input);
        void reset(const gimbal_input& input);

    private:
        gimbal::controller gimbal_;
        shooter::controller shooter_;
        cfg::control_params config_;
        uint32_t session_ = 0;
        uint32_t shots_ = 0;
        float mouse_yaw_ = 0.0f;
        float mouse_pitch_ = 0.0f;
        bool active_ = false;
    };

    class chassis_control
    {
    public:
        explicit chassis_control(const cfg::control_params& config);
        chassis::output update(const chassis_input& input);

    private:
        chassis::controller chassis_;
        cfg::control_params config_;
    };

    // Single writers: receiver adapter, Remote task, sensor adapters, then each control task.
    // msg copies under a mutex; no controller instance or motor object is shared by tasks.
    inline msg::topic<remote::input> remote_inputs;
    inline msg::topic<remote::command> remote_commands;
    inline msg::topic<gimbal_input> gimbal_inputs;
    inline msg::topic<chassis_input> chassis_inputs;
    inline msg::topic<gimbal_output> gimbal_outputs;
    inline msg::topic<chassis::output> chassis_outputs;
} // namespace robot
