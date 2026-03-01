#include "Copter.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_GPS/AP_GPS.h>
#if AP_SIM_ENABLED
#include <SITL/SITL.h>
#include <unistd.h>
#endif

extern const AP_HAL::HAL& hal;


/*
 * 新增代码：自定义飞行模式 ModeAutoDraw 的实现文件
 * 该模式通过预定义的航点引导飞行器飞出一个星形图案
 */

// ModeAutoDraw::init - 初始化自动绘制星形模式
// 新增代码 hangoa
bool ModeAutoDraw::init(bool ignore_checks)
{
    // 检查位置估算是否正常或者是否忽略检查
    if (copter.position_ok() || ignore_checks) {
        // 初始化偏航角（Yaw）模式为默认
        auto_yaw.set_mode_to_default(false);

        // 重置路径索引并生成星形路径
        path_num = 0;
        generate_path();
        
        // 新增：读取飞机名称
        // _aircraft_name[0] = '\0';
        // const int name_fd = AP::FS().open("autodraw_name.txt", O_RDONLY, true);
        // if (name_fd != -1) {
        //     gcs().send_text(MAV_SEVERITY_INFO, "read autodraw_name.txt");
        //     char line[64];
        //     if (AP::FS().fgets(line, sizeof(line) - 1, name_fd)) {
        //         line[sizeof(line) - 1] = '\0';
        //         const size_t len = strcspn(line, "\r\n");
        //         line[len] = '\0';
        //         strncpy(_aircraft_name, line, sizeof(_aircraft_name) - 1);
        //         _aircraft_name[sizeof(_aircraft_name) - 1] = '\0';
        //     }
        //     AP::FS().close(name_fd);
        // }

        // 启动位置控制模式
        pos_control_start();
        return true;
    }else{
        return false;
    }
}

// ModeAutoDraw::generate_path - 生成星形路径的 7 个关键点（含起点）
// 新增代码
void ModeAutoDraw::generate_path()
{
    // 从参数中获取星形半径
    float radius_cm = g2.star_radius_cm;

    Vector3f stopping_point_neu_cm;
    wp_nav->get_wp_stopping_point_NEU_cm(stopping_point_neu_cm);
    path[0] = stopping_point_neu_cm;

    // 基于三角函数计算星形五个顶点的相对位置
    path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    path[6] = path[1]; // 回到第一个顶点完成闭合
}

// ModeAutoDraw::pos_control_start - 初始化位置控制器
// 新增代码
void ModeAutoDraw::pos_control_start()
{
    // 初始化路点和样条控制器
    wp_nav->wp_and_spline_init_m();

    // 设置第一个目标点为起始点
    wp_nav->set_wp_destination_NEU_cm(path[0], false);

    // 再次确保偏航角模式正确
    auto_yaw.set_mode_to_default(false);
}

// ModeAutoDraw::run - 模式的主运行循环，每 100Hz 调用一次  
// 新增代码
void ModeAutoDraw::run()
{
    // 打印 GPS 时间，每 1 秒打印一次
    uint64_t now_us;
    if (AP::rtc().get_utc_usec(now_us)) {
    // now_us 是经过 GPS 授时校准后的当前 UTC 微秒数
    }
    
    static uint32_t last_gps_time_print_ms;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_gps_time_print_ms >= 1000) {
        last_gps_time_print_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_INFO, "ID: %s, GPS time: week %u ms %lu", _aircraft_name, copter.gps.time_week(), (unsigned long)copter.gps.time_week_ms());

        // 新增：写入飞机名称与 GPS 时间到文件
        const int log_fd = AP::FS().open("autodraw_gps_time.log", O_WRONLY | O_CREAT | O_APPEND, true);
        if (log_fd != -1) {
            char log_line[128];
            snprintf(log_line, sizeof(log_line), "%s,GPS time: week %u ms %lu\n",
                     _aircraft_name,
                     copter.gps.time_week(),
                     (unsigned long)copter.gps.time_week_ms());
            const size_t to_write = strnlen(log_line, sizeof(log_line));
            AP::FS().write(log_fd, log_line, to_write);
            AP::FS().close(log_fd);
        }
    }
    
    // 如果还没飞完所有 6 段路径
    if(path_num < 6){
        // 检查是否到达当前目标路点
        if(wp_nav->reached_wp_destination()){
            // 索引增加并指向下一个星形顶点
            path_num ++;
            wp_nav->set_wp_destination_NEU_cm(path[path_num], false);
        }
    }

    // 执行具体的位置控制逻辑
    pos_control_run();
}

// ModeAutoDraw::pos_control_run - 执行底层的位置控制逻辑
// 新增代码
void ModeAutoDraw::pos_control_run()    
{
    // 飞行安全检查：如果未解锁、未自动解锁、电机锁定或着陆已完成，则重置油门并退出
    if (!motors->armed() || !copter.ap.auto_armed || !motors->get_interlock() || copter.ap.land_complete) {
        zero_throttle_and_relax_ac();
        return;
    }

    // 处理飞行员的偏航（Yaw）输入
    float target_yaw_rate_rads = 0.0f;
    if (!copter.failsafe.radio) {
        // 获取飞行员期望的偏航速率
        target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();
        if (!is_zero(target_yaw_rate_rads)) {
            // 如果飞行员有偏航输入，则切换到手动偏航保持模式
            auto_yaw.set_mode(AutoYaw::Mode::PILOT_RATE);
        }
    }

    // 设置电机状态为全范围
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 更新路点导航控制器，并处理地形失效保护
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // 更新 Z 轴（高度）位置控制器
    pos_control->D_update_controller();

    // 根据偏航模式调用姿态控制器
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        wp_nav->get_roll_rad(),
        wp_nav->get_pitch_rad(),
        target_yaw_rate_rads);
}
