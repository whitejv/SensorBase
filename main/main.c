// main/main.c - ESP-IDF Application Entry Point

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"         // From sensor_system component
#include "types.h"          // From sensor_system component
#include "system_init.h"    // From sensor_system component

static const char *TAG = "MAIN";

void app_main(void) {
    // Initialize ESP_LOG system
    esp_log_level_set("*", ESP_LOG_INFO);  // Set global log level
    esp_log_level_set(TAG, ESP_LOG_DEBUG); // Set specific tag level

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "=== GenericSensor ESP-IDF Starting ===");
    ESP_LOGI(TAG, "======================================");

    // Create the system information task (runs once to display system info)
    // Give it higher priority temporarily to ensure it runs first
    BaseType_t xResult = xTaskCreate(
        vSystemInfoTask,              // Task function
        "SystemInfo",                 // Task name
        TASK_STACK_SIZE_BACKGROUND,   // Stack size from config.h
        NULL,                         // Parameters
        TASK_PRIORITY_FIXED_FREQ,     // Higher priority (3) to run first
        &xSystemInfoTaskHandle        // Task handle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "System information task created successfully");
    } else {
        ESP_LOGW(TAG, "WARNING: Failed to create system information task");
    }

    // Create the system monitoring task (runs continuously)
    xResult = xTaskCreate(
        vSystemMonitorTask,           // Task function
        "SysMonitor",                 // Task name
        TASK_STACK_SIZE_BACKGROUND,   // Stack size from config.h
        NULL,                         // Parameters
        TASK_PRIORITY_BACKGROUND,     // Priority from config.h
        &xSystemMonitorTaskHandle     // Task handle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "System monitoring task created successfully");
    } else {
        ESP_LOGE(TAG, "ERROR: Failed to create system monitoring task!");
        // Critical error - halt system
        abort();
    }

    // Create the heartbeat task (LED blink and status)
    xResult = xTaskCreate(
        vHeartbeatTask,               // Task function
        "Heartbeat",                  // Task name
        TASK_STACK_SIZE_BACKGROUND,   // Stack size from config.h
        NULL,                         // Parameters
        TASK_PRIORITY_BACKGROUND,     // Priority from config.h
        &xHeartbeatTaskHandle         // Task handle
    );

    if (xResult == pdPASS) {
        ESP_LOGI(TAG, "Heartbeat task created successfully");
    } else {
        ESP_LOGW(TAG, "WARNING: Failed to create heartbeat task");
    }

    // FreeRTOS scheduler is already running in ESP-IDF
    // Tasks will start executing immediately
    ESP_LOGI(TAG, "All tasks created successfully. System is now running...");
}