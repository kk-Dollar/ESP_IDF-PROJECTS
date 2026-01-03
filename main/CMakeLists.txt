idf_component_register(
    SRCS "app_main.c" "microros_node.c"  "control_loop.c" "motor_driver.c"
    INCLUDE_DIRS "."
    REQUIRES micro_ros_espidf_component 
    PRIV_REQUIRES esp_timer nvs_flash
)
