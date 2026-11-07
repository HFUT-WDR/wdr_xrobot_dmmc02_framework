# 编译未启用模块的构造接口及完整 YAML 生成配置。
# 检查目标的对象文件不链接进实际运行固件。
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(EXTERNAL_CHECK_DIR "${CMAKE_CURRENT_BINARY_DIR}/external_modules")
set(EXTERNAL_CHECK_NAMES CMD RMMotor DMMotor DR16 Referee HostData SharedTopic SharedTopicClient I6x EventBinder BoardAttitude DmImu Chassis PowerControl DualYaw Pitch Launcher Gimbal external_profile imu_profile imu_can_profile imu_rs485_stream_profile chassis_profile gimbal_profile)
set(EXTERNAL_CHECK_SOURCES)
foreach(name IN LISTS EXTERNAL_CHECK_NAMES)
    list(APPEND EXTERNAL_CHECK_SOURCES "${EXTERNAL_CHECK_DIR}/${name}.cpp")
endforeach()
file(GLOB_RECURSE EXTERNAL_CHECK_HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/Modules/*/*.hpp")
add_custom_command(
    OUTPUT ${EXTERNAL_CHECK_SOURCES} "${EXTERNAL_CHECK_DIR}/external_profile.hpp"
           "${EXTERNAL_CHECK_DIR}/imu_profile.hpp"
           "${EXTERNAL_CHECK_DIR}/imu_can_profile.hpp" "${EXTERNAL_CHECK_DIR}/imu_rs485_stream_profile.hpp" "${EXTERNAL_CHECK_DIR}/chassis_profile.hpp" "${EXTERNAL_CHECK_DIR}/gimbal_profile.hpp"
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_external_checks.py"
            --output "${EXTERNAL_CHECK_DIR}"
    DEPENDS tools/generate_external_checks.py config/templates/external_devices.yaml.in
            tests/external_modules/compile_bindings.json xrobot.lock
            User/xrobot.yaml config/templates/chassis.yaml config/templates/gimbal.yaml config/templates/dm_imu.yaml User/app_main.cpp ${EXTERNAL_CHECK_HEADERS}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Generating inactive external-device compile checks"
    VERBATIM)
add_library(external_module_checks OBJECT ${EXTERNAL_CHECK_SOURCES}
)
target_link_libraries(external_module_checks PRIVATE xr)
target_include_directories(external_module_checks PRIVATE "${EXTERNAL_CHECK_DIR}")
