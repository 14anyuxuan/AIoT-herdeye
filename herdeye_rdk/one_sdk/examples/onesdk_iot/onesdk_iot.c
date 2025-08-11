#include "iot_log.h"
#include "onesdk.h"
#include "thing_model/iot_tm_api.h"
#include "thing_model/property.h"
#include <fcntl.h> 
#include <string.h>
#include "../../external_libs/cJson/cJSON.h"

#define TEST_ONESDK_IOT "test_onesdk_iot"

// online vei_dev账户
#define SAMPLE_HTTP_HOST "https://iot-cn-shanghai.iot.volces.com"
#define SAMPLE_INSTANCE_ID "684a6fed9541492e944b6a76"
#define SAMPLE_MQTT_HOST "684a6fed9541492e944b6a76-cn-shanghai.iot.volces.com"
#define SAMPLE_DEVICE_NAME "Herdeye_car"
#define SAMPLE_DEVICE_SECRET "877a1b1ea929f024832d2ed1"
#define SAMPLE_PRODUCT_KEY "68510a81a15eeaadc5ea02ae"
#define SAMPLE_PRODUCT_SECRET "a6aad77a189bae059efe42b1"

// 全局变量存储传感器数据
static double rt_temperature = 0.0;
static double rt_humidity = 0.0;
static int rt_water_level = 0;
static int rt_weight = 0;

static double rt_gps_lng = 0.0;
static double rt_gps_lat = 0.0;
static double rt_gps_altitude = 0.0;
static double rt_gps_cogt = 0.0;
static double rt_gps_speed = 0.0;
static double rt_gps_satellites = 0.0;

static int rt_light = 0;
static int rt_waterpump = 0;
static int rt_motor = 0;
static int rt_door = 0;
static int rt_heat = 0;
static int rt_fan = 0;

// 全局缓冲区用于存储部分数据
static char input_buffer[4096] = {0};
static size_t buffer_index = 0;

// 设置非阻塞模式
void set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

// 解析传感器数据
void parse_sensor_data(const char* json_str) {
    // 调试输出完整JSON
    printf("收到完整JSON数据: %s\n", json_str);
    
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        printf("JSON解析失败: %s\n", json_str);
        return;
    }

    // 处理SHT30传感器数据
    if (cJSON_GetObjectItem(root, "sensor_type") && 
        strcmp(cJSON_GetObjectItem(root, "sensor_type")->valuestring, "sht30") == 0) {
        // 处理温度
        cJSON *temp = cJSON_GetObjectItem(root, "temperature");
        if (temp && cJSON_IsNumber(temp)) {
            rt_temperature = temp->valuedouble;
            printf("更新温度: %.2f\n", rt_temperature);
        }
        
        // 处理湿度
        cJSON *humi = cJSON_GetObjectItem(root, "humidity");
        if (humi && cJSON_IsNumber(humi)) {
            rt_humidity = humi->valuedouble;
            printf("更新湿度: %.2f\n", rt_humidity);
        }
    }
    
    // 处理牧场传感器数据
    if (cJSON_GetObjectItem(root, "sensor_type") && 
        strcmp(cJSON_GetObjectItem(root, "sensor_type")->valuestring, "home") == 0) {
        // 处理水位
        cJSON *water = cJSON_GetObjectItem(root, "water_level");
        if (water && cJSON_IsNumber(water)) {
            rt_water_level = (int)water->valuedouble;
            printf("更新水位: %d\n", rt_water_level);
        }
        // 处理重量
        cJSON *weight = cJSON_GetObjectItem(root, "weight");
        if (weight && cJSON_IsNumber(weight)) {
            rt_weight = (int)weight->valuedouble;
            printf("更新重量: %d\n", rt_weight);
        }
    }
    
    // 处理gps传感器数据
    if (cJSON_GetObjectItem(root, "sensor_type") && 
        strcmp(cJSON_GetObjectItem(root, "sensor_type")->valuestring, "gps") == 0) {
        // 处理经度
        cJSON *gps_lng = cJSON_GetObjectItem(root, "gps_lng");
        if (gps_lng && cJSON_IsNumber(gps_lng)) {
            rt_gps_lng = gps_lng->valuedouble;
            printf("更新经度: %.6f\n", rt_gps_lng);
        }
        // 处理纬度
        cJSON *gps_lat = cJSON_GetObjectItem(root, "gps_lat");
        if (gps_lat && cJSON_IsNumber(gps_lat)) {
            rt_gps_lat = gps_lat->valuedouble;
            printf("更新纬度: %.6f\n", rt_gps_lat);
        }
        // 处理海拔
        cJSON *gps_altitude = cJSON_GetObjectItem(root, "gps_altitude");
        if (gps_altitude && cJSON_IsNumber(gps_altitude)) {
            rt_gps_altitude = gps_altitude->valuedouble;
            printf("更新海拔: %.2f\n", rt_gps_altitude);
        }
        // 处理真北航向角
        cJSON *gps_cogt = cJSON_GetObjectItem(root, "gps_cogt");
        if (gps_cogt && cJSON_IsNumber(gps_cogt)) {
            rt_gps_cogt = gps_cogt->valuedouble;
            printf("更新真北航向角: %.2f\n", rt_gps_cogt);
        }
        // 处理速度
        cJSON *gps_speed = cJSON_GetObjectItem(root, "gps_speed");
        if (gps_speed && cJSON_IsNumber(gps_speed)) {
            rt_gps_speed = gps_speed->valuedouble;
            printf("更新速度: %.2f\n", rt_gps_speed);
        }
        // 处理卫星数
        cJSON *gps_satellites = cJSON_GetObjectItem(root, "gps_satellites");
        if (gps_satellites && cJSON_IsNumber(gps_satellites)) {
            rt_gps_satellites = gps_satellites->valuedouble;
            printf("更新卫星数: %.2f\n", rt_gps_satellites);
        }
    }
    
    // 处理控制器状态数据
    if (cJSON_GetObjectItem(root, "sensor_type") && 
        strcmp(cJSON_GetObjectItem(root, "sensor_type")->valuestring, "control") == 0) {
        // 处理照明状态数据
        cJSON *light = cJSON_GetObjectItem(root, "light");
        if (light && cJSON_IsNumber(light)) {
            rt_light = (int)light->valuedouble;
            printf("更新照明状态数据: %d\n", rt_light);
        }
        // 处理水泵状态数据
        cJSON *waterpump = cJSON_GetObjectItem(root, "waterpump");
        if (waterpump && cJSON_IsNumber(waterpump)) {
            rt_waterpump = (int)waterpump->valuedouble;
            printf("更新水泵状态数据: %d\n", rt_waterpump);
        }
        // 处理电机状态数据
        cJSON *motor = cJSON_GetObjectItem(root, "motor");
        if (motor && cJSON_IsNumber(motor)) {
            rt_motor = (int)motor->valuedouble;
            printf("更新电机状态数据: %d\n", rt_motor);
        }
        // 处理圈门状态数据
        cJSON *door = cJSON_GetObjectItem(root, "door");
        if (door && cJSON_IsNumber(door)) {
            rt_door = (int)door->valuedouble;
            printf("更新圈门状态数据: %d\n", rt_door);
        }
        // 处理加热片状态数据
        cJSON *heat = cJSON_GetObjectItem(root, "heat");
        if (heat && cJSON_IsNumber(heat)) {
            rt_heat = (int)heat->valuedouble;
            printf("更新加热片状态数据: %d\n", rt_heat);
        }
        // 处理风机状态数据
        cJSON *fan = cJSON_GetObjectItem(root, "fan");
        if (fan && cJSON_IsNumber(fan)) {
            rt_fan = (int)fan->valuedouble;
            printf("更新风机状态数据: %d\n", rt_fan);
        }
    }
    

    cJSON_Delete(root);
}

//**********************************************************************************************************************************************************************************//
void* keepalive_thread(void *arg) {
    onesdk_ctx_t *ctx = (onesdk_ctx_t *)arg;
    onesdk_iot_keepalive(ctx);
}

void test_aiot_ota_download_complete_t(void *handler,
    int error_code, iot_ota_job_info_t *job_info,
    const char *ota_file_path,
    void *user_data) {
    DEVICE_LOGI(TEST_ONESDK_IOT, "test_aiot_ota_download_complete_t error_code = %d ota_file_path = %s", error_code, ota_file_path);
    onesdk_ctx_t *ctx = (onesdk_ctx_t *)user_data;
    if (error_code == 0) {
        onesdk_iot_ota_set_device_module_info(ctx, job_info->module, job_info->dest_version);
        onesdk_iot_ota_report_install_status(ctx, job_info->ota_job_id, OTA_UPGRADE_DEVICE_STATUS_SUCCESS);
    }
}

void test_aiot_tm_recv_handler(void *handler, const iot_tm_recv_t *recv, void *userdata) {
    onesdk_ctx_t *ctx = (onesdk_ctx_t *)userdata;
    if (ctx == NULL || ctx->iot_tm_handler == NULL) {
        DEVICE_LOGE(TEST_ONESDK_IOT, "test_aiot_tm_recv_handler userdata is invalid");
        return;
    }

    switch (recv->type) {
        case IOT_TM_RECV_PROPERTY_SET_POST_REPLY: {
            DEVICE_LOGI(TEST_ONESDK_IOT, "test_aiot_tm_recv_handler property_set_reply.msg_id = %s property_set_reply.code = %d",
                        recv->data.property_set_post_reply.msg_id,
                        recv->data.property_set_post_reply.code);
        }
        break;

        case IOT_TM_RECV_PROPERTY_SET: {
            DEVICE_LOGI(TEST_ONESDK_IOT, "test_aiot_tm_recv_handler property_set.msg_id = %s service_call.params_json_str = %.*s", 
                recv->data.property_set.msg_id, recv->data.property_set.params_len, recv->data.property_set.params);
            
            // 结构化输出属性设置消息
            printf("ROS2_MSG|type=property_set|msg_id=%s|payload=%.*s\n",
                   recv->data.property_set.msg_id,
                   recv->data.property_set.params_len,
                   recv->data.property_set.params);
            fflush(stdout);
        }
        break;

        case IOT_TM_RECV_CUSTOM_TOPIC: {
            DEVICE_LOGI(TEST_ONESDK_IOT, "收到自定义 Topic 消息: topic = %s, payload = %s",
                        recv->data.custom_topic.custom_topic_suffix,
                        recv->data.custom_topic.params_json_str);
            
            // 结构化输出自定义Topic消息
            printf("ROS2_MSG|type=custom_topic|topic=%s|payload=%s\n",
                   recv->data.custom_topic.custom_topic_suffix,
                   recv->data.custom_topic.params_json_str);
            fflush(stdout);
        }
        break;

        case IOT_TM_RECV_EVENT_POST_REPLY: {
            DEVICE_LOGI(TEST_ONESDK_IOT, "test_aiot_tm_recv_handler property_set id = %s, code = %d module_key = %s, identifier = %s",
                        recv->data.event_post_reply.msg_id,
                        recv->data.event_post_reply.code,
                        recv->data.event_post_reply.module_key,
                        recv->data.event_post_reply.identifier);
        }
        break;

        case IOT_TM_RECV_SERVICE_CALL: {
            DEVICE_LOGI(TEST_ONESDK_IOT, "test_aiot_tm_recv_handler service_call.uuid = %s service_call.params_json_str = %s", 
                        recv->data.service_call.topic_uuid,
                        recv->data.service_call.params_json_str);
            
            // 结构化输出服务调用消息
            printf("ROS2_MSG|type=service_call|uuid=%s|payload=%s\n",
                   recv->data.service_call.topic_uuid,
                   recv->data.service_call.params_json_str);
            fflush(stdout);
        }
        break;

        default:
            DEVICE_LOGW(TEST_ONESDK_IOT, "收到未处理的 TM 消息类型: %d", recv->type);
            break;
    }
}

static char global_sign_root_ca[] = "-----BEGIN CERTIFICATE-----\n"
"MIIDXzCCAkegAwIBAgILBAAAAAABIVhTCKIwDQYJKoZIhvcNAQELBQAwTDEgMB4G\n"
"A1UECxMXR2xvYmFsU2lnbiBSb290IENBIC0gUjMxEzARBgNVBAoTCkdsb2JhbFNp\n"
"Z24xEzARBgNVBAMTCkdsb2JhbFNpZ24wHhcNMDkwMzE4MTAwMDAwWhcNMjkwMzE4\n"
"MTAwMDAwWjBMMSAwHgYDVQQLExdHbG9iYWxTaWduIFJvb3QgQ0EgLSBSMzETMBEG\n"
"A1UEChMKR2xvYmFsU2lnbjETMBEGA1UEAxMKR2xvYmFsU2lnbjCCASIwDQYJKoZI\n"
"hvcNAQEBBQADggEPADCCAQoCggEBAMwldpB5BngiFvXAg7aEyiie/QV2EcWtiHL8\n"
"RgJDx7KKnQRfJMsuS+FggkbhUqsMgUdwbN1k0ev1LKMPgj0MK66X17YUhhB5uzsT\n"
"gHeMCOFJ0mpiLx9e+pZo34knlTifBtc+ycsmWQ1z3rDI6SYOgxXG71uL0gRgykmm\n"
"KPZpO/bLyCiR5Z2KYVc3rHQU3HTgOu5yLy6c+9C7v/U9AOEGM+iCK65TpjoWc4zd\n"
"QQ4gOsC0p6Hpsk+QLjJg6VfLuQSSaGjlOCZgdbKfd/+RFO+uIEn8rUAVSNECMWEZ\n"
"XriX7613t2Saer9fwRPvm2L7DWzgVGkWqQPabumDk3F2xmmFghcCAwEAAaNCMEAw\n"
"DgYDVR0PAQH/BAQDAgEGMA8GA1UdEwEB/wQFMAMBAf8wHQYDVR0OBBYEFI/wS3+o\n"
"LkUkrk1Q+mOai97i3Ru8MA0GCSqGSIb3DQEBCwUAA4IBAQBLQNvAUKr+yAzv95ZU\n"
"RUm7lgAJQayzE4aGKAczymvmdLm6AC2upArT9fHxD4q/c2dKg8dEe3jgr25sbwMp\n"
"jjM5RcOO5LlXbKr8EpbsU8Yt5CRsuZRj+9xTaGdWPoO4zzUhw8lo/s7awlOqzJCK\n"
"6fBdRoyV3XpYKBovHd7NADdBj+1EbddTKJd+82cEHhXXipa0095MJ6RMG3NzdvQX\n"
"mcIfeg7jLQitChws/zyrVQ4PkX4268NXSb7hLi18YIvDQVETI53O9zJrlAGomecs\n"
"Mx86OyXShkDOOyyGeMlhLxS67ttVb9+E7gUJTb0o2HLO02JQZR7rkpeDMdmztcpH\n"
"WD9f\n"
"-----END CERTIFICATE-----\n";

int main() {

    printf("Hello VIO");
    int i = 0;
    // 初始化配置
    iot_basic_config_t device_config = {
        .http_host = SAMPLE_HTTP_HOST,
        .instance_id = SAMPLE_INSTANCE_ID,
        .auth_type = ONESDK_AUTH_DYNAMIC_NO_PRE_REGISTERED,
        .product_key = SAMPLE_PRODUCT_KEY,
        .product_secret = SAMPLE_PRODUCT_SECRET,
        .device_name = SAMPLE_DEVICE_NAME,
        .device_secret = SAMPLE_DEVICE_SECRET,
        .verify_ssl = true,
        .ssl_ca_cert = global_sign_root_ca,
    };
    iot_mqtt_config_t mqtt_config = {
        .mqtt_host = SAMPLE_MQTT_HOST,
        .basic_config = &device_config,
        .keep_alive = 60,
        .auto_reconnect = true,
    };
    onesdk_config_t config = {
        .device_config = &device_config,
        .mqtt_config = &mqtt_config,
    };
    
    onesdk_ctx_t *ctx = malloc(sizeof(onesdk_ctx_t));
    memset(ctx, 0, sizeof(onesdk_ctx_t));

    int ret = onesdk_init(ctx, &config);
    if (ret) {
        printf("onesdk_init failed\n");
        return 1;
    }
    ret = onesdk_iot_enable_log_upload(ctx);
    if (ret) {
        printf("onesdk_iot_enable_log_upload failed\n");
        return 1;
    }
    ret = onesdk_iot_enable_ota(ctx, "./");
    if (ret) {
        printf("onesdk_iot_enable_ota failed\n");
        return 1;
    }
    onesdk_iot_ota_set_device_module_info(ctx, "default", "1.0.0");
    onesdk_iot_ota_set_download_complete_cb(ctx, test_aiot_ota_download_complete_t);
    ret = onesdk_connect(ctx);
    if (ret) {
        printf("onesdk_connect failed\n");
        return 1;
    }
    
    printf("MQTT_CONNECTED\n");
    fflush(stdout);

    pthread_t keep_alive_thread;
    pthread_create(&keep_alive_thread, NULL, keepalive_thread, ctx);

    ret = onesdk_iot_tm_init(ctx);
    if (ret) {
        printf("onesdk_iot_tm_init failed\n");
        return 1;
    }

    iot_tm_handler_t* tm_handler = onesdk_iot_get_tm_handler(ctx);
    if (tm_handler==NULL) {
        printf("onesdk_iot_get_tm_handler failed\n");
        return 1;
    }

    onesdk_iot_tm_set_recv_cb(ctx, test_aiot_tm_recv_handler, ctx);
    onesdk_iot_tm_custom_topic_sub(ctx, "get");

    // 构造属性上报消息
    iot_tm_msg_t property_post_msg = {0};
    property_post_msg.type = IOT_TM_MSG_PROPERTY_POST;
    iot_tm_msg_property_post_t *property_post;
    iot_property_post_init(&property_post);
    // 添加属性参数
    iot_property_post_add_param_num(property_post, "default:ErrorCurrentThreshold", 0.1);
    iot_property_post_add_param_num(property_post, "default:ErrorPowerThreshold", 0);
    // GeoLocation 作为 JSON 字符串添加
    const char *geo_location_json = "{\"Altitude\":0,\"CoordinateSystem\":2,\"Latitude\":-90,\"Longitude\":-180}";
    iot_property_post_add_param_json_str(property_post, "default:GeoLocation", geo_location_json);
    iot_property_post_add_param_num(property_post, "default:LeakageEnable", 1);
    iot_property_post_add_param_num(property_post, "default:LightAdjustLevel", 0);
    iot_property_post_add_param_num(property_post, "default:LightErrorEnable", 1);
    iot_property_post_add_param_num(property_post, "default:LightStatus", 0);
    iot_property_post_add_param_num(property_post, "default:OverCurrentEnable", 0);
    iot_property_post_add_param_num(property_post, "default:OverCurrentThreshold", 0);
    iot_property_post_add_param_num(property_post, "default:OverTiltEnable", 1);
    iot_property_post_add_param_num(property_post, "default:OverVoltEnable", 1);
    iot_property_post_add_param_num(property_post, "default:OverVoltThreshold", 0);
    iot_property_post_add_param_num(property_post, "default:TiltThreshold", 0);
    iot_property_post_add_param_num(property_post, "default:UnderVoltEnable", 1);
    iot_property_post_add_param_num(property_post, "default:UnderVoltThreshold", 0);
    
    property_post_msg.data.property_post = property_post;
    iot_tm_send(ctx->iot_tm_handler, &property_post_msg);
    iot_property_post_free(property_post);
    
    printf("MODEL_INITIALIZED\n");
    fflush(stdout);
    
    // 设置标准输入为非阻塞
    set_nonblocking(STDIN_FILENO);
    
    // 添加select超时控制
    struct timeval timeout;
    
    while(1) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(STDIN_FILENO, &read_fds);
    
        // 设置超时时间为100毫秒
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;  // 100毫秒
        
        int ready = select(STDIN_FILENO + 1, &read_fds, NULL, NULL, &timeout);
        
        if (ready > 0 && FD_ISSET(STDIN_FILENO, &read_fds)) {
            char buffer[1024];
            ssize_t count;
            
            // 读取所有可用数据
            while ((count = read(STDIN_FILENO, buffer, sizeof(buffer)-1))) {
                if (count <= 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        // 没有更多数据可读
                        break;
                    }
                    perror("读取错误");
                    break;
                }
                
                buffer[count] = '\0';
                
                // 添加到全局缓冲区
                if (buffer_index + count < sizeof(input_buffer)) {
                    strncpy(input_buffer + buffer_index, buffer, count);
                    buffer_index += count;
                } else {
                    printf("输入缓冲区溢出，清空缓冲区\n");
                    buffer_index = 0;
                    continue;
                }
                
                // 处理缓冲区中的所有完整消息
                char *start = input_buffer;
                char *end;
                while ((end = strchr(start, '\n')) != NULL) {
                    // 提取一行
                    *end = '\0';
                    printf("处理完整消息: %s\n", start);
                    parse_sensor_data(start);
                    
                    // 移动到下一行
                    start = end + 1;
                }
                
                // 移动剩余数据到缓冲区开头
                size_t remaining = buffer_index - (start - input_buffer);
                if (remaining > 0) {
                    memmove(input_buffer, start, remaining);
                } else {
                    remaining = 0;
                }
                buffer_index = remaining;
                
                // 如果没有剩余空间，则跳出循环
                if (buffer_index >= sizeof(input_buffer) - 1) {
                    printf("缓冲区满，清空缓冲区\n");
                    buffer_index = 0;
                    break;
                }
            }
        }
        
        // 属性上报部分 - 每5秒执行一次
        static time_t last_report = 0;
        time_t now = time(NULL);
        if (now - last_report >= 5) {
            last_report = now;
            
            // 构造新的属性上报消息
            iot_tm_msg_t property_post_msg = {0};
            property_post_msg.type = IOT_TM_MSG_PROPERTY_POST;
            iot_tm_msg_property_post_t *property_post;
            iot_property_post_init(&property_post);
            
            iot_property_post_add_param_num(property_post, "environment:T", rt_temperature);
            iot_property_post_add_param_num(property_post, "environment:H", rt_humidity);
            
            iot_property_post_add_param_num(property_post, "home:water_level", rt_water_level);
            iot_property_post_add_param_num(property_post, "home:weight", rt_weight);
            
            iot_property_post_add_param_num(property_post, "GPS:lng", rt_gps_lng);
            iot_property_post_add_param_num(property_post, "GPS:lat", rt_gps_lat);
            iot_property_post_add_param_num(property_post, "GPS:altitude", rt_gps_altitude);
            iot_property_post_add_param_num(property_post, "GPS:cogt", rt_gps_cogt);
            iot_property_post_add_param_num(property_post, "GPS:speed", rt_gps_speed);
            
            iot_property_post_add_param_num(property_post, "home:light", rt_light);
            iot_property_post_add_param_num(property_post, "home:pump", rt_waterpump);
            iot_property_post_add_param_num(property_post, "home:motor", rt_motor);
            iot_property_post_add_param_num(property_post, "home:door", rt_door);
            iot_property_post_add_param_num(property_post, "home:heat", rt_heat);
            iot_property_post_add_param_num(property_post, "home:fan", rt_fan);
            
            // 发送属性上报
            property_post_msg.data.property_post = property_post;
            iot_tm_send(tm_handler, &property_post_msg);
            iot_property_post_free(property_post);
            
            printf("属性已上报: g_temperature=%.2f, g_humidity=%.2f, rt_water_level=%d, rt_weight=%d, rt_gps_lng=%.6f, rt_gps_lat=%.6f, gps_altitude=%.2f, rt_gps_cogt=%.2f, rt_gps_speed=%.2f, rt_gps_satellites=%.2f, rt_light=%d, rt_waterpump=%d, rt_motor=%d, rt_door=%d, rt_heat=%d, rt_fan=%d\n", 
                   rt_temperature, rt_humidity, rt_water_level, rt_weight, rt_gps_lng, rt_gps_lat, rt_gps_altitude, rt_gps_cogt, rt_gps_speed, rt_gps_satellites, rt_light, rt_waterpump, rt_motor, rt_door,  rt_heat, rt_fan);
            fflush(stdout);
        }
    }

    onesdk_iot_tm_deinit(ctx);
    onesdk_deinit(ctx);
    return 0;
}