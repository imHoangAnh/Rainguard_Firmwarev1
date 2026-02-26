/**
 * @file network_config.h
 * @brief Manual configuration for network settings
 */

#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H

/*
 * DEVICE CONFIG
 */
#define DEVICE_ID "01"

/*
 * WIFI CONFIG
 */
#define WIFI_SSID "Hoanganhh"
#define WIFI_PASSWORD "250303hanh"

/*
 * MQTT CONFIG
 */
#define MQTT_BROKER_URI "mqtt://10.226.242.27:1883"
#define MQTT_TOPIC_PREFIX "trainguard/data"
#define MQTT_ALERT_TOPIC "trainguard/alert/" DEVICE_ID

/*
 * CLOUDINARY CONFIG
 */
#define CLOUDINARY_UPLOAD_URL                                                  \
  "https://api.cloudinary.com/v1_1/dp5wo5yjr/image/upload"

#define CLOUDINARY_UPLOAD_PRESET "TrainGuard"

/*
 * SENSOR INTERVALS (milliseconds)
 */
#define SENSOR_INTERVAL_MS 10000
#define CAMERA_INTERVAL_MS 60000

#endif /* NETWORK_CONFIG_H */
