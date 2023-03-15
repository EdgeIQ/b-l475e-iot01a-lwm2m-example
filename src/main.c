/*
 * Copyright (c) 2021 Nordic Semiconductor ASA, EdgeIQ
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/random/rand32.h>

#include <zephyr/net/wifi.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include <zephyr/net/lwm2m.h>

#include <stdio.h>

#define LOG_MODULE_NAME net_lwm2m_client_app
#define LOG_LEVEL LOG_LEVEL_DBG



#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define AUTO_CONNECT_SSID "mywifissid"

#define AUTO_CONNECT_SSID_PSK "mywifipassword"
#define LWM2M_SERVER "coaps://lwm2m-server.edgeiq.io:5684"

#define ENDPOINT "my-cool-zephyr-device" // use the same value as your edgeIQ device "unique_id" and "server psk id"
#define PSK "000102030405060708090a0b0c0d0e0f" // use the same value as your edgeIQ device "server psk"

#define WIFI_MGMT_EVENTS (NET_EVENT_WIFI_SCAN_RESULT |		\
				NET_EVENT_WIFI_SCAN_DONE |		\
				NET_EVENT_WIFI_CONNECT_RESULT |		\
				NET_EVENT_WIFI_DISCONNECT_RESULT)

static struct net_mgmt_event_callback cb;
static struct k_sem net_cb_sem;


static struct lwm2m_ctx client;

#define CLIENT_MANUFACTURER	"Zephyr"
#define CLIENT_MODEL_NUMBER	"OMA-LWM2M Sample Client"
#define CLIENT_SERIAL_NUMBER	"345000123"
#define CLIENT_FIRMWARE_VER	"1.0"
#define CLIENT_DEVICE_TYPE	"OMA-LWM2M Client"
#define CLIENT_HW_VER		"1.0.1"

static uint8_t bat_idx = LWM2M_DEVICE_PWR_SRC_TYPE_BAT_INT;
static int bat_mv = 3800;
static int bat_ma = 125;
static uint8_t usb_idx = LWM2M_DEVICE_PWR_SRC_TYPE_USB;
static int usb_mv = 5000;
static int usb_ma = 900;
static uint8_t bat_level = 95;
static uint8_t bat_status = LWM2M_DEVICE_BATTERY_STATUS_CHARGING;
static int mem_free = 15;
static int mem_total = 25;

static void Wifi_check_connect_result( struct net_if *iface, struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status = (const struct wifi_status *) cb->info;
	if (!status->status) {
		LOG_INF("WiFi connected");
		k_sem_give(&net_cb_sem);
	} else {
		LOG_ERR("WiFi connection error");
	}
}

void Wifi_event_listener( struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface ) {
	switch( mgmt_event ) {
		case NET_EVENT_WIFI_CONNECT_RESULT:
			Wifi_check_connect_result( iface, cb );
			break;
	}
}

int wifi_autoconnect() {

	struct wifi_connect_req_params wifi_args;
	wifi_args.security = WIFI_SECURITY_TYPE_PSK;
	wifi_args.channel = WIFI_CHANNEL_ANY;
	wifi_args.psk = AUTO_CONNECT_SSID_PSK;
	wifi_args.psk_length = strlen(AUTO_CONNECT_SSID_PSK);
	wifi_args.ssid = AUTO_CONNECT_SSID;
	wifi_args.ssid_length = strlen(AUTO_CONNECT_SSID);


	// Init semaphore
	k_sem_init(&net_cb_sem, 0, 1);

	// Configure Callback
	net_mgmt_init_event_callback(&cb, Wifi_event_listener, WIFI_MGMT_EVENTS );
	net_mgmt_add_event_callback(&cb);

	// Connect interface to network
	struct net_if *iface = net_if_get_default();
	if( net_mgmt( NET_REQUEST_WIFI_CONNECT, iface, &wifi_args, sizeof(wifi_args) ) ) {
		LOG_ERR("Failed to request connection to SSID "AUTO_CONNECT_SSID);
	}

	// Wait for connection.....
	k_sem_take(&net_cb_sem, K_FOREVER );

	printk("\r\nSuccessful connected to SSID:["AUTO_CONNECT_SSID"]\r\n");

	return 0;
}

#define LIGHT_NAME_1 "User LED 1"
#define LIGHT_NAME_2 "User LED 2"

static const struct gpio_dt_spec led1_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_1), gpios);
static uint32_t led1_state;

static const struct gpio_dt_spec led2_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(green_led_2), gpios);
static uint32_t led2_state;

static int led1_on_off_cb(uint16_t obj_inst_id, uint16_t res_id, uint16_t res_inst_id, uint8_t *data,
			 uint16_t data_len, bool last_block, size_t total_size)
{
	int ret = 0;
	uint32_t led_val;

	led_val = *(uint8_t *)data;
	if (led_val != led1_state) {
		ret = gpio_pin_set_dt(&led1_gpio, (int)led_val);
		if (ret) {
			LOG_ERR("Fail to write to GPIO %d", led1_gpio.pin);
			return ret;
		}
		led1_state = led_val;
		lwm2m_set_s32(&LWM2M_OBJ(3311, 0, 5852), 0);
	}
	return ret;
}

static int led2_on_off_cb(uint16_t obj_inst_id, uint16_t res_id, uint16_t res_inst_id, uint8_t *data,
			 uint16_t data_len, bool last_block, size_t total_size)
{
	int ret = 0;
	uint32_t led_val;

	led_val = *(uint8_t *)data;
	if (led_val != led2_state) {
		ret = gpio_pin_set_dt(&led2_gpio, (int)led_val);
		if (ret) {
			LOG_ERR("Fail to write to GPIO %d", led2_gpio.pin);
			return ret;
		}
		led2_state = led_val;
		lwm2m_set_s32(&LWM2M_OBJ(3311, 1, 5852), 0);
	}
	return ret;
}

int init_led_device(void)
{
	int ret;

	if (!device_is_ready(led1_gpio.port)) {
		LOG_ERR("led1 not ready");
		return -ENODEV;
	}

	if (!device_is_ready(led2_gpio.port)) {
		LOG_ERR("led2 not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&led1_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("gpio_pin_configure_dt error %d for led1", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&led2_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("gpio_pin_configure_dt error %d for led2", ret);
		return ret;
	}

	lwm2m_create_object_inst(&LWM2M_OBJ(3311, 0));
	lwm2m_register_post_write_callback(&LWM2M_OBJ(3311, 0, 5850), led1_on_off_cb);
	lwm2m_set_res_buf(&LWM2M_OBJ(3311, 0, 5750), LIGHT_NAME_1, sizeof(LIGHT_NAME_1),
			  sizeof(LIGHT_NAME_1), LWM2M_RES_DATA_FLAG_RO);

	lwm2m_create_object_inst(&LWM2M_OBJ(3311, 1));
	lwm2m_register_post_write_callback(&LWM2M_OBJ(3311, 1, 5850), led2_on_off_cb);
	lwm2m_set_res_buf(&LWM2M_OBJ(3311, 1, 5750), LIGHT_NAME_2, sizeof(LIGHT_NAME_2),
			  sizeof(LIGHT_NAME_2), LWM2M_RES_DATA_FLAG_RO);

	return 0;
}

static struct k_work_delayable temp_work;
	#define PERIOD K_SECONDS(10)

static void temp_work_cb(struct k_work *work)
{
	const struct device *const hts221 = DEVICE_DT_GET_ONE(st_hts221);
	struct sensor_value val;

	if (!hts221) {
		LOG_ERR("%s: device not ready.", hts221->name);
		goto out;
	}
	if (sensor_sample_fetch(hts221)) {
		LOG_ERR("temperature data update failed");
		goto out;
	}
	sensor_channel_get(hts221, SENSOR_CHAN_AMBIENT_TEMP, &val);
	lwm2m_set_f64(&LWM2M_OBJ(3303, 0, 5700), sensor_value_to_double(&val));

	sensor_channel_get(hts221, SENSOR_CHAN_HUMIDITY, &val);
	lwm2m_set_f64(&LWM2M_OBJ(3304, 0, 5700), sensor_value_to_double(&val));

out:
	k_work_schedule(&temp_work, PERIOD);
}

void init_temp_sensor(void)
{
	if (lwm2m_create_object_inst(&LWM2M_OBJ(3303, 0)) == 0 && lwm2m_create_object_inst(&LWM2M_OBJ(3304, 0)) == 0) {
		k_work_init_delayable(&temp_work, temp_work_cb);
		k_work_schedule(&temp_work, K_NO_WAIT);
	}
}

// sw0 is an alias to the "User" button
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data;

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	int val = gpio_pin_get_dt(&button);
	printk("User button pressed at %" PRIu32 " with pins %d\n", k_cycle_get_32(), val);

	lwm2m_set_bool(&LWM2M_OBJ(3347, 0, 5500), val == 1);
}


void init_button(void) {
	int ret;
	if ( (ret = lwm2m_create_object_inst(&LWM2M_OBJ(3347, 0))) != 0 ) {
		LOG_ERR("Error creating the LWM2M button object %d",ret);
	}

	/* configure button */
	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Error: button device %s is not ready",
		       button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d",
		       ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d",
			ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	LOG_INF("Set up button at %s pin %d", button.port->name, button.pin);
}



static int device_reboot_cb(uint16_t obj_inst_id,
			    uint8_t *args, uint16_t args_len)
{
	LOG_WRN("DEVICE: REBOOT");
	return 0;
}

static int device_factory_default_cb(uint16_t obj_inst_id,
				     uint8_t *args, uint16_t args_len)
{
	LOG_WRN("DEVICE: FACTORY DEFAULT\n");
	return 0;
}

static int lwm2m_setup()
{
	lwm2m_set_string(&LWM2M_OBJ(0, 0, 0), LWM2M_SERVER);

	/* Security Mode */
	lwm2m_set_u8(&LWM2M_OBJ(0, 0, 2), 0 /* PSK */);

	lwm2m_set_string(&LWM2M_OBJ(0, 0, 3), ENDPOINT); /* PSK IDENTITY */
	if (sizeof(PSK) > 1) {
		char psk[1 + sizeof(PSK) / 2];
		/* Need to skip the nul terminator from string */
		size_t len = hex2bin(PSK, sizeof(PSK) - 1, psk,
				     sizeof(psk));
		if (len <= 0) {
			return -EINVAL;
		}
		lwm2m_set_opaque(&LWM2M_OBJ(0, 0, 5), (void *)psk, len);
	}

	lwm2m_set_u16(&LWM2M_OBJ(0,0,10), 1);

    lwm2m_set_u16(&LWM2M_OBJ(1,0,0), 1);
	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 0), CLIENT_MANUFACTURER, sizeof(CLIENT_MANUFACTURER),
		sizeof(CLIENT_MANUFACTURER), LWM2M_RES_DATA_FLAG_RO);

	lwm2m_set_res_buf(&LWM2M_OBJ(3,0,1), CLIENT_MODEL_NUMBER, sizeof(CLIENT_MODEL_NUMBER),
		sizeof(CLIENT_MODEL_NUMBER), LWM2M_RES_DATA_FLAG_RO);

	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 2), CLIENT_SERIAL_NUMBER, sizeof(CLIENT_SERIAL_NUMBER),
		sizeof(CLIENT_SERIAL_NUMBER), LWM2M_RES_DATA_FLAG_RO);

	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 3), CLIENT_FIRMWARE_VER, sizeof(CLIENT_FIRMWARE_VER),
		sizeof(CLIENT_FIRMWARE_VER), LWM2M_RES_DATA_FLAG_RO);

	lwm2m_register_exec_callback(&LWM2M_OBJ(3, 0, 4), device_reboot_cb);
	lwm2m_register_exec_callback(&LWM2M_OBJ(3, 0, 5), device_factory_default_cb);

	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 9), &bat_level, sizeof(bat_level), sizeof(bat_level), 0);
	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 10), &mem_free, sizeof(mem_free), sizeof(mem_free), 0);
	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 17), CONFIG_BOARD, sizeof(CONFIG_BOARD),
			  sizeof(CONFIG_BOARD), LWM2M_RES_DATA_FLAG_RO);
	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 18), CLIENT_HW_VER, sizeof(CLIENT_HW_VER),
			  sizeof(CLIENT_HW_VER), LWM2M_RES_DATA_FLAG_RO);
	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 20), &bat_status, sizeof(bat_status),
			  sizeof(bat_status), 0);
	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 21), &mem_total, sizeof(mem_total),
			  sizeof(mem_total), 0);

	/* add power source resource instances */
	lwm2m_create_res_inst(&LWM2M_OBJ(3, 0, 6, 0));
	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 6, 0), &bat_idx, sizeof(bat_idx), sizeof(bat_idx), 0);
	lwm2m_create_res_inst(&LWM2M_OBJ(3, 0, 7, 0));
	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 7, 0), &bat_mv, sizeof(bat_mv), sizeof(bat_mv), 0);
	lwm2m_create_res_inst(&LWM2M_OBJ(3, 0, 8, 0));
	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 8, 0), &bat_ma, sizeof(bat_ma), sizeof(bat_ma), 0);
	lwm2m_create_res_inst(&LWM2M_OBJ(3, 0, 6, 1));
	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 6, 1), &usb_idx, sizeof(usb_idx), sizeof(usb_idx), 0);
	lwm2m_create_res_inst(&LWM2M_OBJ(3, 0, 7, 1));
	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 7, 1), &usb_mv, sizeof(usb_mv), sizeof(usb_mv), 0);
	lwm2m_create_res_inst(&LWM2M_OBJ(3, 0, 8, 1));
	lwm2m_set_res_buf(&LWM2M_OBJ(3, 0, 8, 1), &usb_ma, sizeof(usb_ma), sizeof(usb_ma), 0);

	/* setup ISPO objects */
	init_temp_sensor();

	init_led_device();

	init_button();

	return 0;
}

static void rd_client_event(struct lwm2m_ctx *client,
			    enum lwm2m_rd_client_event client_event)
{
	switch (client_event) {

	case LWM2M_RD_CLIENT_EVENT_NONE:
		/* do nothing */
		break;

	case LWM2M_RD_CLIENT_EVENT_BOOTSTRAP_REG_FAILURE:
		LOG_DBG("Bootstrap registration failure!");
		break;

	case LWM2M_RD_CLIENT_EVENT_BOOTSTRAP_REG_COMPLETE:
		LOG_DBG("Bootstrap registration complete");
		break;

	case LWM2M_RD_CLIENT_EVENT_BOOTSTRAP_TRANSFER_COMPLETE:
		LOG_DBG("Bootstrap transfer complete");
		break;

	case LWM2M_RD_CLIENT_EVENT_REGISTRATION_FAILURE:
		LOG_DBG("Registration failure!");
		break;

	case LWM2M_RD_CLIENT_EVENT_REGISTRATION_COMPLETE:
		LOG_DBG("Registration complete");
		break;

	case LWM2M_RD_CLIENT_EVENT_REG_TIMEOUT:
		LOG_DBG("Registration timeout!");
		break;

	case LWM2M_RD_CLIENT_EVENT_REG_UPDATE_COMPLETE:
		LOG_DBG("Registration update complete");
		break;

	case LWM2M_RD_CLIENT_EVENT_DEREGISTER_FAILURE:
		LOG_DBG("Deregister failure!");
		break;

	case LWM2M_RD_CLIENT_EVENT_DISCONNECT:
		LOG_DBG("Disconnected");
		break;

	case LWM2M_RD_CLIENT_EVENT_QUEUE_MODE_RX_OFF:
		LOG_DBG("Queue mode RX window closed");
		break;

	case LWM2M_RD_CLIENT_EVENT_ENGINE_SUSPENDED:
		LOG_DBG("LwM2M engine suspended");
		break;

	case LWM2M_RD_CLIENT_EVENT_NETWORK_ERROR:
		LOG_ERR("LwM2M engine reported a network error.");
		lwm2m_rd_client_stop(client, rd_client_event, true);
		break;
	}
}

static void observe_cb(enum lwm2m_observe_event event,
		       struct lwm2m_obj_path *path, void *user_data)
{
	char buf[LWM2M_MAX_PATH_STR_SIZE];

	switch (event) {

	case LWM2M_OBSERVE_EVENT_OBSERVER_ADDED:
		LOG_INF("Observer added for %s", lwm2m_path_log_buf(buf, path));
		break;

	case LWM2M_OBSERVE_EVENT_OBSERVER_REMOVED:
		LOG_INF("Observer removed for %s", lwm2m_path_log_buf(buf, path));
		break;

	case LWM2M_OBSERVE_EVENT_NOTIFY_ACK:
		LOG_INF("Notify acknowledged for %s", lwm2m_path_log_buf(buf, path));
		break;

	case LWM2M_OBSERVE_EVENT_NOTIFY_TIMEOUT:
		LOG_INF("Notify timeout for %s, trying registration update",
			lwm2m_path_log_buf(buf, path));

		lwm2m_rd_client_update();
		break;
	}
}


void main(void)
{
	printk("Zephyr Wifi/LWM2M app connecting %s\n", CLIENT_FIRMWARE_VER);
	// setup wifi
	wifi_autoconnect();

	printk("WiFi done\nSetting up LWM2M\n");

	int ret = lwm2m_setup();

	if (ret <0) {
		printk("errror setting up LWM2M\n");
	}

	(void)memset(&client, 0x0, sizeof(client));
	client.tls_tag = 1;

	/* client.sec_obj_inst is 0 as a starting point */
	lwm2m_rd_client_start(&client, ENDPOINT, 0, rd_client_event, observe_cb);

	while (1) {
		k_sleep(K_MSEC(10));
	}
}
