#include <errno.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/device.h>
#include "adxl367.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(app);

#define BT_UUID_ACCEL_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x045b2c8b, 0x45f0, 0x4c69, 0x9d23, 0x5a4cedf0d079)
#define BT_UUID_ACCEL_MEASUREMENT_VAL \
	BT_UUID_128_ENCODE(0x045b2c8c, 0x45f0, 0x4c69, 0x9d23, 0x5a4cedf0d079)

#define BT_UUID_ACCEL_SERVICE BT_UUID_DECLARE_128(BT_UUID_ACCEL_SERVICE_VAL)
#define BT_UUID_ACCEL_MEASUREMENT BT_UUID_DECLARE_128(BT_UUID_ACCEL_MEASUREMENT_VAL)

static bool accel_notify_enabled;

static void accel_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	accel_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Acceleration notifications %s", accel_notify_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(accel_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ACCEL_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_ACCEL_MEASUREMENT,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(accel_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

static K_SEM_DEFINE(bt_ready_sem, 0, 1);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ACCEL_SERVICE_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static int start_advertising(void)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err && err != -EALREADY) {
		LOG_ERR("Advertising start failed (%d)", err);
		return err;
	}

	if (!err) {
		LOG_INF("Bluetooth advertising started");
	}

	return 0;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Connection to %s failed (0x%02x)", addr, err);
		return;
	}

	LOG_INF("Connected to %s", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected from %s (reason 0x%02x)", addr, reason);

	int err = start_advertising();
	if (err) {
		LOG_ERR("Failed to resume advertising (%d)", err);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static double adxl36x_odr_hz(const struct device *dev)
{
	static const double odr_table[] = {12.5, 25.0, 50.0, 100.0, 200.0, 400.0};
	enum adxl367_odr odr = ADXL367_ODR_12P5HZ;

	if (!device_is_ready(dev)) {
		return odr_table[odr];
	}

#if IS_ENABLED(CONFIG_ADXL367_STREAM)
	const struct adxl367_data *data = dev->data;

	odr = data->odr;
#else
	const struct adxl367_dev_config *cfg = dev->config;

	odr = cfg->odr;
#endif

	if (odr >= ARRAY_SIZE(odr_table)) {
		return odr_table[ADXL367_ODR_12P5HZ];
	}

	return odr_table[odr];
}

struct accel_sample {
	float x;
	float y;
	float z;
};

static void publish_accel(const struct accel_sample *sample)
{
	if (!accel_notify_enabled) {
		return;
	}

	int err = bt_gatt_notify(NULL, &accel_svc.attrs[2], sample, sizeof(*sample));

	if (err && err != -ENOTCONN) {
		LOG_WRN("Failed to send notification (%d)", err);
	}
}

static void bluetooth_ready(int err)
{
	if (err) {
		LOG_ERR("Bluetooth init failed (%d)", err);
		k_sem_give(&bt_ready_sem);
		return;
	}

	(void)start_advertising();

	k_sem_give(&bt_ready_sem);
}

static double accel_to_ms2(const struct sensor_value *val)
{
	return sensor_value_to_double(val);
}

static double accel_to_g(const struct sensor_value *val)
{
	const double ms2_per_g = (double)SENSOR_G / 1000000.0;

	return accel_to_ms2(val) / ms2_per_g;
}

int main(void)
{
	const struct device *dev = DEVICE_DT_GET_ANY(adi_adxl367);

	if (dev == NULL) {
		dev = DEVICE_DT_GET_ANY(adi_adxl366);
	}

	if (dev == NULL) {
		LOG_ERR("No ADXL36x device found in devicetree");
		return 0;
	}
	int err;

	if (!device_is_ready(dev)) {
		LOG_ERR("Accelerometer device is not ready");
		return 0;
	}

	LOG_INF("Accelerometer ready, streaming acceleration samples");
	LOG_INF("Accelerometer sampling rate set to %.1f Hz", adxl36x_odr_hz(dev));

	err = bt_enable(bluetooth_ready);
	if (err) {
		LOG_ERR("Bluetooth enable failed (%d)", err);
	} else {
		k_sem_take(&bt_ready_sem, K_FOREVER);
	}

	while (true) {
		struct sensor_value accel[3];
		err = sensor_sample_fetch(dev);

		if (err) {
			LOG_ERR("Sample fetch failed (%d)", err);
			k_sleep(K_MSEC(100));
			continue;
		}

		err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
		if (err) {
			LOG_ERR("Channel get failed (%d)", err);
			k_sleep(K_MSEC(100));
			continue;
		}

		LOG_INF("Accel [m/s^2]: X=%.3f Y=%.3f Z=%.3f | [g]: X=%.3f Y=%.3f Z=%.3f",
			accel_to_ms2(&accel[0]),
			accel_to_ms2(&accel[1]),
			accel_to_ms2(&accel[2]),
			accel_to_g(&accel[0]),
			accel_to_g(&accel[1]),
			accel_to_g(&accel[2]));

		const struct accel_sample sample = {
			.x = (float)accel_to_ms2(&accel[0]),
			.y = (float)accel_to_ms2(&accel[1]),
			.z = (float)accel_to_ms2(&accel[2]),
		};

		publish_accel(&sample);

		k_sleep(K_MSEC(100));
	}

	return 0;
}
