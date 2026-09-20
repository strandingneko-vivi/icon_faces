#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(le_hts_svc, LOG_LEVEL_DBG);

/* Temperature Measurement: flags + IEEE-11073 32-bit float (3B mantissa + 1B exponent) */
struct temp_measurement {
	/* bit0: 0=Celsius 1=Fahrenheit, 
	* bit1: timestamp present, bit2: type present
	*/
	uint8_t flags; 
	uint8_t temp[4];
} __packed;

static atomic_t ht_notify_enabled = ATOMIC_INIT(false);
static struct temp_measurement measurement = {
	.flags = 0x00, /* Celsius, no timestamp, no type */
};
static const uint8_t temp_type = 2; /* 1=Armpit 2=Body(general) 3=Ear ... 9=Tympanum */

/* value = val1 + val2 * 1e-6 (degC), encode as mantissa * 10^-2 */
static void encode_temp(const struct sensor_value *val)
{
	int32_t mantissa = val->val1 * 100 + val->val2 / 10000;

	measurement.temp[0] = mantissa & 0xFF;
	measurement.temp[1] = (mantissa >> 8) & 0xFF;
	measurement.temp[2] = (mantissa >> 16) & 0xFF;
	measurement.temp[3] = 0xFE; /* exponent = -2 */
}

static ssize_t read_measurement(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &measurement, sizeof(measurement));
}

static ssize_t read_temp_type(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			      uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &temp_type, sizeof(temp_type));
}

static void measurement_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	atomic_set(&ht_notify_enabled, (value == BT_GATT_CCC_NOTIFY));
}

/* Health Thermometer Service */
BT_GATT_SERVICE_DEFINE(
	le_hts_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_HTS),
	BT_GATT_CHARACTERISTIC(BT_UUID_HTS_MEASUREMENT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ_ENCRYPT, read_measurement, NULL, NULL),
	BT_GATT_CCC(measurement_ccc_changed,
		    BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
	BT_GATT_CHARACTERISTIC(BT_UUID_HTS_TEMP_TYP, BT_GATT_CHRC_READ, BT_GATT_PERM_READ_ENCRYPT,
			       read_temp_type, NULL, NULL), );

/* Temperature sensor thread */
#define TEMP_THREAD_SIZE 1024
#define TEMP_THREAD_PRIO 7

static const struct device *sht3x = DEVICE_DT_GET(DT_NODELABEL(sht3xd));
static const struct device *nrf_temp = DEVICE_DT_GET(DT_NODELABEL(temp));

static struct sensor_value temp, hum;

enum {
	TEMP_SHT3X = 0,
	TEMP_NRF = 1,
};

static void temp_thread(void *p1, void *p2, void *p3)
{
	const struct device *sensor_dev;
	int sensor;

	if (device_is_ready(sht3x)) {
		sensor = TEMP_SHT3X;
		LOG_DBG("use TEMP: SHT3X");
	} else if (device_is_ready(nrf_temp)) {
		sensor = TEMP_NRF;
		LOG_DBG("use TEMP: NRF_TEMP");
	} else {
		LOG_ERR("no temp sensor ready (SHT3X / NRF_TEMP)");
		return;
	}
	sensor_dev = (sensor == TEMP_SHT3X) ? sht3x : nrf_temp;

	while (1) {
		if (sensor_sample_fetch(sensor_dev) == 0) {
			if (sensor == TEMP_SHT3X) {
				sensor_channel_get(sensor_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
				sensor_channel_get(sensor_dev, SENSOR_CHAN_HUMIDITY, &hum);
			} else {
				sensor_channel_get(sensor_dev, SENSOR_CHAN_DIE_TEMP, &temp);
			}
			encode_temp(&temp);

			// LOG_INF("Temp: %d.%06d degC, Humidity: %d.%06d %%", temp.val1, temp.val2,
			// 	hum.val1, hum.val2);

			if (atomic_get(&ht_notify_enabled)) {
				bt_gatt_notify_uuid(NULL, BT_UUID_HTS_MEASUREMENT, le_hts_svc.attrs, &measurement, sizeof(measurement));
			}
		} else {
			LOG_ERR("sample fetch failed: %s", sensor == TEMP_SHT3X ? "SHT3X" : "NRF_TEMP");
		}

		k_sleep(K_MSEC(1000));
	}
}

K_THREAD_DEFINE(temp_tid, TEMP_THREAD_SIZE, temp_thread, NULL, NULL, NULL,
		K_PRIO_PREEMPT(TEMP_THREAD_PRIO), 0, 0);
