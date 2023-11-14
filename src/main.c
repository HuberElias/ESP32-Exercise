#include "bmp280.h"
#include "driver/i2c.h"

int8_t BME280_I2C_bus_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data,
                           uint16_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);

  i2c_master_write_byte(cmd, reg_addr, true);

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_READ, true);

  if (len > 1) {
    i2c_master_read(cmd, reg_addr, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, reg_addr + len - 1, I2C_MASTER_NACK);

  i2c_master_stop(cmd);

  i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);

  i2c_cmd_link_delete(cmd);
}

int8_t BMP280_I2C_bus_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data,
                            uint16_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);

  for (uint16_t i = 0; i < len; i++) {
    i2c_master_write_byte(cmd, reg_addr + i, true);
    i2c_master_write_byte(cmd, data[i], true);
  }

  i2c_master_stop(cmd);

  i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
}

void app_main() {
  i2c_config_t i2c_conf = {};

  const uint8_t dev_addr = BMP280_I2C_ADDR_PRIM;
  struct bmp280_config conf;
  struct bmp280_dev bmp = {.dev_id = dev_addr,
                           .intf = BMP280_I2C_INTF,
                           .write = BMP280_I2C_bus_write,
                           .read = BME280_I2C_bus_read};

  bmp280_init(&bmp);
  bmp280_get_config(&conf, &bmp);

  conf.filter = BMP280_FILTER_COEFF_2;
  conf.os_pres = BMP280_OS_NONE;
  conf.os_temp = BMP280_OS_1X;
  conf.odr = BMP280_ODR_1000_MS;

  bmp280_set_config(&conf, &bmp);

  bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);

  while (true) {
    struct bmp280_uncomp_data uncompData;
    bmp280_get_uncomp_data(&uncompData, &bmp);

    double temperature;
    bmp280_get_comp_temp_double(&temperature, uncompData.uncomp_temp, &bmp);

    printf("%lf\n", temperature);
  }
}
