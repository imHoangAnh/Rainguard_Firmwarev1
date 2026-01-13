/**
 * @author Luong Huu Phuc 
 * @date 2025/05/11
 * @copyright Many sources
 * @file I2C_dev.c
 */
 #include <stdio.h>
 #include <stdint.h>
 #include <string.h>
 #include <stdbool.h>
 #include <driver/i2c.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "esp_err.h"
 #include "esp_log.h"
 #include "I2C_dev.h"
 #include "sdkconfig.h"
 
 typedef struct {
     i2c_config_t config;
     bool installed; //Flag de danh dau i2c da cai dat chua
     SemaphoreHandle_t lock;
 } i2c_port_status_t;
 
 static i2c_port_status_t states[I2C_NUM_MAX];
 uint16_t i2c_dev_read_timeout = I2CDEV_DEFAULT_READ_TIMEOUT;
 
 esp_err_t i2c_dev_initialize(void){
   memset(states, 0, sizeof(states));
 
 #if !CONFIG_I2C_DEV_NOLOCK
   for(int i = 0; i < I2C_NUM_MAX; i++){
     states[i].lock = xSemaphoreCreateMutex(); //Khoi tao Mutex
     if(!states[i].lock){
       ESP_LOGE(pcTaskGetName(NULL), "Could not create port mutex: %d", i);
       return ESP_FAIL;
     }
   }
 #endif 
   return ESP_OK;
 }
 
 esp_err_t i2c_dev_deinitialize(void){
   for(int i = 0; i < I2C_NUM_MAX; i++){
     if(!states[i].lock){
       continue;
     }
     if(states[i].installed){
       SEMAPHORE_TAKE_LOCAL(i);
       i2c_driver_delete(i);
       states[i].installed = false;
       SEMAPHORE_GIVE_LOCAL(i);
     }
 #if !CONFIG_I2C_DEV_NOLOCK
       vSemaphoreDelete(states[i].lock);
 #endif
       states[i].lock = NULL;
   }
   return ESP_OK;
 }
 
 esp_err_t i2c_dev_create_mutex(I2C_dev_init_t *dev){
 #if !CONFIG_I2C_DEV_NOLOCK
   if(!dev) return ESP_ERR_INVALID_ARG;
   ESP_LOGV(pcTaskGetName(NULL), "[0x%02x at %d] creating mutex", dev->address, dev->port);
   dev->mutex = xSemaphoreCreateMutex();
 
   if(!dev->mutex){
     ESP_LOGE(pcTaskGetName(NULL), "[0x%02x at %d] Could not create device mutex", dev->address, dev->port);
     return ESP_FAIL;
   }
 
   return ESP_OK;
 
 #endif 
 }
 
 esp_err_t i2c_dev_delete_mutex(I2C_dev_init_t *dev){
 #if !CONFIG_I2C_DEV_NOLOCK
   if(!dev) return ESP_ERR_INVALID_ARG;
   ESP_LOGV(pcTaskGetName(NULL), "[0x%02x at %d] deleting mutex", dev->address, dev->port);
   vSemaphoreDelete(dev->mutex);
 #endif
   return ESP_OK;
 }
 
 esp_err_t i2c_dev_take_mutex(I2C_dev_init_t *dev){
 #if !CONFIG_I2C_DEV_NOLOCK
   if(!dev) return ESP_ERR_INVALID_ARG;
   ESP_LOGV(pcTaskGetName(NULL), "[0x%02x at %d] taking mutex", dev->address, dev->port);
   
   if(!xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(i2c_dev_read_timeout))){
     ESP_LOGE(pcTaskGetName(NULL), "[0x%02x at %d] Could not take device mutex", dev->address, dev->port);
     return ESP_ERR_TIMEOUT;
   }
 #endif 
   return ESP_OK;
 }
 
 esp_err_t i2c_dev_give_mutex(I2C_dev_init_t *dev){
 #if !CONFIG_I2C_DEV_NOLOCK
   if(!dev) return ESP_ERR_INVALID_ARG;
   ESP_LOGV(pcTaskGetName(NULL), "[0x%02x at %d] giving mutex", dev->address, dev->port);
 
   if(!xSemaphoreGive(dev->mutex)){
     ESP_LOGE(pcTaskGetName(NULL), "[0x%02x ar %d] Could not give device mutex", dev->address, dev->port);
     return ESP_FAIL;
   }
 #endif
   return ESP_OK;
 }
 
 static inline bool i2c_dev_cfg_equal(const i2c_config_t *a, const i2c_config_t *b){
   return a->scl_io_num == b->scl_io_num 
       && a->sda_io_num == b->sda_io_num
       && a->master.clk_speed == b->master.clk_speed
       && a->scl_pullup_en == b->scl_pullup_en
       && a->sda_pullup_en == b->sda_pullup_en;
 }
 
 esp_err_t i2c_dev_install_device(I2C_dev_init_t *dev){
   if(dev->port >= I2C_NUM_MAX) return ESP_ERR_INVALID_ARG; //Neu port device vuot qua nguong (toi da 2 thiet bi)
 
   esp_err_t res = ESP_OK; //Gan cho bien res = ESP_OK neu truong hop if() khong xay ra
   
   //Cau hinh lai driver neu cau hinh thay doi hoac chua duoc cai dat
   if(!i2c_dev_cfg_equal(&dev->cfg, &states[dev->port].config) || !states[dev->port].installed){
     ESP_LOGD(pcTaskGetName(NULL), "Reconfiguring I2C driver on port %d", dev->port);
     i2c_config_t temp;
     memcpy(&temp, &dev->cfg, sizeof(i2c_config_t)); //Copy config va ep vao temp
     temp.mode = I2C_MODE_MASTER;
 
     //Xoa driver cu neu da cai dat
     if(states[dev->port].installed){
       i2c_driver_delete(dev->port);
       states[dev->port].installed = false; 
     }
 
     //DEBUG
     if((res = i2c_param_config(dev->port, &temp)) != ESP_OK){
       ESP_LOGW(pcTaskGetName(NULL), "Error at i2c_param_config(): %s", esp_err_to_name(res));
       return res;
     }else{
       ESP_LOGI(pcTaskGetName(NULL), "Function i2c_param_config() check OK !");
     }
 
     //DEBUG
     if((res = i2c_driver_install(dev->port, temp.mode, 0, 0, 0)) != ESP_OK){
       ESP_LOGW(pcTaskGetName(NULL), "Error at i2c_driver_install(): %s", esp_err_to_name(res));
       return res;
     }else{
       ESP_LOGI(pcTaskGetName(NULL), "Function i2c_driver_install() check OK !");
     }
 
     states[dev->port].installed = true;
 
     memcpy(&states[dev->port].config, &temp, sizeof(i2c_config_t));
     ESP_LOGD(pcTaskGetName(NULL), "I2C driver install on port %d: check OK !", dev->port);
     ESP_LOGI(pcTaskGetName(NULL), "Function i2c_dev_install_device() check OK !");
   }
   return res;
 }
 
 int i2c_dev_read_bit(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t *data, uint16_t timeout, void *wire_obj){
   uint8_t b;
   uint8_t count = i2c_dev_read_byte(dev, dev_addr, reg_addr, &b, timeout, wire_obj);
   *data = b & (1 << bit_num);
   return count;
 }
 
 int i2c_dev_read_bits(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t *data, uint16_t timeout, void *wire_obj){
   // 01101001 read byte
   // 76543210 bit numbers
   //    xxx   args: bitStart=4, length=3
   //    010   masked
   // -> 010 shifted
   uint8_t count, b;
   if((count = i2c_dev_read_byte(dev, dev_addr, reg_addr, &b, timeout, wire_obj)) != 0){
     uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
     b &= mask;
     b >>= (bit_start - length + 1);
     *data = b;
   }
   return count;
 }
 
 int i2c_dev_read_bitw(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint16_t *data, uint16_t timeout, void *wire_obj){
   uint16_t b;
   uint8_t count = i2c_dev_read_word(dev, dev_addr, reg_addr, &b, timeout, wire_obj);
   *data = b & (1 << bit_num);
   return count;
 }
 
 int i2c_dev_read_bitsw(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint16_t *data, uint16_t timeout, void *wire_obj){
   // 1101011001101001 read byte
   // fedcba9876543210 bit numbers
   //    xxx           args: bitStart=12, length=3
   //    010           masked
   //           -> 010 shifted 
   uint8_t count;
   uint16_t w;
   if((count = i2c_dev_read_word(dev, dev_addr, reg_addr, &w, timeout, wire_obj)) != 0){
     uint16_t mask = ((1 << length) - 1) << (bit_start - length + 1);
     w &= mask;
     w >>= (bit_start - length + 1);
     *data = w;
   }
   return count;
 }
 
 esp_err_t i2c_dev_read_byte(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t timeout, void *wire_obj){
   return i2c_dev_read_bytes(dev, dev_addr, reg_addr, 1, data, timeout, wire_obj); //Length data = 1 byte
 }
 
 esp_err_t i2c_dev_read_bytes(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t *data, uint16_t timeout, void *wire_obj){
   SEMAPHORE_TAKE_LOCAL(dev->port);
 
   if(length > 0){
     i2c_dev_select_register(dev_addr, reg_addr);
   }
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   
   // i2c_master_write(cmd, &reg_addr, length, true); //Ghi lien tiep nhieu byte vao bus I2C
   //Bat dau doc
   ESP_ERR_CHECK_LOCAL(i2c_master_start(cmd));
   ESP_ERR_CHECK_LOCAL(i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true));
 
   //Neu length > 1 thi doc byte tai vi tri length - 1 truoc
   if(length > 1){
     ESP_ERR_CHECK_LOCAL(i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK));
   }
 
   //Size cua data luon bang 1 byte
   ESP_ERR_CHECK_LOCAL(i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_LAST_NACK));
   ESP_ERR_CHECK_LOCAL(i2c_master_stop(cmd));
 
   esp_err_t err = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(i2c_dev_read_timeout));
   if(err != ESP_OK){
     ESP_LOGE(pcTaskGetName(NULL), "Error at function i2c_master_cmd_begin() [0x%02x at port %d]: %d (%s)", dev_addr, dev->port, err, esp_err_to_name(err));
   }
   
   i2c_cmd_link_delete(cmd);
   SEMAPHORE_GIVE_LOCAL(dev->port);
   return err;
 }
 
 int i2c_dev_read_word(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t *data, uint16_t timeout, void *wire_obj){
   uint8_t msb[2] = {0,0};
   i2c_dev_read_bytes(dev, dev_addr,reg_addr, 2, msb, timeout, wire_obj);
   *data = (int16_t)((msb[0] << 8) | msb[1]);
   return 0;
 }
 
 int i2c_dev_read_words(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint16_t *data, uint16_t timeout, void *wire_obj){ 
   uint8_t msb[2] = {0,0};
   for(int _index = 0; _index < length; _index++){
     uint8_t _reg_addr = reg_addr + (_index * 2);
     i2c_dev_read_bytes(dev, dev_addr, _reg_addr, 2, msb, timeout, wire_obj);
     data[_index] = (int16_t)((msb[0] << 8) | msb[1]);
   }
   return length;
 }
 
 bool i2c_dev_write_bit(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint8_t data, void *wire_obj){
   uint8_t b;
   i2c_dev_read_byte(dev, dev_addr, reg_addr, &b, i2c_dev_read_timeout, wire_obj);
   b = (data != 0) ? (b | (1 << bit_num)) : (b & ~(1 << bit_num));
   return i2c_dev_write_byte(dev, dev_addr, reg_addr, b, wire_obj);
 }
 
 bool i2c_dev_write_bits(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data, void *wire_obj){
   //      010 value to write
   // 76543210 bit numbers
   //    xxx   args: bitStart=4, length=3
   // 00011100 mask byte
   // 10101111 original value (sample)
   // 10100011 original & ~mask
   // 10101011 masked | value
   uint8_t b = 0;
   if(i2c_dev_read_byte(dev, dev_addr, reg_addr, &b, i2c_dev_read_timeout, wire_obj) != ESP_OK){
     uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
     data <<= (bit_start - length + 1); //Dich data vao dung vi tri
     data &= mask; //Bien tat cac bit khong quan trong trong data thanh 0
     b &= ~(mask); //Bien tat ca cac bit quan trong trong byte dang ton tai thanh 0 
     b |= data; //Ket hop data voi byte dang ton tai
     return i2c_dev_write_byte(dev, dev_addr, reg_addr, b, wire_obj);
   }else{
     return false;
   }
 }
 
 bool i2c_dev_write_bitw(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_num, uint16_t data, void *wire_obj){
   uint16_t w;
   i2c_dev_read_word(dev, dev_addr, reg_addr, &w, i2c_dev_read_timeout, wire_obj);
   w = (data != 0) ? (w | (1 << bit_num)) : (w & ~(1 << bit_num));
   return i2c_dev_write_word(dev, dev_addr, reg_addr, w, wire_obj);
 }
 
 bool i2c_dev_write_bitsw(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint16_t data, void *wire_obj){
   //              010 value to write
   // fedcba9876543210 bit numbers
   //    xxx           args: bitStart=12, length=3
   // 0001110000000000 mask word
   // 1010111110010110 original value (sample)
   // 1010001110010110 original & ~mask
   // 1010101110010110 masked | value
   uint16_t w;
   if(i2c_dev_read_word(dev, dev_addr, reg_addr, &w, i2c_dev_read_timeout, wire_obj) != 0){
     uint16_t mask = ((1 << length) - 1) << (bit_start - length + 1);
     data <<= (bit_start - length + 1);
     data &= mask;
     w &= ~(mask);
     w |= data;
     return i2c_dev_write_word(dev, dev_addr, reg_addr, w, wire_obj);
   }else{
     return false;
   }
 }
 
 bool i2c_dev_write_byte(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t data, void *wire_obj){
   i2c_cmd_handle_t cmd;
 
   cmd = i2c_cmd_link_create();
   ESP_ERR_CHECK_LOCAL(i2c_master_start(cmd));
   ESP_ERR_CHECK_LOCAL(i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1));
   ESP_ERR_CHECK_LOCAL(i2c_master_write_byte(cmd, reg_addr, 1));
   ESP_ERR_CHECK_LOCAL(i2c_master_write_byte(cmd, data, 1));
   ESP_ERR_CHECK_LOCAL(i2c_master_stop(cmd));
   ESP_ERR_CHECK_LOCAL(i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS));
   i2c_cmd_link_delete(cmd);
 
   return true;
 }
 
 bool i2c_dev_write_bytes(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t *data, void *wire_obj){
   i2c_cmd_handle_t cmd;
 
   cmd = i2c_cmd_link_create();
   ESP_ERR_CHECK_LOCAL(i2c_master_start(cmd));
   ESP_ERR_CHECK_LOCAL(i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1));
   ESP_ERR_CHECK_LOCAL(i2c_master_write_byte(cmd, reg_addr, 1));
   ESP_ERR_CHECK_LOCAL(i2c_master_write(cmd, data, length - 1, 0));
   ESP_ERR_CHECK_LOCAL(i2c_master_write_byte(cmd, data[length - 1], 1));
   ESP_ERR_CHECK_LOCAL(i2c_master_stop(cmd));
   ESP_ERR_CHECK_LOCAL(i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS));
   i2c_cmd_link_delete(cmd);
   return true;
 }
 
 bool i2c_dev_write_word(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t data, void *wire_obj){
   uint8_t data1[] = {(uint8_t)(data >> 8), (uint8_t)(data & 0xff)};
   i2c_dev_write_bytes(dev, dev_addr, reg_addr, 2, data1, wire_obj);
   return true;
 }
 
 bool i2c_dev_write_words(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint16_t *data, void *wire_obj){
   for(int _index = 0; _index < length; _index++){
     uint8_t _reg_addr = reg_addr + (_index * 2);
     uint8_t data1[] = {(uint8_t)(data[_index >> 8]), (uint8_t)(data[_index] & 0xff)};
     i2c_dev_write_bytes(dev, dev_addr, _reg_addr, 2, data1, wire_obj);
   }
   return true;
 }
 
 void i2c_dev_select_register(uint8_t dev_addr, uint8_t reg){
   i2c_cmd_handle_t cmd;
   
   cmd = i2c_cmd_link_create();
   ESP_ERR_CHECK_LOCAL(i2c_master_start(cmd));
   ESP_ERR_CHECK_LOCAL(i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1)); //Ghi 1 byte (dia chi I2C 7-bit) duy nhat vao bus I2C
   ESP_ERR_CHECK_LOCAL(i2c_master_write_byte(cmd, reg, 1)); //Ghi 1 byte reg vao dia chi I2C (vd bien la reg)
   ESP_ERR_CHECK_LOCAL(i2c_master_stop(cmd));
   ESP_ERR_CHECK_LOCAL(i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS));
   i2c_cmd_link_delete(cmd);
 }
 
 esp_err_t i2c_dev_write_command_16(I2C_dev_init_t *dev, uint8_t dev_addr, uint16_t cmd, void *wire_obj){
   (void)(wire_obj);
 
   esp_err_t ret = ESP_OK;
   i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
 
   ret |= i2c_master_start(cmd_handle);
   ret |= i2c_master_write_byte(cmd_handle, (dev_addr << 1) | I2C_MASTER_WRITE, true);
   ret |= i2c_master_write_byte(cmd_handle, (cmd >> 8) & 0xFF, true); //MSB + bitmask (chan chan lay 8-bit thap - an toan trong lap trinh)
   ret |= i2c_master_write_byte(cmd_handle, cmd & 0xFF, true); //LSB + bitmask
   ret |= i2c_master_stop(cmd_handle);
   ret |= i2c_master_cmd_begin(I2C_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
   i2c_cmd_link_delete(cmd_handle);
 
   // uint8_t buffer[2];
   // buffer[0] = (cmd >> 8) & 0xFF; //MSB
   // buffer[1] = cmd & 0xFF; //LSB
   // ret |= i2c_master_write_to_device(I2C_NUM, dev_addr, buffer, sizeof(buffer), 1000 / portTICK_PERIOD_MS);
 
   return ret;
 }  
 
 esp_err_t i2c_dev_write_command_8(I2C_dev_init_t *dev, uint8_t dev_addr, uint16_t cmd, void *wire_obj){
   esp_err_t ret = ESP_OK; 
   i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
 
   ret |= i2c_master_start(cmd_handle);
   ret |= i2c_master_write_byte(cmd_handle, (dev_addr << 1) | I2C_MASTER_WRITE, true);
   ret |= i2c_master_write_byte(cmd_handle, cmd, true);
   ret |= i2c_master_stop(cmd_handle);
   ret |= i2c_master_cmd_begin(I2C_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
 
   i2c_cmd_link_delete(cmd_handle);
   return ret;
 }
 
 esp_err_t i2c_dev_read_data(I2C_dev_init_t *dev, uint8_t dev_addr, uint8_t *data, size_t length, void *wire_obj){
   (void)(wire_obj);
 
   esp_err_t ret = ESP_OK;
   i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
 
   ret |= i2c_master_start(cmd_handle);
   ret |= i2c_master_write_byte(cmd_handle, (dev_addr << 1) | I2C_MASTER_READ, true);
   ret |= i2c_master_read(cmd_handle, data, length, I2C_MASTER_LAST_NACK); //gui ACK cho tat ca ca byte tru byte cuoi thi gui NACK
   ret |= i2c_master_stop(cmd_handle);
   ret |= i2c_master_cmd_begin(I2C_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
   i2c_cmd_link_delete(cmd_handle);
   return ret;
   
   /**
    * @note Theo chuan I2C:
    * \note - Khi master doc nhieu byte tu slave, sau moi byte no phai gui bit `ACK` de noi: "Toi van muon doc nua"
    * \note - Nhung sau byte cuoi cung, master phai gui bit NACK de thong bao: "Toi da doc xong roi"
    */
   // ret |= i2c_master_read_from_device(I2C_NUM, dev_addr, data, length, 1000 / portTICK_PERIOD_MS);
 }