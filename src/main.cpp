#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <STM32FreeRTOS.h>
#include <string.h>
#include <IWatchdog.h> // STM32 Hardware Watchdog

// Serial1/2/3/6 instansları platformio.ini-dəki -DENABLE_HWSERIALx bayraqları ilə
// core tərəfindən rəsmi olaraq yaradılır; burada əl ilə tərif YOXDUR.

// Watchdog-u bəsləyərək bloklanan gecikmə.
// setup()-ın uzun gecikmələrində (2 saniyəlik) watchdog reset-inin qarşısını alır.
static void wd_delay(uint32_t ms) {
  uint32_t start = millis();
  while ((uint32_t)(millis() - start) < ms) {
    IWatchdog.reload();
    delay(10);
  }
}

#define BUZZER_PIN PE9
#define PIN_APOGEE PD12    
#define PIN_MAIN   PD13    

#define RECIPROCAL_G 0.1019368f       
#define RAD2DEG 57.295779513f         
#define GRAVITY 9.80665f

// --- UÇUŞ MƏRHƏLƏLƏRİ (STATE MACHINE) ---
enum FlightState : uint8_t {
  FS_STANDBY = 0,    // Yerdə gözləyir
  FS_LAUNCHED = 1,   // QALXIŞ BAŞLADI (Motor atəşləndi)
  FS_APOGEE = 2,     // Zirvə nöqtəsi, Drogue paraşütü açıldı
  FS_MAIN = 3,       // Əsas paraşüt açıldı
  FS_LANDED = 4      // Təhlükəsiz yerə endi
};

// --- SENSOR DATA STRUCT ---
struct SensorData {
  float qw=1, qx=0, qy=0, qz=0;
  float bme_temp=0, bme_hum=0, bme_pres=0;
  float aht_temp=0, aht_hum=0;
  float altitude=0;
  float max_altitude=0; // Race condition sığortası
  float gps_lat=0.0f, gps_lon=0.0f, gps_alt=0.0f;
  bool  gps_fix=false;
  uint8_t gps_sats=0;
  float accel_x=0, accel_y=0, accel_z=0;
  float angle_x=0, angle_y=0, angle_z=0;
  float total_g = 1.0f;
  float vertical_speed = 0.0f;
  
  bool bno_ok = false, bme_ok = false, aht_ok = false;
  bool apogee_fired = false;
  bool main_fired = false;
  bool is_landed = false;
  FlightState state = FS_STANDBY; 
};

// --- KALMAN 1D (Səs-küy süzücü) ---
struct Kalman1D {
  float Q, R, x, P; bool ready = false;
  void init(float q, float r) { Q=q; R=r; P=1.0f; }
  float update(float z) {
    if (!ready) { x = z; ready = true; return x; }
    P += Q; const float K = P / (P + R);
    x += K * (z - x); P = (1.0f - K) * P; return x;
  }
};

SensorData shared;
SemaphoreHandle_t i2c_mutex, data_mutex;
Kalman1D kf_bme_temp, kf_bme_hum, kf_bme_pres, kf_aht_temp, kf_aht_hum, kf_altitude, kf_vspeed;

// CANLILIQ TAYMERİ (Watchdog qoruyucusu üçün)
volatile uint32_t last_sensor_time = 0; 

// ======================== I2C DRIVERS ========================
bool i2c_write(uint8_t addr, uint8_t reg, uint8_t data) { 
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(data); 
  return Wire.endTransmission() == 0; 
}

bool i2c_read_buf(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

struct I2CHealth { uint8_t fails = 0; uint32_t next_try = 0; };
static inline bool i2c_should_try(const I2CHealth &h, uint32_t now) { return h.fails < 3 || (int32_t)(now - h.next_try) >= 0; }
static inline void i2c_mark_result(I2CHealth &h, bool ok, uint32_t now) {
  if (ok) { h.fails = 0; return; }
  if (h.fails < 255) h.fails++;
  if (h.fails >= 3) h.next_try = now + 1000;
}
static I2CHealth health_bno, health_bme, health_aht;

// --- BME280 ---
#define BME280_ADDR 0x76
struct { uint16_t T1; int16_t T2, T3; uint16_t P1; int16_t P2,P3,P4,P5,P6,P7,P8,P9; uint8_t H1,H3; int16_t H2,H4,H5; int8_t H6; } bme_cal;

void bme280_init() { 
  uint8_t buf[26]; i2c_read_buf(BME280_ADDR, 0x88, buf, 26);
  bme_cal.T1=buf[1]<<8|buf[0]; bme_cal.T2=buf[3]<<8|buf[2]; bme_cal.T3=buf[5]<<8|buf[4]; 
  bme_cal.P1=buf[7]<<8|buf[6]; bme_cal.P2=buf[9]<<8|buf[8]; bme_cal.P3=buf[11]<<8|buf[10]; 
  bme_cal.P4=buf[13]<<8|buf[12]; bme_cal.P5=buf[15]<<8|buf[14]; bme_cal.P6=buf[17]<<8|buf[16]; 
  bme_cal.P7=buf[19]<<8|buf[18]; bme_cal.P8=buf[21]<<8|buf[20]; bme_cal.P9=buf[23]<<8|buf[22]; 
  bme_cal.H1=buf[25]; 
  uint8_t hb[7]; i2c_read_buf(BME280_ADDR,0xE1,hb,7);
  bme_cal.H2=hb[1]<<8|hb[0]; bme_cal.H3=hb[2]; bme_cal.H4=(int16_t)(hb[3]<<4)|(hb[4]&0x0F); 
  bme_cal.H5=(int16_t)(hb[5]<<4)|(hb[4]>>4); bme_cal.H6=(int8_t)hb[6];
  i2c_write(BME280_ADDR,0xF2,0x01); i2c_write(BME280_ADDR,0xF4,0x27); i2c_write(BME280_ADDR,0xF5,0xA0);
}

bool bme280_read_all(float &temp, float &pres, float &hum) { 
  uint8_t b[8]; if (!i2c_read_buf(BME280_ADDR, 0xF7, b, 8)) return false; 
  int32_t raw_p = (b[0]<<12)|(b[1]<<4)|(b[2]>>4); int32_t raw_t = (b[3]<<12)|(b[4]<<4)|(b[5]>>4); int32_t raw_h = (b[6]<<8)|b[7]; 
  int32_t v1 = ((((raw_t>>3)-((int32_t)bme_cal.T1<<1)))*bme_cal.T2)>>11; 
  int32_t v2 = (((((raw_t>>4)-(int32_t)bme_cal.T1)*((raw_t>>4)-(int32_t)bme_cal.T1))>>12)*bme_cal.T3)>>14; 
  int32_t t_fine = v1 + v2; temp = (t_fine * 5 + 128) * 0.0000390625f;  
  int64_t p1 = (int64_t)t_fine - 128000; int64_t p2 = p1 * p1 * bme_cal.P6; p2 += ((p1 * bme_cal.P5) << 17); p2 += ((int64_t)bme_cal.P4 << 35); 
  p1 = ((p1 * p1 * bme_cal.P3) >> 8) + ((p1 * bme_cal.P2) << 12); p1 = ((((int64_t)1 << 47) + p1) * bme_cal.P1) >> 33; 
  if(p1 == 0) { pres = 0; } else {
    int64_t p = ((int64_t)(1048576 - raw_p) << 31) - p2; p = (p * 3125) / p1; 
    p1 = ((int64_t)bme_cal.P9 * (p >> 13) * (p >> 13)) >> 25; p2 = ((int64_t)bme_cal.P8 * p) >> 19; 
    pres = ((p + p1 + p2) >> 8) * 0.0000390625f; 
  }
  int32_t h1 = t_fine - 76800; 
  h1 = (((raw_h << 14) - ((int32_t)bme_cal.H4 << 20) - ((int32_t)bme_cal.H5 * h1)) + 16384) >> 15; 
  h1 = h1 * (((((((h1 * bme_cal.H6) >> 10) * (((h1 * bme_cal.H3) >> 11) + 32768)) >> 10) + 2097152) * bme_cal.H2 + 8192) >> 14); 
  h1 = h1 - (((((h1 >> 15) * (h1 >> 15)) >> 7) * bme_cal.H1) >> 4); h1 = constrain(h1, 0, 419430400); 
  hum = (h1 >> 12) * 0.0009765625f; 
  return true; 
}

// --- AHT20 ---
#define AHT20_ADDR 0x38
void aht20_trigger() { Wire.beginTransmission(AHT20_ADDR); Wire.write(0xAC); Wire.write(0x33); Wire.write(0x00); Wire.endTransmission(); }
bool aht20_read(float &temp, float &hum) { 
  uint8_t buf[6]; if (Wire.requestFrom((uint8_t)AHT20_ADDR, (uint8_t)6) != 6) return false; 
  for (uint8_t i=0; i<6; i++) buf[i]=Wire.read(); 
  uint32_t hr=((uint32_t)buf[1]<<12)|((uint32_t)buf[2]<<4)|(buf[3]>>4); 
  uint32_t tr=((uint32_t)(buf[3]&0x0F)<<16)|((uint32_t)buf[4]<<8)|buf[5]; 
  hum = hr * 0.00009536743f; temp = tr * 0.00019073486f - 50.0f; return true; 
}

// --- BNO055 ---
#define BNO055_ADDR 0x28
void bno055_init() { i2c_write(BNO055_ADDR,0x3D,0x00); delay(25); i2c_write(BNO055_ADDR,0x3E,0x00); i2c_write(BNO055_ADDR,0x3B,0x00); delay(10); i2c_write(BNO055_ADDR,0x3D,0x0C); delay(20); }
bool bno055_read_quat(float &qw, float &qx, float &qy, float &qz) { 
  uint8_t buf[8]; if (!i2c_read_buf(BNO055_ADDR,0x20,buf,8)) return false; 
  qw=(int16_t)(buf[1]<<8|buf[0]) * 0.00006103515625f; qx=(int16_t)(buf[3]<<8|buf[2]) * 0.00006103515625f; 
  qy=(int16_t)(buf[5]<<8|buf[4]) * 0.00006103515625f; qz=(int16_t)(buf[7]<<8|buf[6]) * 0.00006103515625f; 
  return true; 
}
bool bno055_read_accel(float &ax, float &ay, float &az) { 
  uint8_t buf[6]; if (!i2c_read_buf(BNO055_ADDR,0x08,buf,6)) return false; 
  ax=(int16_t)(buf[1]<<8|buf[0]) * 0.01f; ay=(int16_t)(buf[3]<<8|buf[2]) * 0.01f; az=(int16_t)(buf[5]<<8|buf[4]) * 0.01f; return true; 
}
bool bno055_read_euler(float &ex, float &ey, float &ez) { 
  uint8_t buf[6]; if (!i2c_read_buf(BNO055_ADDR,0x1A,buf,6)) return false; 
  ex=(int16_t)(buf[1]<<8|buf[0]) * 0.0625f; ey=(int16_t)(buf[3]<<8|buf[2]) * 0.0625f; ez=(int16_t)(buf[5]<<8|buf[4]) * 0.0625f; return true; 
}

// ======================== GPS & ALTITUDE ========================
static float p_ref = 1013.25f;
float calc_altitude(float pres_hPa) { return 44330.0f * (1.0f - powf(pres_hPa / p_ref, 0.190284f)); }

void init_altitude_ref() {
  float sum = 0; int good = 0; const int N = 30;
  for (int i = 0; i < N; i++) { float t, p, h; if (bme280_read_all(t, p, h)) { sum += p; good++; } wd_delay(50); }
  p_ref = good > 0 ? sum / good : p_ref;
}

bool verify_nmea_checksum(const char* nmea) {
  if (nmea[0] != '$') return false; uint8_t sum = 0; int i = 1;
  while (nmea[i] != '*' && nmea[i] != '\0' && i < 90) { sum ^= (uint8_t)nmea[i++]; }
  if (nmea[i] == '*') { long expected = strtol(&nmea[i+1], NULL, 16); return (sum == expected); }
  return false;
}

void parse_gps_line(char *line) {
  if (!verify_nmea_checksum(line)) return; 
  if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
    int field_idx = 0; char *saveptr;
    char *token = strtok_r(line, ",", &saveptr); 
    
    float raw_lat = 0, raw_lon = 0, gps_alt = 0; 
    char lat_dir = 'N', lon_dir = 'E'; 
    int fix_quality = 0, sats = 0;
    
    while (token != NULL) {
      if (field_idx == 6) fix_quality = atoi(token); 
      else if (field_idx == 7) sats = atoi(token); 
      else if (field_idx == 2) raw_lat = atof(token); 
      else if (field_idx == 3) lat_dir = token[0]; 
      else if (field_idx == 4) raw_lon = atof(token); 
      else if (field_idx == 5) lon_dir = token[0]; 
      else if (field_idx == 9) gps_alt = atof(token); 
      field_idx++;
      token = strtok_r(NULL, ",", &saveptr);
    }
    
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    shared.gps_sats = sats;
    if (fix_quality > 0) {
      int deg_lat = (int)(raw_lat * 0.01f); float min_lat = raw_lat - (deg_lat * 100); float lat = deg_lat + (min_lat * 0.016666667f); if (lat_dir == 'S') lat = -lat;
      int deg_lon = (int)(raw_lon * 0.01f); float min_lon = raw_lon - (deg_lon * 100); float lon = deg_lon + (min_lon * 0.016666667f); if (lon_dir == 'W') lon = -lon;
      shared.gps_lat = lat; shared.gps_lon = lon; shared.gps_alt = gps_alt; shared.gps_fix = true; 
    } else { shared.gps_fix = false; }
    xSemaphoreGive(data_mutex);
  }
}

void play_buzzer_tone(int frequency, int duration_ms) { tone(BUZZER_PIN, frequency, duration_ms); }

// ════════════════════════════════════════════════════════════
//  BİNARY TELEMETRİYA VƏ RF PROTOKOLU (CRC16)
// ════════════════════════════════════════════════════════════
#define RF_PKT_SYNC1        0xAA
#define RF_PKT_SYNC2        0x55
#define RF_PROTO_VERSION    0x01
#define RF_DEVICE_ID        0xAA 
#define RF_PKT_TELEM        0x01
#define RF_PKT_STATUS       0x02

static uint16_t rf_seq = 0;

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void put_u8(uint8_t* buf, size_t &idx, uint8_t v) { buf[idx++] = v; }
static void put_u16(uint8_t* buf, size_t &idx, uint16_t v) { memcpy(buf + idx, &v, sizeof(v)); idx += sizeof(v); }
static void put_i16(uint8_t* buf, size_t &idx, int16_t v) { memcpy(buf + idx, &v, sizeof(v)); idx += sizeof(v); }
static void put_u32(uint8_t* buf, size_t &idx, uint32_t v) { memcpy(buf + idx, &v, sizeof(v)); idx += sizeof(v); }
static void put_i32(uint8_t* buf, size_t &idx, int32_t v) { memcpy(buf + idx, &v, sizeof(v)); idx += sizeof(v); }

static int16_t f_i16(float v, float scale) {
    if (isnan(v) || isinf(v)) return 0; 
    return (int16_t)fmaxf(fminf(v * scale, 32767.0f), -32768.0f);
}
static uint16_t f_u16(float v, float scale) {
    if (isnan(v) || isinf(v) || v < 0.0f) return 0; 
    return (uint16_t)fminf(v * scale, 65535.0f);
}
static int32_t f_i32(float v, float scale) {
    if (isnan(v) || isinf(v)) return 0; 
    return (int32_t)fmax(fmin((double)v * scale, 2147483647.0), -2147483648.0);
}

static void rf_write_packet(uint8_t type, const uint8_t* payload, uint8_t len) {
    uint8_t buf[128]; size_t i = 0; 
    if (len > sizeof(buf) - 14) len = sizeof(buf) - 14;
    put_u8(buf, i, RF_PKT_SYNC1); put_u8(buf, i, RF_PKT_SYNC2); 
    put_u8(buf, i, RF_PROTO_VERSION); put_u8(buf, i, RF_DEVICE_ID);
    put_u8(buf, i, type); put_u16(buf, i, rf_seq++); 
    put_u32(buf, i, millis()); put_u8(buf, i, len);
    if (len > 0) { memcpy(buf + i, payload, len); i += len; }
    put_u16(buf, i, crc16_ccitt(buf, i));
    Serial3.write(buf, i);
}

void tx_task(void*) {
  TickType_t t = xTaskGetTickCount();
  FlightState prev_tx_state = FS_STANDBY;

  for (;;) {
    SensorData snap; 
    xSemaphoreTake(data_mutex, portMAX_DELAY); 
    memcpy(&snap, &shared, sizeof(SensorData)); 
    xSemaphoreGive(data_mutex);

    if (snap.state != prev_tx_state) {
        uint8_t p[2] = {1, (uint8_t)snap.state}; 
        rf_write_packet(RF_PKT_STATUS, p, 2);
        prev_tx_state = snap.state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    uint8_t p[96]; size_t i = 0;
    uint16_t flags = 0;
    
    if (snap.bno_ok) flags |= 0x0001;
    if (snap.bme_ok) flags |= 0x0002;
    if (snap.aht_ok) flags |= 0x0004;
    if (snap.gps_fix) flags |= 0x0008;
    flags |= 0x0010; // ARMED=True
    if (snap.apogee_fired) flags |= 0x0020;
    if (snap.main_fired) flags |= 0x0040;
    if (snap.is_landed) flags |= 0x0080;

    put_u16(p, i, flags);
    put_i16(p, i, f_i16(snap.accel_x, 100.0f)); 
    put_i16(p, i, f_i16(snap.accel_y, 100.0f)); 
    put_i16(p, i, f_i16(snap.accel_z, 100.0f));
    put_i16(p, i, f_i16(snap.angle_x, 100.0f)); 
    put_i16(p, i, f_i16(snap.angle_y, 100.0f)); 
    put_i16(p, i, f_i16(snap.angle_z, 100.0f));
    put_i16(p, i, f_i16(snap.bme_temp, 100.0f));
    put_u16(p, i, f_u16(snap.bme_pres, 10.0f));
    put_i32(p, i, f_i32(snap.altitude, 100.0f)); 
    put_i16(p, i, f_i16(snap.aht_temp, 100.0f));
    put_u16(p, i, f_u16(snap.aht_hum, 100.0f));
    put_i32(p, i, (int32_t)(snap.gps_lat * 10000000.0));
    put_i32(p, i, (int32_t)(snap.gps_lon * 10000000.0));
    put_i32(p, i, f_i32(snap.gps_alt, 100.0f));
    put_u8(p, i, snap.gps_sats);
    put_i16(p, i, f_i16(snap.vertical_speed, 100.0f));
    put_u16(p, i, f_u16(snap.total_g, 1000.0f));
    put_u8(p, i, (uint8_t)snap.state);

    rf_write_packet(RF_PKT_TELEM, p, i);
    vTaskDelayUntil(&t, pdMS_TO_TICKS(66)); // 15 Hz
  }
}

// ════════════════════════════════════════════════════════════
//  RAKET: UÇUŞ NƏZARƏTÇİSİ (AEROSPACE GRADE DİNAMİKA)
// ════════════════════════════════════════════════════════════
void flight_control_task(void*) {
  TickType_t t = xTaskGetTickCount();
  static float prev_alt = 0.0f;
  static uint32_t prev_time_ms = 0;
  static uint32_t landing_counter = 0;
  static uint32_t launch_detect_counter = 0; 
  static uint32_t apogee_fire_time = 0;
  static uint32_t main_fire_time = 0;
  static float prev_landing_alt = 0.0f; 

  for (;;) {
    // SƏRT QORUYUCU: Sensorlar sağdırsa Watchdog-u bəslə!
    if (millis() - last_sensor_time < 500) {
        IWatchdog.reload(); 
    }

    uint32_t now_ms = millis();
    
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    float cur_alt = shared.altitude;
    float qw = shared.qw, qx = shared.qx, qy = shared.qy, qz = shared.qz;
    float ax = shared.accel_x, ay = shared.accel_y, az = shared.accel_z;
    bool ap_fired = shared.apogee_fired;
    bool mn_fired = shared.main_fired;
    bool is_landed = shared.is_landed;
    FlightState current_state = shared.state;
    
    if (cur_alt > shared.max_altitude) {
        shared.max_altitude = cur_alt;
    }
    float max_alt = shared.max_altitude;
    xSemaphoreGive(data_mutex);

    // --- 1. DİNAMİK dt İLƏ SÜRƏT VƏ BARO-İNERTİAL FÜZYON ---
    float dt = (now_ms - prev_time_ms) / 1000.0f;
    float v_speed = 0.0f;
    
    float gz_vector = qw*qw - qx*qx - qy*qy + qz*qz; 
    float a_world_z = 2.0f*(qx*qz - qw*qy)*ax + 2.0f*(qw*qx + qy*qz)*ay + gz_vector*az;
    float a_vert = a_world_z - GRAVITY; 

    xSemaphoreTake(data_mutex, portMAX_DELAY);
    v_speed = shared.vertical_speed; 
    if (dt > 0.001f && dt < 0.1f) { 
      float baro_vspeed = (cur_alt - prev_alt) / dt;
      float inertial_vspeed = v_speed + (a_vert * dt);
      v_speed = (inertial_vspeed * 0.95f) + (baro_vspeed * 0.05f); 
    }
    shared.vertical_speed = v_speed;
    xSemaphoreGive(data_mutex);
    
    prev_alt = cur_alt;
    prev_time_ms = now_ms;

    // --- 2. G QÜVVƏSİ VƏ TILT ---
    float accel_magnitude_sq = ax * ax + ay * ay + az * az;
    float g_force = sqrtf(accel_magnitude_sq) * RECIPROCAL_G;

    xSemaphoreTake(data_mutex, portMAX_DELAY); shared.total_g = g_force; xSemaphoreGive(data_mutex);
    float tilt_angle = acosf(fmaxf(fminf(gz_vector, 1.0f), -1.0f)) * RAD2DEG;

    // --- 3. QALXIŞ DETEKSİYASI (Sərtləşdirilmiş: Həm Təcil Həm Sürət) ---
    if (current_state == FS_STANDBY) {
        if ((g_force > 2.5f) && (v_speed > 5.0f)) { 
            launch_detect_counter++;
            if (launch_detect_counter > 5) { 
                xSemaphoreTake(data_mutex, portMAX_DELAY); 
                shared.state = FS_LAUNCHED; 
                xSemaphoreGive(data_mutex);
                current_state = FS_LAUNCHED;
                play_buzzer_tone(1000, 200);
            }
        } else { 
            launch_detect_counter = 0; 
        }
    }

    // --- 4. TƏHLÜKƏSİZLİK VƏ PİRO ATƏŞLƏMƏ ---
    if (max_alt > 50.0f && current_state != FS_STANDBY) {  
      
      // Apogee Paraşütü (Drogue)
      if (!ap_fired) {
        bool is_falling = (cur_alt < (max_alt - 3.0f)) && (v_speed < -1.0f);
        bool is_tilted = (tilt_angle > 70.0f) && (g_force < 1.5f); 

        if (is_falling || is_tilted) {
          xSemaphoreTake(data_mutex, portMAX_DELAY); 
          shared.apogee_fired = true; 
          shared.state = FS_APOGEE;
          xSemaphoreGive(data_mutex);
          current_state = FS_APOGEE;
          digitalWrite(PIN_APOGEE, HIGH); 
          apogee_fire_time = now_ms;
          play_buzzer_tone(1500, 500);
          ap_fired = true;
        }
      }
      
      // Main Paraşüt 
      if (ap_fired && !mn_fired) {
        bool reached_main_alt = (cur_alt <= 500.0f) && ((max_alt - cur_alt) > 30.0f);
        
        if (reached_main_alt) { 
          xSemaphoreTake(data_mutex, portMAX_DELAY); 
          shared.main_fired = true; 
          shared.state = FS_MAIN;
          xSemaphoreGive(data_mutex);
          current_state = FS_MAIN;
          digitalWrite(PIN_MAIN, HIGH); 
          main_fire_time = now_ms;
          play_buzzer_tone(2000, 800);
          mn_fired = true;
        }
      }

      // Yerə Oturma Deteksiyası 
      if (mn_fired && !is_landed) {
        if (fabs(v_speed) < 0.5f && fabs(cur_alt - prev_landing_alt) < 0.5f) {
          landing_counter++;
          if (landing_counter > 100) {  
            xSemaphoreTake(data_mutex, portMAX_DELAY); 
            shared.is_landed = true; 
            shared.state = FS_LANDED;
            xSemaphoreGive(data_mutex);
            current_state = FS_LANDED;
            play_buzzer_tone(3000, 2000);
            is_landed = true;
          }
        } else { 
          landing_counter = 0; 
        }
        prev_landing_alt = cur_alt;
      }
    }

    // --- 5. PİRO KANAL SÖNDÜRÜCÜ (Təhlükəsizlik) ---
    if (ap_fired && digitalRead(PIN_APOGEE) == HIGH && (now_ms - apogee_fire_time > 1500)) {
        digitalWrite(PIN_APOGEE, LOW); 
    }
    if (mn_fired && digitalRead(PIN_MAIN) == HIGH && (now_ms - main_fire_time > 1500)) {
        digitalWrite(PIN_MAIN, LOW);   
    }

    vTaskDelayUntil(&t, pdMS_TO_TICKS(20)); 
  }
}

void env_task(void*) {
  TickType_t t = xTaskGetTickCount();
  uint8_t aht_counter = 0;

  if (i2c_should_try(health_aht, millis())) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY); aht20_trigger(); xSemaphoreGive(i2c_mutex);
  }

  for (;;) {
    uint32_t now = millis();
    bool bme_ok = false; float bt=0, bp=0, bh=0;
    
    if (i2c_should_try(health_bme, now)) { 
      xSemaphoreTake(i2c_mutex, portMAX_DELAY); bme_ok = bme280_read_all(bt, bp, bh); xSemaphoreGive(i2c_mutex); 
      i2c_mark_result(health_bme, bme_ok, now); 
    }
    
    aht_counter++;
    bool aht_ok = false; float at=0, ah=0;
    if (aht_counter >= 5) { 
      aht_counter = 0; 
      if (i2c_should_try(health_aht, now)) { 
        xSemaphoreTake(i2c_mutex, portMAX_DELAY); 
        aht_ok = aht20_read(at, ah); 
        aht20_trigger(); 
        xSemaphoreGive(i2c_mutex); 
        i2c_mark_result(health_aht, aht_ok, now); 
      }
    }
    
    xSemaphoreTake(data_mutex, portMAX_DELAY); 
    shared.bme_ok = bme_ok;
    shared.aht_ok = aht_ok;
    if (bme_ok) {
      shared.bme_temp = kf_bme_temp.update(bt); shared.bme_hum = kf_bme_hum.update(bh); shared.bme_pres = kf_bme_pres.update(bp); 
      shared.altitude = kf_altitude.update(calc_altitude(shared.bme_pres)); 
    }
    if (aht_ok) { shared.aht_temp = kf_aht_temp.update(at); shared.aht_hum = kf_aht_hum.update(ah); }
    xSemaphoreGive(data_mutex);
    
    vTaskDelayUntil(&t, pdMS_TO_TICKS(20)); 
  }
}

void imu_task(void*) {
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    uint32_t now = millis();
    bool imu_ok = false;
    
    if (i2c_should_try(health_bno, now)) {
      float qw, qx, qy, qz, ax, ay, az, ex, ey, ez;
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      bool okQ = bno055_read_quat(qw, qx, qy, qz); 
      bool okA = bno055_read_accel(ax, ay, az); 
      bool okE = bno055_read_euler(ex, ey, ez);
      xSemaphoreGive(i2c_mutex);
      
      imu_ok = (okQ && okA && okE);
      i2c_mark_result(health_bno, imu_ok, now);
      
      if (imu_ok) {
        last_sensor_time = millis(); // SENSOR SAĞLAMDIR TƏSDİQİ (WATCHDOG ÜÇÜN)
        
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        shared.qw = qw; shared.qx = qx; shared.qy = qy; shared.qz = qz;
        shared.accel_x = ax; shared.accel_y = ay; shared.accel_z = az; 
        shared.angle_x = ex; shared.angle_y = ey; shared.angle_z = ez;
        xSemaphoreGive(data_mutex);
      }
    }
    
    xSemaphoreTake(data_mutex, portMAX_DELAY); 
    shared.bno_ok = imu_ok;
    xSemaphoreGive(data_mutex);
    
    vTaskDelayUntil(&t, pdMS_TO_TICKS(20));
  }
}

void gps_task(void*) {
  static char gps_buf[100]; 
  static int buf_idx = 0;
  
  for (;;) {
    while (Serial2.available()) {
      char c = Serial2.read();
      
      if (c == '$') { buf_idx = 0; }
      
      if (c == '\n' || c == '\r') { 
        if (buf_idx > 10) { 
          gps_buf[buf_idx] = '\0'; 
          parse_gps_line(gps_buf); 
        } 
        buf_idx = 0; 
      } else if (buf_idx < 99) { 
        gps_buf[buf_idx++] = c; 
      } else {
        buf_idx = 0; 
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  IWatchdog.begin(2000000); // 2 Saniyəlik Qoruyucu Watchdog
  IWatchdog.reload();

  // STM32F407 üçün standart HardwareSerial interfeyslərinin birbaşa başladılması
  Serial1.begin(115200); 
  Serial2.begin(9600); 
  Serial3.begin(115200); 
  Serial6.begin(115200);
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIN_APOGEE, OUTPUT); digitalWrite(PIN_APOGEE, LOW);
  pinMode(PIN_MAIN, OUTPUT); digitalWrite(PIN_MAIN, LOW);
  
  wd_delay(1500);
  
  Wire.begin(); 
  Wire.setClock(400000);
  
  bme280_init(); 
  wd_delay(10);
  
  wd_delay(700); 
  bno055_init(); 
  
  init_altitude_ref();
  
  kf_bme_temp.init(0.005f, 0.25f); kf_bme_hum.init(0.05f, 9.0f); kf_bme_pres.init(0.05f, 1.0f);
  kf_altitude.init(0.1f, 4.0f); kf_aht_temp.init(0.005f, 0.09f); kf_aht_hum.init(0.05f, 4.0f);
  kf_vspeed.init(0.1f, 2.0f);
  
  i2c_mutex = xSemaphoreCreateMutex(); data_mutex = xSemaphoreCreateMutex();
  
  xTaskCreate(flight_control_task, "FLIGHT", 512, NULL, 4, NULL);
  xTaskCreate(imu_task, "IMU", 512, NULL, 3, NULL);
  xTaskCreate(gps_task, "GPS", 384, NULL, 2, NULL);
  xTaskCreate(tx_task, "TX", 512, NULL, 1, NULL); 
  xTaskCreate(env_task, "ENV", 512, NULL, 1, NULL); 
  
  vTaskStartScheduler();
}

void loop() {}