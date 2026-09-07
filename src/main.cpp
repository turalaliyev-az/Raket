// ============================================================================
//  RAKET UÇUŞ KOMPÜTERİ — SENAYE STANDARTI (AEROSPACE GRADE)
//  STM32F407VGT6 + FreeRTOS
//  • IWDG Watchdog (4 s) — "istənilən sensor canlıdırsa bəslə"
//  • Açıq flight state-machine (FS_STANDBY → FS_LANDED)
//  • Sensor sağlamlıq monitorinqi (I2C backoff + canlılıq zamanı)
//  • Quaternion əsaslı tilt (gimbal-lock yox)
//  • PİRO Qoruyucu (1.5s brownout qoruyucu taymeri əlavə olundu!)
//  • Transonic Baro Şok filtrasiyası və Təhlükəsiz Tilt Açılışı
//  • CRC16 qorunan binary telemetriya (Serial3) + debug CSV (Serial1)
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <STM32FreeRTOS.h>
#include <IWatchdog.h>

// ----------------------------- PIN TƏYİNATLARI -----------------------------
#define BUZZER_PIN   PE9
#define PIN_APOGEE   PD12
#define PIN_MAIN     PD13

// ----------------------------- FİZİKİ SABİTLƏR -----------------------------
#define GRAVITY_MS2   9.80665f
#define RECIPROCAL_G  (1.0f / GRAVITY_MS2)
#define RAD2DEG       57.295779513f

// ----------------------------- ZAMANLAMA (ms) -----------------------------
#define TASK_PERIOD_MS        20
#define TX_PERIOD_MS          66    // ~15 Hz
#define AHT_PERIOD_DIV        5     // AHT20 hər 5-ci dövrdə (~100 ms)
#define WATCHDOG_TIMEOUT_US   4000000UL  // 4 s
#define SENSOR_ALIVE_WINDOW_MS 500

// ----------------------------- UÇUŞ EŞİKLƏRİ ------------------------------
#define LAUNCH_ARM_ALT_M   5.0f     
#define APOGEE_DROP_M      3.0f
#define APOGEE_TILT_DEG    60.0f
#define MAIN_DEPLOY_ALT_M  500.0f
#define LAND_ALT_M         5.0f
#define LAND_VSPEED_MS     0.5f
#define LAND_SETTLE_COUNT  100    // 100 × 20 ms = 2.0 s

// ----------------------------- I2C ÜNVANLARI ------------------------------
#define BME280_ADDR 0x76
#define AHT20_ADDR  0x38
#define BNO055_ADDR 0x28

// ----------------------------- TELEMETRİYA --------------------------------
#define RF_HDR0   0xAA
#define RF_HDR1   0x55
#define RF_TELEM  0x01

// ============================================================================
//  TİPLƏR
// ============================================================================

enum FlightState : uint8_t {
  FS_STANDBY  = 0,
  FS_LAUNCHED = 1,
  FS_APOGEE   = 2,
  FS_MAIN     = 3,
  FS_LANDED   = 4
};

struct SensorData {
  float qw=1, qx=0, qy=0, qz=0;
  float accel_x=0, accel_y=0, accel_z=0;
  float angle_x=0, angle_y=0, angle_z=0;
  float tilt_deg = 0;
  float bme_temp=0, bme_hum=0, bme_pres=0;
  float aht_temp=0, aht_hum=0;
  float altitude=0;
  float vertical_speed=0;
  float total_g=1.0f;
  float gps_lat=0, gps_lon=0, gps_alt=0;
  bool  gps_fix=false;
  bool  bno_ok=false, bme_ok=false, aht_ok=false;
  uint8_t bno_calib=0;               
  FlightState state = FS_STANDBY;
  float max_altitude=0;
};

struct Kalman1D {
  float Q, R, x, P; bool ready=false;
  void init(float q, float r) { Q=q; R=r; P=1.0f; }
  float update(float z) {
    if (!ready) { x=z; ready=true; return x; }
    P += Q; const float K = P/(P+R);
    x += K*(z-x); P = (1.0f-K)*P; return x;
  }
};

struct QuatSlerp {
  float w=1, x=0, y=0, z=0, alpha; bool ready=false;
  void init(float a) { alpha=a; }
  void update(float nw, float nx, float ny, float nz) {
    if (!ready) { w=nw; x=nx; y=ny; z=nz; ready=true; return; }
    float dot = w*nw + x*nx + y*ny + z*nz;
    if (dot < 0.0f) { nw=-nw; nx=-nx; ny=-ny; nz=-nz; dot=-dot; }
    float t0, t1;
    if (dot < 0.9999f) {
      const float th = acosf(dot), s = sinf(th);
      t0 = sinf((1.0f-alpha)*th)/s; t1 = sinf(alpha*th)/s;
    } else { t0 = 1.0f-alpha; t1 = alpha; }
    w = t0*w + t1*nw; x = t0*x + t1*nx; y = t0*y + t1*ny; z = t0*z + t1*nz;
    const float n = sqrtf(w*w + x*x + y*y + z*z);
    if (n > 0.0f) { const float inv = 1.0f/n; w*=inv; x*=inv; y*=inv; z*=inv; }
  }
};

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// ============================================================================
//  GLOBAL OBYEKTLƏR
// ============================================================================

SensorData shared;
SemaphoreHandle_t i2c_mutex = NULL, data_mutex = NULL;
Kalman1D kf_bme_temp, kf_bme_hum, kf_bme_pres, kf_aht_temp, kf_aht_hum, kf_altitude, kf_vspeed;
QuatSlerp qslerp;

volatile uint32_t last_sensor_activity = 0;

struct I2CHealth { uint8_t fails=0; uint32_t next_try=0; };
static I2CHealth health_bno, health_bme, health_aht;

static inline bool i2c_should_try(const I2CHealth &h, uint32_t now) {
  return h.fails < 3 || (int32_t)(now - h.next_try) >= 0;
}
static inline void i2c_mark_result(I2CHealth &h, bool ok, uint32_t now) {
  if (ok) { h.fails = 0; return; }
  if (h.fails < 255) h.fails++;
  if (h.fails >= 3) h.next_try = now + 1000;
}

static void wd_delay(uint32_t ms) {
  uint32_t start = millis();
  while ((uint32_t)(millis() - start) < ms) { IWatchdog.reload(); delay(10); }
}

static float quat_tilt_deg(float qw, float qx, float qy, float qz) {
  (void)qw; (void)qz;
  float c = 1.0f - 2.0f*(qx*qx + qy*qy);
  if (c > 1.0f) c = 1.0f; else if (c < -1.0f) c = -1.0f;
  return acosf(c) * RAD2DEG;
}

// ============================================================================
//  I2C AŞAĞI SƏVİYYƏ
// ============================================================================

static bool i2c_write(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(data);
  return Wire.endTransmission() == 0;
}
static bool i2c_read_buf(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ============================================================================
//  BME280 (Təzyiq / Temperatur / Rütubət)
// ============================================================================

struct BmeCal {
  uint16_t T1; int16_t T2, T3; uint16_t P1;
  int16_t P2,P3,P4,P5,P6,P7,P8,P9; uint8_t H1,H3; int16_t H2,H4,H5; int8_t H6;
};
static BmeCal bme_cal;

static void bme280_init() {
  uint8_t buf[26];
  if (!i2c_read_buf(BME280_ADDR, 0x88, buf, 26)) return;
  bme_cal.T1=buf[1]<<8|buf[0]; bme_cal.T2=buf[3]<<8|buf[2]; bme_cal.T3=buf[5]<<8|buf[4];
  bme_cal.P1=buf[7]<<8|buf[6]; bme_cal.P2=buf[9]<<8|buf[8]; bme_cal.P3=buf[11]<<8|buf[10];
  bme_cal.P4=buf[13]<<8|buf[12]; bme_cal.P5=buf[15]<<8|buf[14]; bme_cal.P6=buf[17]<<8|buf[16];
  bme_cal.P7=buf[19]<<8|buf[18]; bme_cal.P8=buf[21]<<8|buf[20]; bme_cal.P9=buf[23]<<8|buf[22];
  bme_cal.H1=buf[25];
  uint8_t hb[7]; i2c_read_buf(BME280_ADDR, 0xE1, hb, 7);
  bme_cal.H2=hb[1]<<8|hb[0]; bme_cal.H3=hb[2];
  bme_cal.H4=(int16_t)(hb[3]<<4)|(hb[4]&0x0F); bme_cal.H5=(int16_t)(hb[5]<<4)|(hb[4]>>4); bme_cal.H6=(int8_t)hb[6];
  
  i2c_write(BME280_ADDR, 0xF2, 0x01);
  i2c_write(BME280_ADDR, 0xF4, 0x27);
  i2c_write(BME280_ADDR, 0xF5, 0x00);
}

static bool bme280_read_all(float &temp, float &pres, float &hum) {
  uint8_t b[8];
  if (!i2c_read_buf(BME280_ADDR, 0xF7, b, 8)) return false;
  int32_t raw_p = (b[0]<<12)|(b[1]<<4)|(b[2]>>4);
  int32_t raw_t = (b[3]<<12)|(b[4]<<4)|(b[5]>>4);
  int32_t raw_h = (b[6]<<8)|b[7];
  int32_t v1 = ((((raw_t>>3)-((int32_t)bme_cal.T1<<1)))*bme_cal.T2)>>11;
  int32_t v2 = (((((raw_t>>4)-(int32_t)bme_cal.T1)*((raw_t>>4)-(int32_t)bme_cal.T1))>>12)*bme_cal.T3)>>14;
  int32_t t_fine = v1 + v2;
  temp = (t_fine*5 + 128) * 0.0000390625f;       
  int64_t p1 = (int64_t)t_fine - 128000;
  int64_t p2 = p1*p1*bme_cal.P6; p2 += ((p1*bme_cal.P5)<<17); p2 += ((int64_t)bme_cal.P4<<35);
  p1 = ((p1*p1*bme_cal.P3)>>8) + ((p1*bme_cal.P2)<<12);
  p1 = ((((int64_t)1<<47) + p1) * bme_cal.P1) >> 33;
  if (p1 == 0) { pres = 0; }
  else {
    int64_t p = ((int64_t)(1048576 - raw_p)<<31) - p2; p = (p*3125)/p1;
    p1 = ((int64_t)bme_cal.P9 * (p>>13) * (p>>13)) >> 25;
    p2 = ((int64_t)bme_cal.P8 * p) >> 19;
    pres = ((p + p1 + p2) >> 8) * 0.01f;          
  }
  int32_t h1 = t_fine - 76800;
  h1 = (((raw_h<<14) - ((int32_t)bme_cal.H4<<20) - ((int32_t)bme_cal.H5*h1)) + 16384) >> 15;
  h1 = h1 * (((((((h1*bme_cal.H6)>>10) * (((h1*bme_cal.H3)>>11) + 32768))>>10) + 2097152) * bme_cal.H2 + 8192) >> 14);
  h1 = h1 - (((((h1>>15)*(h1>>15))>>7)*bme_cal.H1)>>4);
  if (h1 < 0) h1 = 0; else if (h1 > 419430400) h1 = 419430400;
  hum = (h1>>12) * 0.0009765625f;                 
  return true;
}

// ============================================================================
//  AHT20 (Temperatur / Rütubət)
// ============================================================================

static void aht20_trigger() {
  Wire.beginTransmission(AHT20_ADDR); Wire.write(0xAC); Wire.write(0x33); Wire.write(0x00);
  Wire.endTransmission();
}
static bool aht20_read(float &temp, float &hum) {
  uint8_t buf[6];
  if (Wire.requestFrom((uint8_t)AHT20_ADDR, (uint8_t)6) != 6) return false;
  for (uint8_t i=0;i<6;i++) buf[i]=Wire.read();
  uint32_t hr = ((uint32_t)buf[1]<<12)|((uint32_t)buf[2]<<4)|(buf[3]>>4);
  uint32_t tr = ((uint32_t)(buf[3]&0x0F)<<16)|((uint32_t)buf[4]<<8)|buf[5];
  hum  = hr * 0.00009536743f;
  temp = tr * 0.00019073486f - 50.0f;
  return true;
}

// ============================================================================
//  BNO055 (9-DOF IMU — NDOF)
// ============================================================================

static void bno055_init() {
  i2c_write(BNO055_ADDR, 0x3D, 0x00); delay(25);   
  i2c_write(BNO055_ADDR, 0x3E, 0x00);              
  i2c_write(BNO055_ADDR, 0x3B, 0x00);              
  delay(10);
  i2c_write(BNO055_ADDR, 0x3D, 0x0C); delay(20);   
}
static bool bno055_read_quat(float &qw, float &qx, float &qy, float &qz) {
  uint8_t buf[8]; if (!i2c_read_buf(BNO055_ADDR, 0x20, buf, 8)) return false;
  qw=(int16_t)(buf[1]<<8|buf[0]) * 0.00006103515625f;
  qx=(int16_t)(buf[3]<<8|buf[2]) * 0.00006103515625f;
  qy=(int16_t)(buf[5]<<8|buf[4]) * 0.00006103515625f;
  qz=(int16_t)(buf[7]<<8|buf[6]) * 0.00006103515625f;
  return true;
}
static bool bno055_read_accel(float &ax, float &ay, float &az) {
  uint8_t buf[6]; if (!i2c_read_buf(BNO055_ADDR, 0x08, buf, 6)) return false;
  ax=(int16_t)(buf[1]<<8|buf[0]) * 0.01f;
  ay=(int16_t)(buf[3]<<8|buf[2]) * 0.01f;
  az=(int16_t)(buf[5]<<8|buf[4]) * 0.01f;
  return true;
}
static bool bno055_read_euler(float &ex, float &ey, float &ez) {
  uint8_t buf[6]; if (!i2c_read_buf(BNO055_ADDR, 0x1A, buf, 6)) return false;
  ex=(int16_t)(buf[1]<<8|buf[0]) * 0.0625f;
  ey=(int16_t)(buf[3]<<8|buf[2]) * 0.0625f;
  ez=(int16_t)(buf[5]<<8|buf[4]) * 0.0625f;
  return true;
}
static uint8_t bno055_read_calib() {
  uint8_t buf[1];
  if (!i2c_read_buf(BNO055_ADDR, 0x35, buf, 1)) return 0;
  return (buf[0] >> 6) & 0x03;   
}

// ============================================================================
//  BAROMETRİK HÜNDÜRLÜK
// ============================================================================

static float p_ref = 1013.25f;  

static float calc_altitude(float pres_hPa) {
  return 44330.0f * (1.0f - powf(pres_hPa / p_ref, 0.190284f));
}

static void init_altitude_ref() {
  float sum = 0; int good = 0; const int N = 30;
  for (int i = 0; i < N; i++) {
    float t, p, h;
    if (bme280_read_all(t, p, h)) { sum += p; good++; }
    wd_delay(50);
  }
  if (good > 0) p_ref = sum / good;
}

// ============================================================================
//  GPS (NMEA GGA)
// ============================================================================

static bool verify_nmea_checksum(const char *nmea) {
  if (nmea[0] != '$') return false;
  uint8_t sum = 0; int i = 1;
  while (nmea[i] != '*' && nmea[i] != '\0' && i < 90) sum ^= (uint8_t)nmea[i++];
  if (nmea[i] == '*') { long expected = strtol(&nmea[i+1], NULL, 16); return (sum == expected); }
  return false;
}

static void parse_gps_line(char *line) {
  if (!verify_nmea_checksum(line)) return;
  if (strncmp(line, "$GNGGA", 6) != 0 && strncmp(line, "$GPGGA", 6) != 0) return;

  int field_idx = 0; char *p = line; char *token;
  float raw_lat = 0, raw_lon = 0, gps_alt = 0;
  char lat_dir = 'N', lon_dir = 'E'; int fix_quality = 0;

  while ((token = strsep(&p, ",")) != NULL) {
    if (field_idx == 6) fix_quality = atoi(token);
    else if (field_idx == 2) raw_lat = atof(token);
    else if (field_idx == 3) lat_dir = token[0];
    else if (field_idx == 4) raw_lon = atof(token);
    else if (field_idx == 5) lon_dir = token[0];
    else if (field_idx == 9) gps_alt = atof(token);
    field_idx++;
  }

  if (fix_quality > 0) {
    int deg_lat = (int)(raw_lat * 0.01f); float min_lat = raw_lat - deg_lat*100;
    float lat = deg_lat + min_lat * 0.016666667f; if (lat_dir == 'S') lat = -lat;
    int deg_lon = (int)(raw_lon * 0.01f); float min_lon = raw_lon - deg_lon*100;
    float lon = deg_lon + min_lon * 0.016666667f; if (lon_dir == 'W') lon = -lon;
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    shared.gps_lat = lat; shared.gps_lon = lon; shared.gps_alt = gps_alt; shared.gps_fix = true;
    xSemaphoreGive(data_mutex);
  } else {
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    shared.gps_fix = false;
    xSemaphoreGive(data_mutex);
  }
}

static void play_buzzer_tone(int freq, int dur_ms) { tone(BUZZER_PIN, freq, dur_ms); }

// ============================================================================
//  TELEMETRİYA 
// ============================================================================

#pragma pack(push, 1)
struct TelemetryFrame {
  uint8_t  hdr0, hdr1, type, len;
  uint32_t ms;
  int16_t  qw, qx, qy, qz;            
  int16_t  accel_x, accel_y, accel_z; 
  int16_t  angle_x, angle_y, angle_z; 
  int16_t  bme_temp, bme_pres, bme_hum; 
  int32_t  altitude;                  
  int16_t  aht_temp, aht_hum;         
  int32_t  gps_lat, gps_lon;          
  int16_t  gps_alt;                   
  int16_t  total_g;                   
  int16_t  vertical_speed;            
  uint8_t  state;
  uint8_t  flags;                     
  uint8_t  bno_calib;                 
  uint16_t crc16;
};
#pragma pack(pop)

static void tx_send_frame(const SensorData &s, uint8_t type) {
  TelemetryFrame f;
  memset(&f, 0, sizeof(f));
  f.hdr0 = RF_HDR0; f.hdr1 = RF_HDR1; f.type = type;
  const uint16_t payload_len = sizeof(TelemetryFrame) - 6; 
  f.len = (uint8_t)payload_len;
  f.ms = millis();
  f.qw = (int16_t)(s.qw*32767); f.qx = (int16_t)(s.qx*32767);
  f.qy = (int16_t)(s.qy*32767); f.qz = (int16_t)(s.qz*32767);
  f.accel_x = (int16_t)(s.accel_x*100); f.accel_y = (int16_t)(s.accel_y*100); f.accel_z = (int16_t)(s.accel_z*100);
  f.angle_x = (int16_t)(s.angle_x*10); f.angle_y = (int16_t)(s.angle_y*10); f.angle_z = (int16_t)(s.angle_z*10);
  f.bme_temp = (int16_t)(s.bme_temp*10); f.bme_pres = (int16_t)(s.bme_pres*10); f.bme_hum = (int16_t)(s.bme_hum*10);
  f.altitude = (int32_t)(s.altitude*100);
  f.aht_temp = (int16_t)(s.aht_temp*10); f.aht_hum = (int16_t)(s.aht_hum*10);
  f.gps_lat = (int32_t)(s.gps_lat*1e7f); f.gps_lon = (int32_t)(s.gps_lon*1e7f);
  f.gps_alt = (int16_t)(s.gps_alt);
  f.total_g = (int16_t)(s.total_g*100);
  f.vertical_speed = (int16_t)(s.vertical_speed*100);
  f.state = (uint8_t)s.state;
  f.flags = (s.gps_fix?1:0) | (s.bno_ok?2:0) | (s.bme_ok?4:0) | (s.aht_ok?8:0);
  f.bno_calib = s.bno_calib;
  f.crc16 = crc16_ccitt(((const uint8_t*)&f) + 4, payload_len);
  Serial3.write((const uint8_t*)&f, sizeof(f));
}

// ============================================================================
//  TASKLAR
// ============================================================================

void env_task(void*) {
  TickType_t t = xTaskGetTickCount();
  uint8_t aht_counter = 0;

  if (i2c_should_try(health_aht, millis())) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY); aht20_trigger(); xSemaphoreGive(i2c_mutex);
  }

  for (;;) {
    uint32_t now = millis();
    bool bme_ok = false, aht_ok = false;
    float bt=0, bp=0, bh=0, at=0, ah=0;

    if (i2c_should_try(health_bme, now)) {
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      bme_ok = bme280_read_all(bt, bp, bh);
      xSemaphoreGive(i2c_mutex);
      i2c_mark_result(health_bme, bme_ok, now);
    }

    aht_counter++;
    if (aht_counter >= AHT_PERIOD_DIV) {
      aht_counter = 0;
      if (i2c_should_try(health_aht, now)) {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        aht_ok = aht20_read(at, ah);
        aht20_trigger();
        xSemaphoreGive(i2c_mutex);
        i2c_mark_result(health_aht, aht_ok, now);
      }
    }

    if (bme_ok || aht_ok) last_sensor_activity = millis();

    xSemaphoreTake(data_mutex, portMAX_DELAY);
    if (bme_ok) {
      shared.bme_temp = kf_bme_temp.update(bt);
      shared.bme_hum  = kf_bme_hum.update(bh);
      shared.bme_pres = kf_bme_pres.update(bp);
      shared.altitude = kf_altitude.update(calc_altitude(shared.bme_pres));
    }
    if (aht_ok) { shared.aht_temp = kf_aht_temp.update(at); shared.aht_hum = kf_aht_hum.update(ah); }
    shared.bme_ok = bme_ok; shared.aht_ok = aht_ok;
    xSemaphoreGive(data_mutex);

    vTaskDelayUntil(&t, pdMS_TO_TICKS(TASK_PERIOD_MS));
  }
}

void imu_task(void*) {
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    uint32_t now = millis();
    bool ok = false;

    if (i2c_should_try(health_bno, now)) {
      float qw,qx,qy,qz, ax,ay,az, ex,ey,ez;
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      bool okQ = bno055_read_quat(qw,qx,qy,qz);
      bool okA = bno055_read_accel(ax,ay,az);
      bool okE = bno055_read_euler(ex,ey,ez);
      uint8_t calib = bno055_read_calib();
      xSemaphoreGive(i2c_mutex);

      ok = okQ && okA && okE;
      i2c_mark_result(health_bno, ok, now);

      if (ok) {
        last_sensor_activity = millis();
        qslerp.update(qw,qx,qy,qz);
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        shared.qw=qslerp.w; shared.qx=qslerp.x; shared.qy=qslerp.y; shared.qz=qslerp.z;
        shared.accel_x=ax; shared.accel_y=ay; shared.accel_z=az;
        shared.angle_x=ex; shared.angle_y=ey; shared.angle_z=ez;
        shared.tilt_deg = quat_tilt_deg(qslerp.w, qslerp.x, qslerp.y, qslerp.z);
        shared.bno_calib = calib;
        xSemaphoreGive(data_mutex);
      }
    }

    xSemaphoreTake(data_mutex, portMAX_DELAY);
    shared.bno_ok = ok;
    xSemaphoreGive(data_mutex);

    vTaskDelayUntil(&t, pdMS_TO_TICKS(TASK_PERIOD_MS));
  }
}

void gps_task(void*) {
  static char gps_buf[100]; static int buf_idx = 0;
  for (;;) {
    while (Serial2.available()) {
      char c = Serial2.read();
      if (c == '$') buf_idx = 0;
      if (c == '\n' || c == '\r') {
        if (buf_idx > 10) { gps_buf[buf_idx] = '\0'; parse_gps_line(gps_buf); }
        buf_idx = 0;
      } else if (buf_idx < 99) {
        gps_buf[buf_idx++] = c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ════════════════════════════════════════════════════════════
//  UÇUŞ DİNAMİKASI (KOSMİK SƏNAYE STANDARTI)
// ════════════════════════════════════════════════════════════
void flight_control_task(void*) {
  TickType_t t = xTaskGetTickCount();
  float prev_alt = 0;
  uint32_t prev_ms = millis();
  uint32_t landing_counter = 0;
  
  uint32_t apogee_detect_counter = 0; 
  
  uint32_t apogee_fire_time = 0;
  uint32_t main_fire_time = 0;
  bool piro_ap_active = false;
  bool piro_mn_active = false;

  for (;;) {
    uint32_t now = millis();
    float dt = (now - prev_ms) * 0.001f;
    prev_ms = now;
    if (dt < 0.005f) dt = 0.005f; else if (dt > 0.05f) dt = 0.05f;

    xSemaphoreTake(data_mutex, portMAX_DELAY);
    float cur_alt = shared.altitude;
    float tilt = shared.tilt_deg;
    FlightState st = shared.state;
    float max_alt = shared.max_altitude;
    xSemaphoreGive(data_mutex);

    float raw_vs = (cur_alt - prev_alt) / dt;
    prev_alt = cur_alt;
    float vs = kf_vspeed.update(raw_vs);

    // MÜHƏNDİSLİK DÜZƏLİŞİ 1: Max Hündürlük YALNIZ şaquli sürət müsbətdirsə (yuxarı uçarkən) yenilənir
    // Transonic şok dalğası nəticəsində təzyiqin yalançı düşməsindən qaynaqlanan "fantom zirvənin" qarşısını alır
    if (cur_alt > max_alt && vs > 0.0f) {
        max_alt = cur_alt;
    }

    xSemaphoreTake(data_mutex, portMAX_DELAY);
    float ax = shared.accel_x, ay = shared.accel_y, az = shared.accel_z;
    xSemaphoreGive(data_mutex);
    float g_force = sqrtf(ax*ax + ay*ay + az*az) * RECIPROCAL_G;

    FlightState new_state = st;
    
    // Qalxış Təsdiqi 
    if (st == FS_STANDBY && (g_force > 2.5f || (vs > 10.0f && cur_alt > LAUNCH_ARM_ALT_M))) {
        new_state = FS_LAUNCHED;
    }
    
    else if (st == FS_LAUNCHED) {
      bool falling = (cur_alt < (max_alt - APOGEE_DROP_M)) && (vs < -1.0f);
      
      // MÜHƏNDİSLİK DÜZƏLİŞİ 2: Tilt ehtiyatına sürət itirilməsi (vs < 5.0f) şərti əlavə olundu
      // Weathercocking (küləkdən əyilmə) zamanı motor hələ itələyirsə və ya raket hələ sürətlə qalxırsa paraşüt açılmayacaq
      bool tilted = (tilt > APOGEE_TILT_DEG) && (g_force < 1.5f) && (vs < 5.0f);
      
      if (falling || tilted) {
          apogee_detect_counter++;
          if (apogee_detect_counter > 3) new_state = FS_APOGEE; 
      } else { apogee_detect_counter = 0; }
    }
    
    else if (st == FS_APOGEE && cur_alt <= MAIN_DEPLOY_ALT_M) new_state = FS_MAIN;
    
    else if (st == FS_MAIN) {
      if (cur_alt < LAND_ALT_M && fabsf(vs) < LAND_VSPEED_MS) {
        landing_counter++;
        if (landing_counter >= LAND_SETTLE_COUNT) new_state = FS_LANDED;
      } else landing_counter = 0;
    }

    if (new_state != st) {
      if (new_state == FS_APOGEE) { 
          digitalWrite(PIN_APOGEE, HIGH); 
          piro_ap_active = true;
          apogee_fire_time = now; 
          play_buzzer_tone(1500, 500); 
      }
      else if (new_state == FS_MAIN) { 
          digitalWrite(PIN_MAIN, HIGH);   
          piro_mn_active = true;
          main_fire_time = now;
          play_buzzer_tone(2000, 800); 
      }
      else if (new_state == FS_LANDED) { play_buzzer_tone(3000, 2000); }
    }

    xSemaphoreTake(data_mutex, portMAX_DELAY);
    shared.state = new_state;
    shared.max_altitude = max_alt;
    shared.vertical_speed = vs;
    shared.total_g = g_force;
    xSemaphoreGive(data_mutex);

    // PİRO KANAL SÖNDÜRÜCÜ (BATARYA QORUYUCUSU 1.5 SANİYƏ)
    if (piro_ap_active && (now - apogee_fire_time > 1500)) {
        digitalWrite(PIN_APOGEE, LOW);
        piro_ap_active = false;
    }
    if (piro_mn_active && (now - main_fire_time > 1500)) {
        digitalWrite(PIN_MAIN, LOW);
        piro_mn_active = false;
    }

    if ((int32_t)(millis() - last_sensor_activity) < SENSOR_ALIVE_WINDOW_MS) {
      IWatchdog.reload();
    }

    vTaskDelayUntil(&t, pdMS_TO_TICKS(TASK_PERIOD_MS));
  }
}

void tx_task(void*) {
  TickType_t t = xTaskGetTickCount();
  Serial1.println("ms,state,alt_m,vs_ms,total_g,temp_C,pres_hPa,tilt_deg,lat,lon,fix,bno,bme,aht,calib");
  for (;;) {
    SensorData snap;
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    memcpy(&snap, &shared, sizeof(SensorData));
    xSemaphoreGive(data_mutex);

    tx_send_frame(snap, RF_TELEM);

    Serial1.print(millis()); Serial1.print(',');
    Serial1.print((int)snap.state); Serial1.print(',');
    Serial1.print(snap.altitude, 1); Serial1.print(',');
    Serial1.print(snap.vertical_speed, 1); Serial1.print(',');
    Serial1.print(snap.total_g, 2); Serial1.print(',');
    Serial1.print(snap.bme_temp, 1); Serial1.print(',');
    Serial1.print(snap.bme_pres, 1); Serial1.print(',');
    Serial1.print(snap.tilt_deg, 1); Serial1.print(',');
    Serial1.print(snap.gps_lat, 6); Serial1.print(',');
    Serial1.print(snap.gps_lon, 6); Serial1.print(',');
    Serial1.print(snap.gps_fix ? 1 : 0); Serial1.print(',');
    Serial1.print(snap.bno_ok); Serial1.print(',');
    Serial1.print(snap.bme_ok); Serial1.print(',');
    Serial1.print(snap.aht_ok); Serial1.print(',');
    Serial1.println(snap.bno_calib);

    vTaskDelayUntil(&t, pdMS_TO_TICKS(TX_PERIOD_MS));
  }
}

// ============================================================================
//  SETUP & LOOP
// ============================================================================

void setup() {
  IWatchdog.begin(WATCHDOG_TIMEOUT_US);
  IWatchdog.reload();

  Serial1.begin(115200);
  Serial2.begin(9600);
  Serial3.begin(115200);
  Serial6.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIN_APOGEE, OUTPUT); digitalWrite(PIN_APOGEE, LOW);
  pinMode(PIN_MAIN, OUTPUT);   digitalWrite(PIN_MAIN, LOW);

  wd_delay(1500);

  Wire.begin(); Wire.setClock(400000);

  bme280_init();
  wd_delay(10);
  bno055_init();
  init_altitude_ref();

  kf_bme_temp.init(0.005f, 0.25f); kf_bme_hum.init(0.05f, 9.0f); kf_bme_pres.init(0.05f, 1.0f);
  kf_altitude.init(0.1f, 4.0f); kf_aht_temp.init(0.005f, 0.09f); kf_aht_hum.init(0.05f, 4.0f);
  qslerp.init(0.15f); kf_vspeed.init(0.1f, 2.0f);

  i2c_mutex = xSemaphoreCreateMutex();
  data_mutex = xSemaphoreCreateMutex();

  xTaskCreate(tx_task,             "TX",      512, NULL, 4, NULL);
  xTaskCreate(flight_control_task, "FLIGHT",  512, NULL, 4, NULL);
  xTaskCreate(imu_task,            "IMU",     512, NULL, 3, NULL);
  xTaskCreate(gps_task,            "GPS",     384, NULL, 2, NULL);
  xTaskCreate(env_task,            "ENV",     512, NULL, 1, NULL);

  vTaskStartScheduler();
}

void loop() {}