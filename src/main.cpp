#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <STM32FreeRTOS.h>
#include <string.h>

// ════════════════════════════════════════════════════════════
//  UART Port & Pin Bağlantıları
// ════════════════════════════════════════════════════════════
HardwareSerial Serial1(PA10, PA9);   // Debug / PC
HardwareSerial Serial2(PA3, PA2);    // GPS
HardwareSerial Serial3(PB11, PB10);  // RFD900x (Telemetriya)
HardwareSerial Serial6(PC7, PC6);    // RS232 (SUT/SIT Test Cihazı)

#define BUZZER_PIN PE9

// --- Alovlandırma / Paraşüt Pinləri ---
#define PIN_APOGEE PD12   
#define PIN_MAIN   PD13   

// ════════════════════════════════════════════════════════════
//  Sensor Data Strukturu
// ════════════════════════════════════════════════════════════
struct SensorData {
  float qw=1, qx=0, qy=0, qz=0;
  float bme_temp=0, bme_hum=0, bme_pres=0;
  float aht_temp=0, aht_hum=0;
  float altitude=0;
  float gps_lat=0.0f, gps_lon=0.0f, gps_alt=0.0f;
  bool  gps_fix=false;
  float accel_x=0, accel_y=0, accel_z=0;
  float angle_x=0, angle_y=0, angle_z=0;
  
  // Təhlükəsizlik: Statuslar Mutex ilə qorunur
  bool apogee_fired = false;
  bool main_fired = false;
};

// ════════════════════════════════════════════════════════════
//  Filter & Slerp (FPU Optimizasiyalı)
// ════════════════════════════════════════════════════════════
struct Kalman1D {
  float Q, R, x, P; bool ready = false;
  void init(float q, float r) { Q=q; R=r; P=1.0f; }
  float update(float z) {
    if (!ready) { x = z; ready = true; return x; }
    P += Q; const float K = P / (P + R);
    x += K * (z - x); P = (1.0f - K) * P; return x;
  }
};

struct QuatSlerp {
  float w=1, x=0, y=0, z=0, alpha; bool ready = false;
  void init(float a) { alpha = a; }
  void update(float nw, float nx, float ny, float nz) {
    if (!ready) { w=nw; x=nx; y=ny; z=nz; ready=true; return; }
    float dot = w*nw + x*nx + y*ny + z*nz;
    if (dot < 0.0f) { nw=-nw; nx=-nx; ny=-ny; nz=-nz; dot=-dot; }
    float t0, t1;
    if (dot < 0.9999f) {
      const float theta = acosf(dot), s = sinf(theta);
      t0 = sinf((1.0f - alpha) * theta) / s;
      t1 = sinf(alpha * theta) / s;
    } else { t0 = 1.0f - alpha; t1 = alpha; }
    w = t0*w + t1*nw; x = t0*x + t1*nx; y = t0*y + t1*ny; z = t0*z + t1*nz;
    
    // FPU donanım kökaltı və sürətli vurma
    const float n = sqrtf(w*w + x*x + y*y + z*z); 
    if (n > 0.0f) { 
      const float inv_n = 1.0f / n; 
      w *= inv_n; x *= inv_n; y *= inv_n; z *= inv_n; 
    }
  }
};

SensorData shared;
SemaphoreHandle_t i2c_mutex, data_mutex;

Kalman1D kf_bme_temp, kf_bme_hum, kf_bme_pres, kf_aht_temp, kf_aht_hum, kf_altitude;
QuatSlerp qslerp;
volatile float max_flight_alt = 0.0f;

// ════════════════════════════════════════════════════════════
//  I2C Sensorlar - BURST READ VƏ OPTİMİZASİYA
// ════════════════════════════════════════════════════════════
bool i2c_write(uint8_t addr, uint8_t reg, uint8_t data) { Wire.beginTransmission(addr); Wire.write(reg); Wire.write(data); return Wire.endTransmission() == 0; }
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

#define BME280_ADDR 0x76
struct { uint16_t T1; int16_t T2, T3; uint16_t P1; int16_t P2,P3,P4,P5,P6,P7,P8,P9; uint8_t H1,H3; int16_t H2,H4,H5; int8_t H6; } bme_cal;

void bme280_init() { 
  uint8_t buf[26]; i2c_read_buf(BME280_ADDR, 0x88, buf, 26);
  bme_cal.T1=buf[1]<<8|buf[0]; bme_cal.T2=buf[3]<<8|buf[2]; bme_cal.T3=buf[5]<<8|buf[4]; bme_cal.P1=buf[7]<<8|buf[6]; bme_cal.P2=buf[9]<<8|buf[8]; bme_cal.P3=buf[11]<<8|buf[10]; bme_cal.P4=buf[13]<<8|buf[12]; bme_cal.P5=buf[15]<<8|buf[14]; bme_cal.P6=buf[17]<<8|buf[16]; bme_cal.P7=buf[19]<<8|buf[18]; bme_cal.P8=buf[21]<<8|buf[20]; bme_cal.P9=buf[23]<<8|buf[22]; bme_cal.H1=buf[25]; uint8_t hb[7]; i2c_read_buf(BME280_ADDR,0xE1,hb,7);
  bme_cal.H2=hb[1]<<8|hb[0]; bme_cal.H3=hb[2]; bme_cal.H4=(int16_t)(hb[3]<<4)|(hb[4]&0x0F); bme_cal.H5=(int16_t)(hb[5]<<4)|(hb[4]>>4); bme_cal.H6=(int8_t)hb[6];
  i2c_write(BME280_ADDR,0xF2,0x01); i2c_write(BME280_ADDR,0xF4,0x27); i2c_write(BME280_ADDR,0xF5,0xA0);
}

// Burst Read: Təzyiq, temperatur və rütubət eyni anda 1 paketdə oxunur
bool bme280_read_all(float &temp, float &pres, float &hum) { 
  uint8_t b[8]; 
  if (!i2c_read_buf(BME280_ADDR, 0xF7, b, 8)) return false; 
  
  int32_t raw_p = (b[0]<<12)|(b[1]<<4)|(b[2]>>4); 
  int32_t raw_t = (b[3]<<12)|(b[4]<<4)|(b[5]>>4); 
  int32_t raw_h = (b[6]<<8)|b[7]; 
  
  // Temperatur
  int32_t v1 = ((((raw_t>>3)-((int32_t)bme_cal.T1<<1)))*bme_cal.T2)>>11; 
  int32_t v2 = (((((raw_t>>4)-(int32_t)bme_cal.T1)*((raw_t>>4)-(int32_t)bme_cal.T1))>>12)*bme_cal.T3)>>14; 
  int32_t t_fine = v1 + v2; 
  temp = (t_fine * 5 + 128) * 0.0000390625f;  
  
  // Təzyiq
  int64_t p1 = (int64_t)t_fine - 128000; 
  int64_t p2 = p1 * p1 * bme_cal.P6; p2 += ((p1 * bme_cal.P5) << 17); p2 += ((int64_t)bme_cal.P4 << 35); 
  p1 = ((p1 * p1 * bme_cal.P3) >> 8) + ((p1 * bme_cal.P2) << 12); p1 = ((((int64_t)1 << 47) + p1) * bme_cal.P1) >> 33; 
  if(p1 == 0) { pres = 0; } else {
    int64_t p = ((int64_t)(1048576 - raw_p) << 31) - p2; p = (p * 3125) / p1; 
    p1 = ((int64_t)bme_cal.P9 * (p >> 13) * (p >> 13)) >> 25; p2 = ((int64_t)bme_cal.P8 * p) >> 19; 
    pres = ((p + p1 + p2) >> 8) * 0.0000390625f; 
  }

  // Rütubət
  int32_t h1 = t_fine - 76800; 
  h1 = (((raw_h << 14) - ((int32_t)bme_cal.H4 << 20) - ((int32_t)bme_cal.H5 * h1)) + 16384) >> 15; 
  h1 = h1 * (((((((h1 * bme_cal.H6) >> 10) * (((h1 * bme_cal.H3) >> 11) + 32768)) >> 10) + 2097152) * bme_cal.H2 + 8192) >> 14); 
  h1 = h1 - (((((h1 >> 15) * (h1 >> 15)) >> 7) * bme_cal.H1) >> 4); h1 = constrain(h1, 0, 419430400); 
  hum = (h1 >> 12) * 0.0009765625f; 
  
  return true; 
}

#define AHT20_ADDR 0x38
void aht20_init() { delay(40); Wire.beginTransmission(AHT20_ADDR); Wire.write(0xBE); Wire.write(0x08); Wire.write(0x00); Wire.endTransmission(); delay(10); }
void aht20_trigger() { Wire.beginTransmission(AHT20_ADDR); Wire.write(0xAC); Wire.write(0x33); Wire.write(0x00); Wire.endTransmission(); }
bool aht20_read(float &temp, float &hum) { 
  uint8_t buf[6]; if (Wire.requestFrom((uint8_t)AHT20_ADDR, (uint8_t)6) != 6) return false; 
  for (uint8_t i=0; i<6; i++) buf[i]=Wire.read(); 
  uint32_t hr=((uint32_t)buf[1]<<12)|((uint32_t)buf[2]<<4)|(buf[3]>>4); 
  uint32_t tr=((uint32_t)(buf[3]&0x0F)<<16)|((uint32_t)buf[4]<<8)|buf[5]; 
  hum = hr * 0.00009536743f; temp = tr * 0.00019073486f - 50.0f; return true; 
}

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

static float p_ref = 1013.25f;
float calc_altitude(float pres_hPa) { return 44330.0f * (1.0f - powf(pres_hPa / p_ref, 0.190284f)); }
void init_altitude_ref() {
  float sum = 0; int good = 0; const int N = 30;
  for (int i = 0; i < N; i++) { float t, p, h; if (bme280_read_all(t, p, h)) { sum += p; good++; } delay(50); }
  p_ref = good > 0 ? sum / good : p_ref;
}

// ════════════════════════════════════════════════════════════
//  GPS Parser (NMEA Checksum Yoxlaması)
// ════════════════════════════════════════════════════════════
bool verify_nmea_checksum(const char* nmea) {
  if (nmea[0] != '$') return false;
  uint8_t sum = 0; int i = 1;
  while (nmea[i] != '*' && nmea[i] != '\0' && i < 90) { sum ^= (uint8_t)nmea[i++]; }
  if (nmea[i] == '*') { long expected = strtol(&nmea[i+1], NULL, 16); return (sum == expected); }
  return false;
}

void parse_gps_line(char *line) {
  if (!verify_nmea_checksum(line)) return; 
  
  if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
    int field_idx = 0; char *p = line; char *token;
    float raw_lat = 0, raw_lon = 0, gps_alt = 0; char lat_dir = 'N', lon_dir = 'E'; int fix_quality = 0;
    while ((token = strsep(&p, ",")) != NULL) {
      if (field_idx == 6) fix_quality = atoi(token); else if (field_idx == 2) raw_lat = atof(token); else if (field_idx == 3) lat_dir = token[0]; else if (field_idx == 4) raw_lon = atof(token); else if (field_idx == 5) lon_dir = token[0]; else if (field_idx == 9) gps_alt = atof(token); field_idx++;
    }
    if (fix_quality > 0) {
      int deg_lat = (int)(raw_lat * 0.01f); float min_lat = raw_lat - (deg_lat * 100); float lat = deg_lat + (min_lat * 0.016666667f); if (lat_dir == 'S') lat = -lat;
      int deg_lon = (int)(raw_lon * 0.01f); float min_lon = raw_lon - (deg_lon * 100); float lon = deg_lon + (min_lon * 0.016666667f); if (lon_dir == 'W') lon = -lon;
      xSemaphoreTake(data_mutex, portMAX_DELAY); shared.gps_lat = lat; shared.gps_lon = lon; shared.gps_alt = gps_alt; shared.gps_fix = true; xSemaphoreGive(data_mutex);
    } else { xSemaphoreTake(data_mutex, portMAX_DELAY); shared.gps_fix = false; xSemaphoreGive(data_mutex); }
  }
}

// ════════════════════════════════════════════════════════════
//  TEST REJİMLƏRİ (SIT / SUT)
// ════════════════════════════════════════════════════════════
enum TestMode : uint8_t { MODE_NORMAL = 0, MODE_SIT, MODE_SUT };
volatile TestMode currentMode = MODE_NORMAL;
volatile bool testRunning = false;
volatile bool synDataValid = false;
volatile TestMode pendingMode = MODE_NORMAL;
volatile uint32_t pendingStartTime = 0;

float synAlt = 0.0f, synAccX = 0.0f, synAccY = 0.0f, synAccZ = 0.0f;
float synAngX = 0.0f, synAngY = 0.0f, synAngZ = 0.0f;
volatile uint16_t statusBits = 0;
volatile float apogeeAlt = 0.0f, prevAltForApogee = 0.0f;  

uint8_t calc_checksum(const uint8_t* data, uint8_t len) {
  uint16_t sum = 0; for (uint8_t i = 0; i < len; i++) sum += data[i]; return (uint8_t)(sum & 0xFF);
}

void send_sit_telemetry() {
  SensorData snap; xSemaphoreTake(data_mutex, portMAX_DELAY); memcpy(&snap, &shared, sizeof(SensorData)); xSemaphoreGive(data_mutex);
  uint8_t pkt[36]; uint8_t idx = 0; pkt[idx++] = 0xAB; 
  auto append_float = [&](float val) { uint8_t* b = (uint8_t*)&val; for (int i = 3; i >= 0; i--) pkt[idx++] = b[i]; };
  if (snap.altitude < 0.0f) snap.altitude = 0.0f;
  append_float(snap.altitude); append_float(snap.bme_pres); append_float(snap.accel_x); append_float(snap.accel_y);
  append_float(snap.accel_z); append_float(snap.angle_x); append_float(snap.angle_y); append_float(snap.angle_z);
  pkt[idx] = calc_checksum(pkt, idx); idx++; pkt[idx++] = 0x0D; pkt[idx++] = 0x0A;
  if (Serial6.availableForWrite() >= idx) Serial6.write(pkt, idx);
}

void parse_synthetic_data(const uint8_t* buf, uint8_t len) {
  if (len < 36 || buf[0] != 0xAB) return;
  if (calc_checksum(buf, 33) != buf[33]) return; 
  auto read_float = [&](uint8_t field_idx) {
    uint8_t tmp[4]; const uint8_t* src = buf + 1 + field_idx * 4;
    tmp[0] = src[3]; tmp[1] = src[2]; tmp[2] = src[1]; tmp[3] = src[0]; 
    float v; memcpy(&v, tmp, sizeof(float)); return v;
  };
  xSemaphoreTake(data_mutex, portMAX_DELAY);
  synAlt = read_float(0); synAccX = read_float(2); synAccY = read_float(3); synAccZ = read_float(4);
  synAngX = read_float(5); synAngY = read_float(6); synAngZ = read_float(7);
  xSemaphoreGive(data_mutex); synDataValid = true;
}

void send_sut_status() {
  uint8_t pkt[6]; pkt[0] = 0xAA; uint16_t bits; xSemaphoreTake(data_mutex, portMAX_DELAY); bits = statusBits; xSemaphoreGive(data_mutex);
  pkt[1] = (uint8_t)(bits & 0xFF); pkt[2] = (uint8_t)((bits >> 8) & 0xFF); pkt[3] = calc_checksum(pkt, 3); pkt[4] = 0x0D; pkt[5] = 0x0A;
  if (Serial6.availableForWrite() >= 6) Serial6.write(pkt, 6);
}

void update_status_bits(float altitude, float accelZ, float angleX) {
  uint16_t bits; float prev, apg;
  xSemaphoreTake(data_mutex, portMAX_DELAY); bits = statusBits; prev = prevAltForApogee; apg = apogeeAlt; xSemaphoreGive(data_mutex);
  if (altitude > 15.0f) bits |= (1 << 0); if (altitude > 100.0f) bits |= (1 << 1); if (altitude > 250.0f) bits |= (1 << 2);  
  if ((bits & (1 << 2)) && !(bits & (1 << 4))) { if (altitude > apg) { apg = altitude; apogeeAlt = altitude; } if (apg > 0 && altitude < apg - 10.0f && altitude > 50.0f) { bits |= (1 << 4); } }
  if ((bits & (1 << 4)) && apg > 0 && altitude < apg * 0.90f) { bits |= (1 << 5); }
  if ((bits & (1 << 5))) { if (altitude < apg * 0.22f) bits |= (1 << 7); } // Main
  bits &= ~((1 << 3) | (1 << 6)); if (angleX > 25.0f || fabs(accelZ) > 10.0f) bits |= (1 << 3); if (altitude < 500.0f) bits |= (1 << 6);
  xSemaphoreTake(data_mutex, portMAX_DELAY); prevAltForApogee = altitude; statusBits = bits; xSemaphoreGive(data_mutex);
}

void play_buzzer_tone(int frequency, int duration_ms) { tone(BUZZER_PIN, frequency, duration_ms); }

void start_sit() { if(currentMode!=MODE_NORMAL)return; pendingMode = MODE_SIT; pendingStartTime = millis(); play_buzzer_tone(400, 100); }
void start_sut() { if(currentMode!=MODE_NORMAL)return; xSemaphoreTake(data_mutex, portMAX_DELAY); statusBits = 0; apogeeAlt = 0.0f; prevAltForApogee = 0.0f; shared.apogee_fired=false; shared.main_fired=false; xSemaphoreGive(data_mutex); pendingMode = MODE_SUT; pendingStartTime = millis(); play_buzzer_tone(600, 150); }
void stop_test() { if(!testRunning && pendingMode==MODE_NORMAL)return; currentMode=MODE_NORMAL; testRunning=false; synDataValid=false; pendingMode=MODE_NORMAL; xSemaphoreTake(data_mutex, portMAX_DELAY); statusBits=0; xSemaphoreGive(data_mutex); play_buzzer_tone(200, 200); }

// ════════════════════════════════════════════════════════════
//  UÇUŞ MƏNTİQİ VƏ PİNLƏRİN İDARƏSİ (Barometr || IMU Füzionu)
// ════════════════════════════════════════════════════════════
void flight_control_task(void*) {
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    float cur_alt, cur_ay, cur_az; 
    bool ap_fired, mn_fired;
    
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    cur_alt = shared.altitude;
    cur_ay = shared.angle_y; 
    cur_az = shared.angle_z; 
    ap_fired = shared.apogee_fired;
    mn_fired = shared.main_fired;
    xSemaphoreGive(data_mutex);

    // Maksimum hündürlüyün yadda saxlanması
    if (cur_alt > max_flight_alt) {
      max_flight_alt = cur_alt;
    }

    // Təhlükəsizlik kilidi: Raket yerdən 50 metr qalxmadan sistemlər aktivləşmir
    if (max_flight_alt > 50.0f) { 
      
      // 1. APOGEY MƏNTİQİ (Barometr VƏ YA IMU)
      if (!ap_fired) {
        // Şərt 1: Maksimum hündürlükdən 3 metr aşağı düşmək
        bool is_falling = (cur_alt < (max_flight_alt - 3.0f));
        
        // Şərt 2: Külək və ya trayektoriya əyilməsi (60 dərəcədən çox əyilmə)
        float tilt_angle = max(fabs(cur_ay), fabs(cur_az)); 
        bool is_tilted = (tilt_angle > 60.0f);

        // FÜZİON: Hündürlük düşərsə YAXUD raket əyilərsə
        if (is_falling || is_tilted) {
          xSemaphoreTake(data_mutex, portMAX_DELAY); 
          shared.apogee_fired = true; 
          xSemaphoreGive(data_mutex);
          
          digitalWrite(PIN_APOGEE, HIGH); // Apogey paraşütü aktiv
          play_buzzer_tone(1500, 500);
        }
      }
      
      // 2. MAIN MƏNTİQİ: Qaydalara əsasən 500 Metr
      if (ap_fired && !mn_fired) {
        if (cur_alt <= 500.0f) {
          xSemaphoreTake(data_mutex, portMAX_DELAY); 
          shared.main_fired = true; 
          xSemaphoreGive(data_mutex);
          
          digitalWrite(PIN_MAIN, HIGH); // Əsas paraşüt aktiv
          play_buzzer_tone(2000, 800);
        }
      }
    }
    
    // 50Hz (20ms) reaksiyası - Çox sürətli yoxlama
    vTaskDelayUntil(&t, pdMS_TO_TICKS(20)); 
  }
}

// ════════════════════════════════════════════════════════════
//  DİGƏR TASKLAR 
// ════════════════════════════════════════════════════════════
void rs232_rx_task(void*) {
  static uint8_t buf[64]; static uint8_t idx = 0;
  for (;;) {
    while (Serial6.available()) {
      uint8_t c = Serial6.read();
      if (idx == 0 && c != 0xAA && c != 0xAB) continue; buf[idx++] = c;
      if (buf[0] == 0xAA && idx >= 5) {
        if (buf[3] == 0x0D && buf[4] == 0x0A) { if (buf[2] == calc_checksum(buf, 2)) { if (buf[1] == 0x20) start_sit(); else if (buf[1] == 0x22) start_sut(); else if (buf[1] == 0x24) stop_test(); } }
        idx = 0; continue;
      }
      if (buf[0] == 0xAB && idx >= 36) {
        if (buf[34] == 0x0D && buf[35] == 0x0A) { if (calc_checksum(buf, 33) == buf[33]) parse_synthetic_data(buf, 36); }
        idx = 0; continue;
      }
      if (idx >= sizeof(buf)) idx = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void test_tx_task(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    if (pendingMode != MODE_NORMAL && (millis() - pendingStartTime) >= 1000) { currentMode = pendingMode; testRunning = true; synDataValid = false; pendingMode = MODE_NORMAL; }
    if (testRunning) {
      if (currentMode == MODE_SIT) { send_sit_telemetry(); } 
      else if (currentMode == MODE_SUT) {
        if (synDataValid) {
          float snapAlt, snapAccZ, snapAngX; xSemaphoreTake(data_mutex, portMAX_DELAY); snapAlt = synAlt; snapAccZ = synAccZ; snapAngX = synAngX; xSemaphoreGive(data_mutex);
          update_status_bits(snapAlt, snapAccZ, snapAngX);
        }
        send_sut_status();
      }
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));
  }
}

void imu_task(void*) {
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    uint32_t now = millis();
    if (currentMode == MODE_SUT && synDataValid) {
      xSemaphoreTake(data_mutex, portMAX_DELAY); shared.angle_x = synAngX; shared.angle_y = synAngY; shared.angle_z = synAngZ; shared.accel_x = synAccX; shared.accel_y = synAccY; shared.accel_z = synAccZ; xSemaphoreGive(data_mutex);
    } else {
      if (i2c_should_try(health_bno, now)) {
        float qw, qx, qy, qz, ax, ay, az, ex, ey, ez;
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        bool okQ = bno055_read_quat(qw, qx, qy, qz); bool okA = bno055_read_accel(ax, ay, az); bool okE = bno055_read_euler(ex, ey, ez);
        xSemaphoreGive(i2c_mutex);
        i2c_mark_result(health_bno, okQ && okA && okE, now);
        if (okQ && okA && okE) {
          qslerp.update(qw, qx, qy, qz);
          xSemaphoreTake(data_mutex, portMAX_DELAY);
          shared.qw = qslerp.w; shared.qx = qslerp.x; shared.qy = qslerp.y; shared.qz = qslerp.z;
          shared.accel_x = ax; shared.accel_y = ay; shared.accel_z = az; shared.angle_x = ex; shared.angle_y = ey; shared.angle_z = ez;
          xSemaphoreGive(data_mutex);
        }
      }
    }
    vTaskDelayUntil(&t, pdMS_TO_TICKS(20));
  }
}

void env_task(void*) {
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    uint32_t now = millis();
    bool bme_ok = false; float bt=0, bp=0, bh=0;
    if (i2c_should_try(health_bme, now)) { 
      xSemaphoreTake(i2c_mutex, portMAX_DELAY); 
      bme_ok = bme280_read_all(bt, bp, bh); 
      xSemaphoreGive(i2c_mutex); 
      i2c_mark_result(health_bme, bme_ok, now); 
    }
    
    bool aht_try = i2c_should_try(health_aht, now);
    if (aht_try) { xSemaphoreTake(i2c_mutex, portMAX_DELAY); aht20_trigger(); xSemaphoreGive(i2c_mutex); }
    
    if (currentMode == MODE_SUT && synDataValid) {
      xSemaphoreTake(data_mutex, portMAX_DELAY); shared.altitude = synAlt; xSemaphoreGive(data_mutex);
    } else {
      if (bme_ok) {
        float f_bt = kf_bme_temp.update(bt), f_bh = kf_bme_hum.update(bh), f_bp = kf_bme_pres.update(bp);
        float f_alt = kf_altitude.update(calc_altitude(f_bp));
        xSemaphoreTake(data_mutex, portMAX_DELAY); shared.bme_temp = f_bt; shared.bme_hum = f_bh; shared.bme_pres = f_bp; shared.altitude = f_alt; xSemaphoreGive(data_mutex);
      }
    }
    // 50Hz tezlik (20ms)
    vTaskDelayUntil(&t, pdMS_TO_TICKS(20));
  }
}

void gps_task(void*) {
  static char gps_buf[100]; static int buf_idx = 0;
  for (;;) {
    while (Serial2.available()) {
      char c = Serial2.read();
      if (c == '\n' || c == '\r') { if (buf_idx > 0) { gps_buf[buf_idx] = '\0'; parse_gps_line(gps_buf); buf_idx = 0; } } else if (buf_idx < 99) { gps_buf[buf_idx++] = c; }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void tx_task(void*) {
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    SensorData snap;
    xSemaphoreTake(data_mutex, portMAX_DELAY); memcpy(&snap, &shared, sizeof(SensorData)); xSemaphoreGive(data_mutex);

    // CİHAZ ID-si (13) VƏ YER STANSİYASINA MƏLUMAT AXINI
    Serial3.print("13,AA,"); 
    Serial3.print(millis()); Serial3.print(',');
    Serial3.print(snap.qw, 3); Serial3.print(','); Serial3.print(snap.qx, 3); Serial3.print(',');
    Serial3.print(snap.qy, 3); Serial3.print(','); Serial3.print(snap.qz, 3); Serial3.print(',');
    Serial3.print(snap.accel_x, 2); Serial3.print(','); Serial3.print(snap.accel_y, 2); Serial3.print(','); Serial3.print(snap.accel_z, 2); Serial3.print(',');
    Serial3.print(snap.angle_x, 1); Serial3.print(','); Serial3.print(snap.angle_y, 1); Serial3.print(','); Serial3.print(snap.angle_z, 1); Serial3.print(',');
    Serial3.print(snap.bme_temp, 1); Serial3.print(','); Serial3.print(snap.bme_pres, 1); Serial3.print(',');
    Serial3.print(snap.bme_hum, 1); Serial3.print(','); Serial3.print(snap.altitude, 1); Serial3.print(',');
    Serial3.print(snap.aht_temp, 1); Serial3.print(','); Serial3.print(snap.aht_hum, 1); Serial3.print(',');
    Serial3.print(snap.gps_lat, 6); Serial3.print(','); Serial3.print(snap.gps_lon, 6); Serial3.print(',');
    Serial3.print(snap.gps_alt, 1); Serial3.print(','); Serial3.print(snap.gps_fix ? 1 : 0);
    
    // Statuslar Snap-dən oxunur (Data Race yoxdur)
    Serial3.print(','); Serial3.print(snap.apogee_fired ? 1 : 0);
    Serial3.print(','); Serial3.print(snap.main_fired ? 1 : 0);

    Serial3.println();
    vTaskDelayUntil(&t, pdMS_TO_TICKS(66));  // 15 Hz
  }
}

void setup() {
  Serial1.begin(115200); 
  Serial2.begin(9600); 
  Serial3.begin(115200); 
  Serial6.begin(115200);
  
  // Serial TX buffer (Ac qalmanın - Starvation qarşısını alır)
  #if defined(USBCON) || defined(HAL_UART_MODULE_ENABLED)
    // Əgər framework dəstəkləyirsə Serial3.setTxBufferSize(256);
  #endif

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIN_APOGEE, OUTPUT); digitalWrite(PIN_APOGEE, LOW);
  pinMode(PIN_MAIN, OUTPUT); digitalWrite(PIN_MAIN, LOW);
  delay(1500);
  
  Wire.begin(); Wire.setClock(400000);
  bme280_init(); aht20_init(); bno055_init(); init_altitude_ref();
  
  kf_bme_temp.init(0.005f, 0.25f); kf_bme_hum.init(0.05f, 9.0f); kf_bme_pres.init(0.05f, 1.0f);
  kf_altitude.init(0.1f, 4.0f); kf_aht_temp.init(0.005f, 0.09f); kf_aht_hum.init(0.05f, 4.0f); qslerp.init(0.15f);
  
  i2c_mutex = xSemaphoreCreateMutex(); data_mutex = xSemaphoreCreateMutex();
  
  xTaskCreate(tx_task, "TX", 512, NULL, 4, NULL);
  xTaskCreate(imu_task, "IMU", 512, NULL, 3, NULL);
  xTaskCreate(gps_task, "GPS", 384, NULL, 2, NULL);
  xTaskCreate(env_task, "ENV", 512, NULL, 1, NULL); 
  xTaskCreate(rs232_rx_task, "RS232_RX", 512, NULL, 3, NULL);
  xTaskCreate(test_tx_task, "TEST_TX", 512, NULL, 3, NULL);
  xTaskCreate(flight_control_task, "FLIGHT", 512, NULL, 4, NULL);
  
  vTaskStartScheduler();
}
void loop() {}