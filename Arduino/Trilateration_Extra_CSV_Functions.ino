#include <SPI.h>
#include <RH_RF95.h>
#include <vector>
#include <cmath>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define LED_PIN 13

#define RSSI_BUFFER_SIZE 6    // Rolling RSSI buffer per transmitter, increase if in noisy enviornemnt but will make the code slower to respond to movement
#define EMA_ALPHA 0.45         // Exponential moving average alpha
#define STALE_TIMEOUT_MS 4500  // 3 seconds timeout for stale transmitters

RH_RF95 rf95(RFM95_CS, RFM95_INT);

struct Transmitter {
  String id;
  float latitude;
  float longitude;
  float x; // Local flat-earth X (m)
  float y; // Local flat-earth Y (m)
  float distance; // Estimated distance from RSSI
  bool gpsReceived;

  // RSSI filtering
  float rssiBuffer[RSSI_BUFFER_SIZE];
  int rssiIndex = 0;
  bool bufferFilled = false;
  float filteredRSSI = -50; // Initial RSSI estimate

  // Stale timer
  unsigned long lastActive = 0;
  bool isStale = false;
};

std::vector<Transmitter> txs;
bool originSet = false;
float originLat = 0.0;
float originLon = 0.0;
const float EARTH_RADIUS_M = 6371000.0;

// Previous receiver position for smoothing
float rx_prev = 0.0;
float ry_prev = 0.0;

// -------------------- RSSI to Distance --------------------
// Full-range model (1–100 m regression fit)
// RSSI(d) = -24.952*log10(d) - 41.565

// Short-Mid-range model (1–70 m regression fit)
// RSSI(d) = -23.979*log10(d) - 42.667

float rssiToDistance(float rssi) {
  const float P0 = -41.565f;   // RSSI at 1 m (from intercept)
  const float n  = 2.4952f;    // Path loss exponent (from slope/10)
  float exp10 = (P0 - rssi) / (10.0f * n);
  return pow(10.0f, exp10);
}

// -------------------- Median Filter --------------------
float median(float arr[], int size) {
  float temp[size];
  memcpy(temp, arr, size * sizeof(float));
  for (int i = 0; i < size-1; i++) {
    for (int j = i+1; j < size; j++) {
      if (temp[j] < temp[i]) {
        float t = temp[i]; temp[i] = temp[j]; temp[j] = t;
      }
    }
  }
  return (size % 2 == 0) ? (temp[size/2-1] + temp[size/2]) / 2.0 : temp[size/2];
}

// -------------------- Trilateration --------------------
bool trilaterateLS(float &rx, float &ry, const std::vector<Transmitter*> &activeTxs) {
  int n = activeTxs.size();
  if (n < 3) return false;

  float x1 = activeTxs[0]->x, y1 = activeTxs[0]->y, r1 = activeTxs[0]->distance;
  float sumAxx=0, sumAxy=0, sumAyy=0, sumBx=0, sumBy=0;

  for (int i=1; i<n; i++) {
    float xi = activeTxs[i]->x, yi = activeTxs[i]->y, ri = activeTxs[i]->distance;
    float Ai = 2*(xi - x1);
    float Bi = 2*(yi - y1);
    float Ci = r1*r1 - ri*ri + (xi*xi - x1*x1) + (yi*yi - y1*y1);
    sumAxx += Ai*Ai; sumAxy += Ai*Bi; sumAyy += Bi*Bi;
    sumBx += Ai*Ci; sumBy += Bi*Ci;
  }

  float det = sumAxx*sumAyy - sumAxy*sumAxy;
  if (det == 0) return false;

  rx = (sumBx*sumAyy - sumBy*sumAxy) / det;
  ry = (sumBy*sumAxx - sumBx*sumAxy) / det;
  return true;
}

// -------------------- Setup --------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(9600); while (!Serial);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW); delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  if (!rf95.init()) { Serial.println("LoRa init failed"); while(1); }
  if (!rf95.setFrequency(RF95_FREQ)) { Serial.println("Frequency set failed"); while(1); }
  rf95.setTxPower(23,false);
  Serial.println("Receiver ready.");
}

// -------------------- Loop --------------------
void loop() {
  if (!rf95.available()) return;

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (!rf95.recv(buf,&len)) return;
  buf[len] = '\0';
  String msg = String((char*)buf);

  // Parse ID and GPS (updated: only 1 comma now)
  int sepIndex = msg.indexOf(':');
  if (sepIndex<=0) return;
  String id = msg.substring(0,sepIndex);
  String content = msg.substring(sepIndex+1);

  int commaIndex = content.indexOf(',');
  if (commaIndex < 0) return; // reject if malformed

  float lat = content.substring(0, commaIndex).toFloat();
  float lon = content.substring(commaIndex+1).toFloat();
  float rssiMeasured = rf95.lastRssi();

  bool known=false;
  for (auto &tx : txs) {
    if (tx.id==id) {
      known=true;

      // Update last active timestamp
      unsigned long now = millis();
      if (tx.isStale) {
        tx.isStale = false;
        Serial.print("Transmitter "); Serial.print(tx.id); Serial.println(" is active again.");
      }
      tx.lastActive = now;

      if (!tx.gpsReceived) {
        tx.latitude = lat; tx.longitude = lon; tx.gpsReceived = true;
        if (!originSet) { originLat=lat; originLon=lon; originSet=true; }

        float dLat = radians(lat-originLat), dLon = radians(lon-originLon);
        float avgLat = radians((lat+originLat)/2.0);
        tx.x = dLon*EARTH_RADIUS_M*cos(avgLat);
        tx.y = dLat*EARTH_RADIUS_M;

        Serial.print("Stored "); Serial.print(id);
        Serial.print(" | Lat: "); Serial.print(lat,6);
        Serial.print(" | Lon: "); Serial.print(lon,6);
        Serial.print(" | X: "); Serial.print(tx.x,2);
        Serial.print(" m | Y: "); Serial.println(tx.y,2);
      }

      // RSSI filtering
      tx.rssiBuffer[tx.rssiIndex] = rssiMeasured;
      tx.rssiIndex = (tx.rssiIndex+1)%RSSI_BUFFER_SIZE;
      if (tx.rssiIndex==0) tx.bufferFilled = true;
      int size = tx.bufferFilled ? RSSI_BUFFER_SIZE : tx.rssiIndex;
      float medRSSI = median(tx.rssiBuffer,size);
      tx.filteredRSSI = EMA_ALPHA*medRSSI + (1-EMA_ALPHA)*tx.filteredRSSI;
      tx.distance = rssiToDistance(tx.filteredRSSI);

      // --- Raw RSSI + Distance ---
      Serial.print(id); Serial.print(" | Raw RSSI: "); Serial.print(rssiMeasured,2);
      Serial.print(" | Raw Distance: "); Serial.println(rssiToDistance(rssiMeasured),2);

      // --- Filtered RSSI + Distance ---
      Serial.print(id); Serial.print(" | RSSI: "); Serial.print(tx.filteredRSSI,2);
      Serial.print(" | Distance: "); Serial.println(tx.distance,2);
      break;
    }
  }

  if (!known) {
    Transmitter newTx;
    newTx.id=id; newTx.latitude=lat; newTx.longitude=lon; newTx.gpsReceived=true;

    unsigned long now = millis();
    newTx.lastActive = now;
    newTx.isStale = false;

    if (!originSet) { originLat=lat; originLon=lon; originSet=true; }

    float dLat = radians(lat-originLat), dLon = radians(lon-originLon);
    float avgLat = radians((lat+originLat)/2.0);
    newTx.x = dLon*EARTH_RADIUS_M*cos(avgLat);
    newTx.y = dLat*EARTH_RADIUS_M;

    for (int i=0;i<RSSI_BUFFER_SIZE;i++) newTx.rssiBuffer[i]=rssiMeasured;
    newTx.rssiIndex=0; newTx.bufferFilled=false; newTx.filteredRSSI=rssiMeasured;
    newTx.distance=rssiToDistance(rssiMeasured);

    txs.push_back(newTx);

    Serial.print("Stored "); Serial.print(id);
    Serial.print(" | Lat: "); Serial.print(lat,6);
    Serial.print(" | Lon: "); Serial.print(lon,6);
    Serial.print(" | X: "); Serial.print(newTx.x,2);
    Serial.print(" m | Y: "); Serial.println(newTx.y,2);

    // --- Raw RSSI + Distance ---
    Serial.print(id); Serial.print(" | Raw RSSI: "); Serial.print(rssiMeasured,2);
    Serial.print(" | Raw Distance: "); Serial.println(rssiToDistance(rssiMeasured),2);

    // --- Filtered RSSI + Distance ---
    Serial.print(id); Serial.print(" | RSSI: "); Serial.print(newTx.filteredRSSI,2);
    Serial.print(" | Distance: "); Serial.println(newTx.distance,2);
  }

  // -------------------- Check for stale transmitters --------------------
  unsigned long now = millis();
  for (auto &tx : txs) {
    if (!tx.isStale && (now - tx.lastActive > STALE_TIMEOUT_MS)) {
      tx.isStale = true;
      Serial.print("Transmitter "); Serial.print(tx.id); Serial.println(" is stale.");
    }
  }

  // -------------------- Trilateration --------------------
  std::vector<Transmitter*> activeTxs;
  for (auto &tx : txs) {
    if (!tx.isStale) activeTxs.push_back(&tx);
  }

  if (activeTxs.size() >= 3) {
    float rx, ry;
    if (trilaterateLS(rx, ry, activeTxs)) {
      // --- Raw Position (before EMA smoothing) ---
      float lat_raw = originLat + (ry/EARTH_RADIUS_M)*(180.0/PI);
      float avgLat_raw = radians(originLat);
      float lon_raw = originLon + (rx/(EARTH_RADIUS_M*cos(avgLat_raw)))*(180.0/PI);
      Serial.print("Raw Estimated Receiver: X = "); Serial.print(rx,2);
      Serial.print(" | Y = "); Serial.print(ry,2);
      Serial.print(" | Lat = "); Serial.print(lat_raw,6);
      Serial.print(" | Lon = "); Serial.println(lon_raw,6);

      // --- Apply EMA smoothing ---
      rx = EMA_ALPHA*rx + (1-EMA_ALPHA)*rx_prev;
      ry = EMA_ALPHA*ry + (1-EMA_ALPHA)*ry_prev;
      rx_prev=rx; ry_prev=ry;

      float lat_est = originLat + (ry/EARTH_RADIUS_M)*(180.0/PI);
      float avgLat = radians(originLat);
      float lon_est = originLon + (rx/(EARTH_RADIUS_M*cos(avgLat)))*(180.0/PI);

      Serial.print("Estimated Receiver: X = "); Serial.print(rx,2);
      Serial.print(" | Y = "); Serial.print(ry,2);
      Serial.print(" | Lat = "); Serial.print(lat_est,6);
      Serial.print(" | Lon = "); Serial.println(lon_est,6);
    }
  }
}
