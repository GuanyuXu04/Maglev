#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // I2C 速度，400kHz 通常没问题
  Wire.setClock(400000);

  sensor.setTimeout(500);

  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize VL53L1X!");
    while (1);
  }

  // Long 模式：量程更远，适合一般测距
  sensor.setDistanceMode(VL53L1X::Short);

  // 单次测量时间，单位是微秒
  // 50000 = 50ms，数值越大越稳定，但刷新越慢
  sensor.setMeasurementTimingBudget(5000);

  // 连续测量，每 50ms 测一次
  sensor.startContinuous(5);

  Serial.println("VL53L1X started.");
}

void loop() {
  int distance = sensor.read();   // 单位：mm

  if (sensor.timeoutOccurred()) {
    Serial.println("Sensor timeout!");
    return;
  }

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" mm");

  delay(50);
}