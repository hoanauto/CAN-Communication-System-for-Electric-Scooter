#include <SPI.h>
#include <mcp_can.h>
// Node32s,115200,com6
#define CAN_CS 5   // Chân CS của MCP2515 nối với GPIO5
MCP_CAN mcp2515(5);  // Chân CS kết nối với GPIO 5 trên ESP32

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 started");
  pinMode(2, OUTPUT); // GPIO2 thường là LED tích hợp
  digitalWrite(2, HIGH); // bật LED
  delay(1000);
  digitalWrite(2, LOW); // tắt LED
  if (mcp2515.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN init ok");
  } else {
    Serial.println("CAN init fail");
    while (1);
  }

  mcp2515.setMode(MCP_NORMAL);  // Chế độ nhận/gửi dữ liệu
}

void loop() {
  if (CAN_MSGAVAIL == mcp2515.checkReceive()) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];

    mcp2515.readMsgBuf(&rxId, &len, rxBuf);
 Serial.println("------ CAN Frame Received ------");
    Serial.print("ID: 0x"); Serial.println(rxId, HEX);
    Serial.print("DLC: "); Serial.println(len);

    if (rxId == 0x202 && len == 8) {
      uint16_t Vpin_adc = (rxBuf[0] << 8) | rxBuf[1];
      uint16_t temperature_adc = (rxBuf[2] << 8) | rxBuf[3];
      bool headlight = (rxBuf[4] >> 3) & 0x01;
      bool turn_left = (rxBuf[4] >> 2) & 0x01;
      bool turn_right = (rxBuf[4] >> 1) & 0x01;
      bool brake = (rxBuf[4] >> 0) & 0x01;
      Serial.println("------ Received CAN Data ------");
      Serial.print("Vpin ADC: "); Serial.println(current_adc);
      Serial.print("Temperature ADC: "); Serial.println(temperature_adc);
      Serial.print("Headlight: "); Serial.println(headlight);
      Serial.print("Turn Left: "); Serial.println(turn_left);
      Serial.print("Turn Right: "); Serial.println(turn_right);
      Serial.print("Brake: "); Serial.println(brake);
    }
    else if(rxId == 0x201 && len == 8) 
    { uint16_t throttle_adc = (rxBuf[0] << 8) | rxBuf[1];
     Serial.println("==> From Node 2 [0x201]");
      Serial.print("Throttle ADC: "); Serial.println(throttle_adc);
    }
  }
}

