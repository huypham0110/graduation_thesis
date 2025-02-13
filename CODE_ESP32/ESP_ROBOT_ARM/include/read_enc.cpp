
void as5600_bus(volatile byte bus) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}
void encoder_init() {
  Wire.begin();
  as5600_bus(0);
  as5600_1.begin(4);                         //  set direction pin.
  as5600_1.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.

  as5600_bus(1);
  as5600_2.begin(4);                         //  set direction pin.
  as5600_2.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.

  as5600_bus(2);
  as5600_3.begin(4);                         //  set direction pin.
  as5600_3.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  Serial.println(as5600_3.getAddress());

  as5600_bus(0);
  F_SETPOINT_POS = (as5600_1.getCumulativePosition() / F_RATIO);
  as5600_bus(1);
  M_SETPOINT_POS = (as5600_2.getCumulativePosition() / M_RATIO);
  as5600_bus(2);
  L_SETPOINT_POS = (as5600_3.getCumulativePosition() / L_RATIO);
}

 void encoder_read() {
  as5600_bus(0);
  F_EN_RAW = -as5600_1.getCumulativePosition();
  F_ENCODER_POS = (int32_t)((F_EN_RAW*100)/2048) - F_SETPOINT_POS;
  as5600_bus(1);
  M_EN_RAW = -as5600_2.getCumulativePosition();
  M_ENCODER_POS = (M_EN_RAW / M_RATIO) - M_SETPOINT_POS;
  as5600_bus(2);
  L_EN_RAW = as5600_3.getCumulativePosition();
  L_ENCODER_POS = (L_EN_RAW / L_RATIO) - L_SETPOINT_POS;
}