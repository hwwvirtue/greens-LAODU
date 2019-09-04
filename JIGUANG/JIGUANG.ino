float JG_R1[5];
float JG_R2[5];
void setup() {
  Serial.begin(19200);
  Serial1.begin(19200);
  Serial2.begin(19200);
}
void loop() {
  Serial1.print('D');
  for (int i = 0; i < 2; i++)
  JG_R1[i] = Serial1.parseFloat();
  Serial.println(JG_R1[0]);
  Serial2.print('D');
  for (int i = 0; i < 2; i++)
  JG_R2[i] = Serial2.parseFloat();
  Serial.println(JG_R2[0]);
}
