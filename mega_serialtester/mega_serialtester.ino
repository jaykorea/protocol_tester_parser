void setup() {
Serial.begin(115200);
Serial1.begin(115200);
}
void loop() {
// Check for received Characters from the computer
if (Serial.available())
{
// Write what is received to the default serial port
Serial1.write(Serial.read());
}
}
