void setup() {
Serial.begin(115200);
Serial1.begin(115200);
}
void loop() {
  char send_data[50];
  char delimiter='>';
  int data[12];
  data[0]= random(10,99);
  data[1]= random(10,99);
  data[2]= random(10,99);
  data[3]= random(10,99);
  data[4]= random(10,99);
  data[5]= random(10,99);
  data[6]= random(10,99);
  data[7]= random(10,99);
  data[8]= random(10,99);
  data[9]= random(10,99);
  sprintf(send_data, "%d%d%d%d%d%d%d%d%d%c",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],'>');
  String send_string="011121314151617121516>";
  String send_string2 = "1345>";
  if (Serial1.available() > 0 )
  {
    String cmdString = "at+qd1?";
    String recString = Serial1.readStringUntil('\n');
    int index = recString.indexOf('\r');
    String parsedString = recString.substring(0, index);
    Serial.println(send_data);
    if (parsedString.equals(cmdString))
    {
      //Serial2.println(gyro_buf);
      Serial1.print(send_string2);
    }
  }


//if(Serial.available()){
//Serial1.write(Serial.read());  
//}
}
