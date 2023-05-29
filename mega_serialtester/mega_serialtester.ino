#include <MsTimer2.h>

volatile int send_data_flag = 0;

void toggleFlag();

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  
  MsTimer2::set(3000, toggleFlag);
  MsTimer2::start();
}

void loop() {  
  char send_data[22]; // Increased the size by 1 to accommodate the delimiter character
  char send_data2[22]; // Increased the size by 1 to accommodate the delimiter character
  char send_data3[22]; // Increased the size by 1 to accommodate the delimiter character
  char delimiter = '>';
  
  String data[8];
  data[0]="00"; //model status - 00(Y), 11(N)
  data[1]="03"; //result - 00(front), 01(up), 02(down), 03(left), 04(right)
  data[2]="01"; //result - 00(mouth closed), 01(mouth opened)
  data[3]="01"; //result - 00(eyes opend), 01(eyes closed)
  data[4]="00"; //model status - 00(Y), 11(N)
  data[5]="04"; //result - 00(angry), 01(happy), 02(neutral), 03(sad), 04(surprise)
  data[6]="00"; //model status - 00(Y), 11(N)
  data[7]="00"; //result - 00(positive), 11(negative)

  String data2[8];
  data2[0]="00"; //model status - 00(Y), 11(N)
  data2[1]="03"; //result - 00(front), 01(up), 02(down), 03(left), 04(right)
  data2[2]="00"; //result - 00(mouth closed), 01(mouth opened)
  data2[3]="01"; //result - 00(eyes opend), 01(eyes closed)
  data2[4]="00"; //model status - 00(Y), 11(N)
  data2[5]="01"; //result - 00(angry), 01(happy), 02(neutral), 03(sad), 04(surprise)
  data2[6]="00"; //model status - 00(Y), 11(N)
  data2[7]="00"; //result - 00(positive), 11(negative)

  String data3[8];
  data3[0]="00"; //model status - 00(Y), 11(N)
  data3[1]="00"; //result - 00(front), 01(up), 02(down), 03(left), 04(right)
  data3[2]="00"; //result - 00(mouth closed), 01(mouth opened)
  data3[3]="00"; //result - 00(eyes opend), 01(eyes closed)
  data3[4]="00"; //model status - 00(Y), 11(N)
  data3[5]="01"; //result - 00(angry), 01(happy), 02(neutral), 03(sad), 04(surprise)
  data3[6]="00"; //model status - 00(Y), 11(N)
  data3[7]="00"; //result - 00(positive), 11(negative)
  
  sprintf(send_data, "%s%s%s%s%s%s%s%s%c", data[0].c_str(), data[1].c_str(), data[2].c_str(), data[3].c_str(), data[4].c_str(), data[5].c_str(), data[6].c_str(), data[7].c_str(), delimiter);
  sprintf(send_data2, "%s%s%s%s%s%s%s%s%c", data2[0].c_str(), data2[1].c_str(), data2[2].c_str(), data2[3].c_str(), data2[4].c_str(), data2[5].c_str(), data2[6].c_str(), data2[7].c_str(), delimiter);
  sprintf(send_data3, "%s%s%s%s%s%s%s%s%c", data3[0].c_str(), data3[1].c_str(), data3[2].c_str(), data3[3].c_str(), data3[4].c_str(), data3[5].c_str(), data3[6].c_str(), data3[7].c_str(), delimiter);

  if (Serial2.available() > 0) {
    String cmdString = "at+qd1?";
    String recString = Serial2.readStringUntil('\n');
    recString.trim(); // Remove leading/trailing whitespace

//    Serial.print("Received String: [");
//    Serial.print(recString);
//    Serial.println("]");

    if (recString.equals(cmdString)) {
      if (send_data_flag == 0) {
        Serial.print("Send Data: ");
        Serial.println(send_data);
        Serial2.print(send_data); 
      }
      else if(send_data_flag == 1) {
        Serial.print("Send Data2: ");
        Serial.println(send_data2);
        Serial2.print(send_data2);
      }
      else if(send_data_flag == 2) {
        Serial.print("Send Data3: ");
        Serial.println(send_data3);
        Serial2.print(send_data3);
      }
    }
  }
}

void toggleFlag() {
  send_data_flag++;
  if (send_data_flag >= 3) send_data_flag = 0;
}
