#include <MsTimer2.h>

volatile int send_data_flag = 0;

void toggleFlag();

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  
  MsTimer2::set(5000, toggleFlag);
  MsTimer2::start();
}

void loop() {  
  char send_data1[22]; // Increased the size by 1 to accommodate the delimiter character
  char send_data2[22]; // Increased the size by 1 to accommodate the delimiter character
  char send_data3[22]; // Increased the size by 1 to accommodate the delimiter character
  char send_data4[22]; // Increased the size by 1 to accommodate the delimiter character
  char send_data5[22]; // Increased the size by 1 to accommodate the delimiter character
  char delimiter = '>';
  
  String data1[8];
  data1[0]="00"; //model status - 00(Y), 11(N)
  data1[1]="03"; //result - 00(front), 01(up), 02(down), 03(left), 04(right)
  data1[2]="01"; //result - 00(mouth closed), 01(mouth opened)
  data1[3]="01"; //result - 00(eyes opend), 01(eyes closed)
  data1[4]="00"; //model status - 00(Y), 11(N)
  data1[5]="04"; //result - 00(angry), 01(happy), 02(neutral), 03(sad), 04(surprise)
  data1[6]="00"; //model status - 00(Y), 11(N)
  data1[7]="00"; //result - 00(positive), 11(negative)

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

  String data4[8];
  data4[0]="00"; //model status - 00(Y), 11(N)
  data4[1]="01"; //result - 00(front), 01(up), 02(down), 03(left), 04(right)
  data4[2]="00"; //result - 00(mouth closed), 01(mouth opened)
  data4[3]="01"; //result - 00(eyes opend), 01(eyes closed)
  data4[4]="00"; //model status - 00(Y), 11(N)
  data4[5]="01"; //result - 00(angry), 01(happy), 02(neutral), 03(sad), 04(surprise)
  data4[6]="00"; //model status - 00(Y), 11(N)
  data4[7]="00"; //result - 00(positive), 11(negative)

  String data5[8];
  data5[0]="00"; //model status - 00(Y), 11(N)
  data5[1]="00"; //result - 00(front), 01(up), 02(down), 03(left), 04(right)
  data5[2]="00"; //result - 00(mouth closed), 01(mouth opened)
  data5[3]="00"; //result - 00(eyes opend), 01(eyes closed)
  data5[4]="00"; //model status - 00(Y), 11(N)
  data5[5]="01"; //result - 00(angry), 01(happy), 02(neutral), 03(sad), 04(surprise)
  data5[6]="00"; //model status - 00(Y), 11(N)
  data5[7]="00"; //result - 00(positive), 11(negative)
  
  sprintf(send_data1, "%s%s%s%s%s%s%s%s%c", data1[0].c_str(), data1[1].c_str(), data1[2].c_str(), data1[3].c_str(), data1[4].c_str(), data1[5].c_str(), data1[6].c_str(), data1[7].c_str(), delimiter);
  sprintf(send_data2, "%s%s%s%s%s%s%s%s%c", data2[0].c_str(), data2[1].c_str(), data2[2].c_str(), data2[3].c_str(), data2[4].c_str(), data2[5].c_str(), data2[6].c_str(), data2[7].c_str(), delimiter);
  sprintf(send_data3, "%s%s%s%s%s%s%s%s%c", data3[0].c_str(), data3[1].c_str(), data3[2].c_str(), data3[3].c_str(), data3[4].c_str(), data3[5].c_str(), data3[6].c_str(), data3[7].c_str(), delimiter);
  sprintf(send_data4, "%s%s%s%s%s%s%s%s%c", data4[0].c_str(), data4[1].c_str(), data4[2].c_str(), data4[3].c_str(), data4[4].c_str(), data4[5].c_str(), data4[6].c_str(), data4[7].c_str(), delimiter);
  sprintf(send_data5, "%s%s%s%s%s%s%s%s%c", data5[0].c_str(), data5[1].c_str(), data5[2].c_str(), data5[3].c_str(), data5[4].c_str(), data5[5].c_str(), data5[6].c_str(), data5[7].c_str(), delimiter);

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
        Serial.println(send_data1);
        Serial2.print(send_data1); 
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
      else if(send_data_flag == 3) {
        Serial.print("Send Data4: ");
        Serial.println(send_data4);
        Serial2.print(send_data4);
      }
      else if(send_data_flag == 4) {
        Serial.print("Send Data5: ");
        Serial.println(send_data5);
        Serial2.print(send_data5);
      }
    }
  }
}

void toggleFlag() {
  send_data_flag++;
  if (send_data_flag > 4) send_data_flag = 0;
}
