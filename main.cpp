#include <Arduino.h>

union FloatToByte{
  float asFloat;
  uint8_t asByte[4]; 
}foo;

union floatPack
{
    float asFloat    ;
    uint8_t asByte[4] ; 
} ;

union intPack 
{
    int16_t asInt ;
    uint8_t  asByte[2] ;
};


struct Position // size 10 byte
{
    // uint8_t      header[2]  = { 0xFF , 0xFF } ; 
    uint8_t      lenght     = 9               ;
    uint8_t      data_type  = 'P'             ; 
    floatPack    position_x                    ; 
    floatPack    position_y                    ; 
    // uint8_t       check_sum                     ; 
}pos_package;

struct Position_Lagoli_config // size 21 byte 
{
    // uint8_t      header[2]  = { 0xFF , 0xFF } ; 
    uint8_t      lenght     = 20              ;
    uint8_t      data_type_1  = 'P'             ; 
    floatPack    position_x                   ; 
    floatPack    position_y                   ;
    uint8_t      data_type_2  = 'L'             ; 
    floatPack    lago_x                       ; 
    floatPack    lago_y                       ;
    intPack      lago_ang                     ;
    // uint8_t      check_sum                    ;
}pos_lago_package ; 

struct Shooter  // size 14 byte
{
    // uint8_t      header[2]  = { 0xFF , 0xFF } ; 
    uint8_t      lenght     = 13              ;
    uint8_t      data_type  = 'S'             ; 
    floatPack    time                         ; 
    floatPack    time_shoot                   ;
    intPack      theta                        ;
    intPack      phi                          ;
    // uint8_t       check_sum                    ;
}shooter_package;

// uint8_t data[22] ;

uint8_t calChecksum(uint8_t data_in[] , int size )
{
    uint8_t checksum = 0 ;

    for(int i =2 ;i < size ;i++)
    {
            checksum = checksum + data_in[i] ;
            // std::cout << std::hex << (int)data[i] ; 
            // std::cout << '\n' ;
    }
    checksum = ~ ( checksum & 0xFF );
    // std::cout << '\n' ;
    return checksum ;
}

void SendLagori();
void SendPosition();
void SendPositionLagori();
/*
-send position 
-read pos_package
-read lagori
-read shoote ( teensy ตัวเดียว )
*/


void setup() 
{
  Serial.begin(9600);
  Serial4.begin(9600);
  pinMode(13, OUTPUT);
  foo.asFloat = 11.58f;
  pos_package.position_x.asFloat = 155.236f;  
  pos_package.position_y.asFloat = 20.369f;
  pos_lago_package.position_x.asFloat = 1923.246f;
  pos_lago_package.position_y.asFloat = 15896324.00f;
  pos_lago_package.lago_x.asFloat = 255.31f;
  pos_lago_package.lago_y.asFloat = -456.36f;
  pos_lago_package.lago_ang.asInt = 15;

}

void loop() {
  // Serial.println("Hello world");  
  digitalWrite(LED_BUILTIN, HIGH);
  SendPosition();
  // SendPositionLagori();

}

void SendFunction(uint8_t data[], int size)
{
  for (int i = 0; i < size; i++)
  {
    // Serial.println(data[i], DEC);
    Serial4.write(data[i]);
  }
  
}

void TestCommunication()
{
  static uint32_t last_time = 0;
    if(millis() - last_time >= 10){
    last_time = millis() ; 
    Serial.write("%f");
    Serial.write(foo.asByte[0]);
    Serial.write(foo.asByte[1]);
    Serial.write(foo.asByte[2]);
    Serial.write(foo.asByte[3]);
    Serial.write("\r\n");
    }
}

void SendPosition() // send position of seeker 10 byte
{
  static uint32_t last_time = 0;
    if(millis() - last_time >= 10){
    last_time = millis() ; 

    uint8_t data[13];  // size of position lagori package

    data[0] = 0xFF;  //header
    data[1] = 0xFF;  //header
    data[2] = pos_package.lenght;  //lenght of data = 9
    data[3] = pos_package.data_type;  // data type  = p
    data[4] = pos_package.position_x.asByte[0];  // position x in byte
    data[5] = pos_package.position_x.asByte[1];
    data[6] = pos_package.position_x.asByte[2];
    data[7] = pos_package.position_x.asByte[3];
    data[8] = pos_package.position_y.asByte[0];  // position y in byte 
    data[9] = pos_package.position_y.asByte[1];
    data[10] = pos_package.position_y.asByte[2];
    data[11] = pos_package.position_y.asByte[3];
    data[12] = calChecksum(data, 12);  // checksum 

    SendFunction(data, 13);  //  send data
    
  }
}

void SendPositionLagori()  // send position of lagori 23 byte
{
  static uint32_t last_time = 0;
    if(millis() - last_time >= 10){
    last_time = millis();

    uint8_t data[24];  // size of position lagori package 

    data[0] = 0xFF;  //  header
    data[1] = 0xFF;  //  header
    data[2] = pos_lago_package.lenght;  // lenght of data = 20
    data[3] = pos_lago_package.data_type_1;  //  data type1 = p
    data[4] = pos_lago_package.position_x.asByte[0];  // position x in byte
    data[5] = pos_lago_package.position_x.asByte[1];
    data[6] = pos_lago_package.position_x.asByte[2];
    data[7] = pos_lago_package.position_x.asByte[3];
    data[8] = pos_lago_package.position_y.asByte[0];  //  position y in byte
    data[9] = pos_lago_package.position_y.asByte[1];
    data[10] = pos_lago_package.position_y.asByte[2];
    data[11] = pos_lago_package.position_y.asByte[3];
    data[12] = pos_lago_package.data_type_2;  // data type2 = l
    data[13] = pos_lago_package.lago_x.asByte[0];  //  lagori x in byte
    data[14] = pos_lago_package.lago_x.asByte[1];
    data[15] = pos_lago_package.lago_x.asByte[2];
    data[16] = pos_lago_package.lago_x.asByte[3];
    data[17] = pos_lago_package.lago_y.asByte[0];  // lagori y in byte
    data[18] = pos_lago_package.lago_y.asByte[1];
    data[19] = pos_lago_package.lago_y.asByte[2];
    data[20] = pos_lago_package.lago_y.asByte[3];
    data[20] = pos_lago_package.lago_ang.asByte[0];  // lagori ang in byte
    data[21] = pos_lago_package.lago_ang.asByte[1];
    data[22] = calChecksum(data, 22);  //  checksum
    
    SendFunction(data, 23);  //  send data


    // Serial4.write(0xFF);
    // Serial4.write(0xFF);
    // Serial4.write(pos_lago_package.lenght);
    // Serial4.write(pos_lago_package.data_type_1);
    // Serial4.write(pos_lago_package.position_x.asByte[0]);
    // Serial4.write(pos_lago_package.position_x.asByte[1]);
    // Serial4.write(pos_lago_package.position_x.asByte[2]);
    // Serial4.write(pos_lago_package.position_x.asByte[3]);
    // Serial4.write(pos_lago_package.position_y.asByte[0]);
    // Serial4.write(pos_lago_package.position_y.asByte[1]);
    // Serial4.write(pos_lago_package.position_y.asByte[2]);
    // Serial4.write(pos_lago_package.position_y.asByte[3]);
    // Serial4.write(pos_lago_package.data_type_2);
    // Serial4.write(pos_lago_package.lago_x.asByte[0]);
    // Serial4.write(pos_lago_package.lago_x.asByte[1]);
    // Serial4.write(pos_lago_package.lago_x.asByte[2]);
    // Serial4.write(pos_lago_package.lago_x.asByte[3]);
    // Serial4.write(pos_lago_package.lago_y.asByte[0]);
    // Serial4.write(pos_lago_package.lago_y.asByte[1]);
    // Serial4.write(pos_lago_package.lago_y.asByte[2]);
    // Serial4.write(pos_lago_package.lago_y.asByte[3]);
    // Serial4.write(pos_lago_package.lago_ang.asByte[0]);
    // Serial4.write(pos_lago_package.lago_ang.asByte[1]);
    // Serial4.write(calChecksum(data, 23)); 
    }  
}

void SendShooter()
{
  static uint32_t last_time = 0;
    if(millis() - last_time >= 10){
    last_time = millis() ; 

    uint8_t data[16];

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = shooter_package.lenght;
    data[3] = shooter_package.time.asByte[0];
    data[4] = shooter_package.time.asByte[1];
    data[5] = shooter_package.time.asByte[2];
    data[6] = shooter_package.time.asByte[3];
    data[7] = shooter_package.time_shoot.asByte[0];
    data[8] = shooter_package.time_shoot.asByte[1];
    data[9] = shooter_package.time_shoot.asByte[2];
    data[10] = shooter_package.time_shoot.asByte[3];
    data[11] = shooter_package.theta.asByte[0];
    data[12] = shooter_package.theta.asByte[1];
    data[13] = shooter_package.phi.asByte[0];  
    data[14] = shooter_package.phi.asByte[1];
    data[15] = calChecksum(data, 15);

    SendFunction(data, 16);

    // Serial4.write(0xFF);
    // Serial4.write(0xFF);
    // Serial4.write(shooter_package.lenght);
    // Serial4.write(shooter_package.time.asByte[0]);
    // Serial4.write(shooter_package.time.asByte[1]);
    // Serial4.write(shooter_package.time.asByte[2]);
    // Serial4.write(shooter_package.time.asByte[3]);
    // Serial4.write(shooter_package.time_shoot.asByte[0]);
    // Serial4.write(shooter_package.time_shoot.asByte[1]);
    // Serial4.write(shooter_package.time_shoot.asByte[2]);
    // Serial4.write(shooter_package.time_shoot.asByte[3]);
    // Serial4.write(shooter_package.theta.asByte[0]);
    // Serial4.write(shooter_package.theta.asByte[1]);
    // Serial4.write(shooter_package.phi.asByte[0]);
    // Serial4.write(shooter_package.phi.asByte[1]);
    // Serial4.write(calChecksum(data, 16)); 
    }  
}

void ReciveCheck(){
  ;
}
