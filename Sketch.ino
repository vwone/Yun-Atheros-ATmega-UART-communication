String Cmd;

//#define DEBUG

//unsigned long Time_Current;

typedef struct STRUCT_MOT{
  long int StepCount;
  long int StepCount_Cpy;
  long int StepCount_Cpy_Old;
  long int StepSign;
  long int StepTimeNew;
  long int StepTimeNew_Cpy;
  long int StepTimeOld;
  long int StepTimeOld_Cpy;
  long int StepDelay;
  long int StepDelay_Cpy;
  long int StepSpeed;
  long int MotSpeed;
  long int MotSpeed_Cpy;
  long int MotPos;
  long int MotPos_Cpy;
  long int MotTmp;
  unsigned int Idle;
}STRUCT_MOT;

STRUCT_MOT Mot;
long int Err = 0x55555555; 

long int Count = 0;


/*******************************************************************/
/* SETUP                                                           */
/*******************************************************************/
void setup() 
{

  Mot.MotPos    = 1;
  Mot.MotSpeed  = 1;
  Mot.MotTmp    = 1;

  /*---------------------------------------------------------------*/
  /* µC <=> Monitor serial communication.                          */
  /*---------------------------------------------------------------*/
#ifdef DEBUG
  Serial.begin( 115200 ); // Serial speed 115200 bauds
  while(!Serial);
  Serial.println( "Let's go!" );
#endif //DEBUG

  /*---------------------------------------------------------------*/
  /* µC <=> µP communication.                                      */
  /*---------------------------------------------------------------*/
  // Open uC <-> uP communication
  // Configuration : 
  // default config 8N1
  // data     : 8 bit
  // parity   : None
  // Stop bit : 1

//  SERIAL_PORT_HARDWARE.begin(250000); // open serial connection to Linux
//  SERIAL_PORT_HARDWARE.begin(230400); // open serial connection to Linux
//  SERIAL_PORT_HARDWARE.begin(115200); // open serial connection to Linux
//  Serial1.begin(115200, SERIAL_8N1);

  // lede ko 
  // linino withuC_uP_Com_2019-05-17 @ 250kbps 
  // fails success
  //   1     20     (network)
  Serial1.begin(250000, SERIAL_8N1);

}//setup()

/*******************************************************************/
/* Int32 => Serial 1 (µP).                                         */
/*******************************************************************/
void Serial_1_Send_Int32(long int *Val)
{
//    SERIAL_PORT_HARDWARE.write(*    (((unsigned char *)Val)+0));
//    SERIAL_PORT_HARDWARE.write(*    (((unsigned char *)Val)+1));
//    SERIAL_PORT_HARDWARE.write(*    (((unsigned char *)Val)+2));
//    SERIAL_PORT_HARDWARE.write(*    (((unsigned char *)Val)+3));
    Serial1.write(*    (((unsigned char *)Val)+0));
    Serial1.write(*    (((unsigned char *)Val)+1));
    Serial1.write(*    (((unsigned char *)Val)+2));
    Serial1.write(*    (((unsigned char *)Val)+3));

}//Serial_1_Send_Int32()

/*******************************************************************/
/* Managment of the communication µC / µP on Serial 1 (µP).        */
/*******************************************************************/
void Com_mC_mP_Mngt(void)
{
//    Cmd = SERIAL_PORT_HARDWARE.readStringUntil('\n');
    Cmd = Serial1.readStringUntil('\n');

#ifdef DEBUG
    Serial.println( Cmd );
#endif //DEBUG

  if (Cmd == "RST") 
  {
//        ResetFlag = true;
    Count = 0;
    Mot.MotPos = 0;
    Mot.MotSpeed = 0;
    Mot.MotTmp = 0;
    //Acquitement du reset
    Serial_1_Send_Int32(&Mot.MotPos);
    Serial_1_Send_Int32(&Mot.MotSpeed);
    Serial_1_Send_Int32(&Mot.MotTmp);

  }
  else if (Cmd == "RD") 
  {
    Serial_1_Send_Int32(&Mot.MotPos);
    Serial_1_Send_Int32(&Mot.MotSpeed);
    Serial_1_Send_Int32(&Mot.MotTmp);

#ifdef DEBUG
    Serial.print( "Pos: ");
    Serial.print( Mot.MotPos);
    Serial.print( ", Spd: ");
    Serial.print( Mot.MotSpeed);
    Serial.print( ", Tmp: ");
    Serial.println( Mot.MotTmp);
#endif //DEBUG
  } 
  else 
  {
#ifdef DEBUG
    Serial.println( "Bad Cmd !" );
#endif //DEBUG
    Serial_1_Send_Int32(&Err);
    Serial_1_Send_Int32(&Err);
    Serial_1_Send_Int32(&Err);     
  }//else if

}//Com_mC_mP_Mngt


/*******************************************************************/
/* LOOP                                                            */
/*******************************************************************/
void loop() {

//  Time_Current = micros();
  
  // Read from Serial port 1 (µP)
  if( Serial1.available() )
  { 
    Mot.MotPos = Count;
    Mot.MotSpeed = Count+1;
    Mot.MotTmp = Count+2;

    Com_mC_mP_Mngt();
    
    Count++;
  }

}//loop()



//End
