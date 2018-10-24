#include <Wire.h>
#include <Servo.h>

////////////////////////////Gyro var/////////////////////////////
float elapsedTime, time, timePrev;        
int gyro_fehler=0;                         
int16_t Gyr_eingang_X, Gyr_eingang_Y, Gyr_eingang_Z;     
float Gyro_winkel_X, Gyro_winkel_Y;         
float Gyro_eingang_fehler_X, Gyro_eingang_fehler_Y; 
float Ges_winkel_X;
float Ges_winkel_Y;

///////////////////////////Acc var//////////////////////////////

int acc_fehler=0;                         
float rad_zu_grad = 180/3.14159265  ;      
float Acc_eingang_X, Acc_eingang_Y, Acc_eingang_Z;    
float Acc_winkel_Y, Acc_winkel_X;          
float Acc_winkel_fehler_Y, Acc_winkel_fehler_X;

//////////////////////////reiceiver////////////////////////////

unsigned long Zaehler1, Zaehler2, Zaehler3, Zaehler4, aktuell_Zaehler;

byte letzter_CH1_status, letzter_CH2_status, letzter_CH3_status, letzter_CH4_status;

int input_drehen;   //Pin D12
int input_X;        //Pin D9
int input_Y;        //Pin D8
int input_Z;        //Pin D10
float rec_grad_x;
float rec_grad_y;

/////////////////////////servo////////////////////////////////////
Servo servoy;
Servo servox;
Servo esc1;
Servo esc2;

int servox_pos;
int servoy_pos;

//////////////////////////////PID X/////////////////////////////////

float winkel_fehler_X, winkel_fehler_alt_X;
float PID_P_X, PID_I_X, PID_D_X;
float PID_X;

 
////////////PID werte X/////////////////////////////////////////////
double P_Wert_X = 1;
double I_Wert_X = 1;
double D_Wert_X = 1;

//////////////////////////////PID Y/////////////////////////////////

float winkel_fehler_Y, winkel_fehler_alt_Y;
float PID_P_Y, PID_I_Y, PID_D_Y;
float PID_Y;

 
////////////PID werte Y/////////////////////////////////////////////
double P_Wert_Y = 1;
double I_Wert_Y = 1;
double D_Wert_Y = 1;









void setup() {


servox.attach(5);
servoy.attach(4);


esc1.attach(6);
esc2.attach(7);

esc1.writeMicroseconds(1000);
esc2.writeMicroseconds(1000);

 
 Wire.begin();                          
  
  Wire.beginTransmission(0x68);                       
  Wire.write(0x6B);                      
  Wire.write(0x00);
  Wire.endTransmission(true);             
  
  Wire.beginTransmission(0x68);           
  Wire.write(0x1B);                     
  Wire.write(0x10);      
  Wire.write(0x1C);                      
  Wire.write(0x10);   
  Wire.endTransmission(true);
  
  PCICR |= (1 << PCIE0);                                                     
  PCMSK0 |= (1 << PCINT0);   
  PCMSK0 |= (1 << PCINT1);                                               
  PCMSK0 |= (1 << PCINT2);                                                 
  PCMSK0 |= (1 << PCINT4);       
 
  
  Serial.begin(250000);                    
  time = millis();   


 if(acc_fehler==0)
  {
    for(int a=0; a<200; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);                       
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true); 
      
      Acc_eingang_X=(Wire.read()<<8|Wire.read())/4096.0 ; 
      Acc_eingang_Y=(Wire.read()<<8|Wire.read())/4096.0 ;
      Acc_eingang_Z=(Wire.read()<<8|Wire.read())/4096.0 ;

      
    ////////////////////////////X////////////////////
      Acc_winkel_fehler_Y = Acc_winkel_fehler_X + ((atan((Acc_eingang_Y)/sqrt(pow((Acc_eingang_X),2) + pow((Acc_eingang_Z),2)))*rad_zu_grad));
   /////////////////////////////Y////////////////////
      Acc_winkel_fehler_Y = Acc_winkel_fehler_Y + ((atan(-1*(Acc_winkel_X)/sqrt(pow((Acc_eingang_Y),2) + pow((Acc_eingang_Z),2)))*rad_zu_grad)); 
      
      if(a==199)
      {
        Acc_winkel_fehler_X = Acc_winkel_fehler_Y/200;
        Acc_winkel_fehler_Y = Acc_winkel_fehler_Y/200;
        acc_fehler=1;
      }
    }
  }




if(gyro_fehler==0)
  {
    for(int i=0; i<200; i++)
    {
      Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
      Wire.write(0x43);                        //First adress of the Gyro data
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,4,true);           //We ask for just 4 registers 
         
      Gyr_eingang_X=Wire.read()<<8|Wire.read();     //Once again we shif and sum
      Gyr_eingang_Y=Wire.read()<<8|Wire.read();
   
      /*---X---*/
      Gyro_eingang_fehler_X = Gyro_eingang_fehler_X + (Gyr_eingang_X/32.8); 
      /*---Y---*/
      Gyro_eingang_fehler_Y = Gyro_eingang_fehler_Y + (Gyr_eingang_Y/32.8);
      if(i==199)
      {
        Gyro_eingang_fehler_X = Gyro_eingang_fehler_X/200;
        Gyro_eingang_fehler_Y = Gyro_eingang_fehler_Y/200;
        gyro_fehler=1;
      }
    }
  }  

 esc1.writeMicroseconds(1000);
 esc2.writeMicroseconds(1000);
 delay(4000);



}

void loop() {

 
  timePrev = time;                        
  time = millis();   
  elapsedTime = (time - timePrev) / 1000;

  Gyroscope();
  accelerometer();
  
  Serial.println(Gyro_eingang_fehler_X); //Gyro_winekl_Y = Drehen
                                //Gyro_winekl_X = Normal X
                                //Acc_winkel_X = NOrmal Y
                                //Acc_winekl_Y = Acc_winkel_Y


}



void Gyroscope() {
   Wire.beginTransmission(0x68);           
    Wire.write(0x43);                        
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true);           
        
    Gyr_eingang_X=Wire.read()<<8|Wire.read();    
    Gyr_eingang_Y=Wire.read()<<8|Wire.read();


    Gyr_eingang_X = (Gyr_eingang_X/32.8) - Gyro_eingang_fehler_X; 
    Gyr_eingang_Y = (Gyr_eingang_Y/32.8) - Gyro_eingang_fehler_Y;

    Gyro_winkel_X = Gyro_winkel_X  + Gyr_eingang_X*elapsedTime;
    Gyro_winkel_Y = Gyro_winkel_Y  + Gyr_eingang_Y*elapsedTime;

    Ges_winkel_X = 0.98 *(Ges_winkel_X + Gyro_winkel_X) + 0.02*Acc_winkel_X;

    Ges_winkel_Y = 0.98 *(Ges_winkel_Y + Gyro_winkel_Y) + 0.02*Acc_winkel_Y;
}





void accelerometer(){

Wire.beginTransmission(0x68);      
  Wire.write(0x3B);               
  Wire.endTransmission(false);      
  Wire.requestFrom(0x68,6,true);    


     Acc_eingang_X=(Wire.read()<<8|Wire.read())/4096.0 ; 
     Acc_eingang_Y=(Wire.read()<<8|Wire.read())/4096.0 ;
     Acc_eingang_Z=(Wire.read()<<8|Wire.read())/4096.0 ;

//////////////////////acc X/////////////////////////////////
 Acc_winkel_X = (atan((Acc_eingang_Y)/sqrt(pow((Acc_eingang_X),2) + pow((Acc_eingang_Z),2)))*rad_zu_grad) - Acc_winkel_fehler_Y;
/////////////////////acc Y//////////////////////////////////
 Acc_winkel_Y = (atan(-1*(Acc_eingang_X)/sqrt(pow((Acc_eingang_Y),2) + pow((Acc_eingang_Z),2)))*rad_zu_grad) - Acc_winkel_fehler_Y;  
}








//Kanal 1 (input_Y)
ISR(PCINT0_vect){

  aktuell_Zaehler = micros();
  if(PINB & B00000001) {
    if(letzter_CH1_status == 0) {
      letzter_CH1_status = 1;
      Zaehler1 = aktuell_Zaehler; 
    }
  }
  else if(letzter_CH1_status == 1){
    letzter_CH1_status = 0;
    input_Y = aktuell_Zaehler - Zaehler1;
  }




//Kanal 2 (input_X)
  if(PINB & B00000010) {
    if(letzter_CH2_status == 0) {
      letzter_CH2_status = 1;
      Zaehler2 = aktuell_Zaehler; 
    }
  }
  else if(letzter_CH2_status == 1){
    letzter_CH2_status = 0;
    input_X = aktuell_Zaehler - Zaehler2;
  }



//Kanal 3 (input_Z)
  if(PINB & B00000100) {
    if(letzter_CH3_status == 0) {
      letzter_CH3_status = 1;
      Zaehler3 = aktuell_Zaehler; 
    }
  }
  else if(letzter_CH3_status == 1){
    letzter_CH3_status = 0;
    input_Z = aktuell_Zaehler - Zaehler3;
  }



  //Kanal 4 (input_drehen)
  if(PINB & B00010000) {
    if(letzter_CH4_status == 0) {
      letzter_CH4_status = 1;
      Zaehler4 = aktuell_Zaehler; 
    }
  }
  else if(letzter_CH4_status == 1){
    letzter_CH4_status = 0;
    input_drehen = aktuell_Zaehler - Zaehler4;


  float rec_grad_x = komprimieren_x(input_X, 1172, 1772, -45, 45);
  float rec_grad_y = komprimieren_y(input_Y, 1232, 1770, -45, 45);
  Serial.println(rec_grad_x);
  
}

}
 float komprimieren_x(float eingang, int altMin, int altMax, int neuMin, int neuMax)
  {

    float Differenz_alt = altMax - altMin;
    float Differenz_neu = neuMax - neuMin;
    float Multiplikator = Differenz_neu / Differenz_alt;
    rec_grad_x = input_X * Multiplikator;
    rec_grad_x = rec_grad_x - 220.20;
    return rec_grad_x;
  }




  float komprimieren_y(float eingang, int altMin, int altMax, int neuMin, int neuMax)

  {

    float Differenz_alt = altMax - altMin;
    float Differenz_neu = neuMax - neuMin;
    float Multiplikator = Differenz_neu / Differenz_alt;
    rec_grad_y = input_X * Multiplikator;
    rec_grad_y = rec_grad_y - 220.20;
    return rec_grad_y;
  }










