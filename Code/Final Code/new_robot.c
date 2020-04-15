/*===========================BUILD ENVIRONMENT=================================
    When building the software environment for this code, make sure that the
    following two libraries are imported:
    
    nRF24L01P (published library)
    mbed      (this library is usually given once a new project is made, but
                sure it is in the program)
    mbed-rtos (published library)
    QEI       (published library)
    
    All published libraries should be found on mbed website
    
    NOTE: To communicate with puTTY and the computer, go to http://www.pemicro.com/opensda/
    to install the kl25Z driver; under the 'Windows USB Drivers' section, click on the URL
    to install the kl25Z driver. This will make the KL25Z show up as a COM port.
==============================================================================*/

// Robot Code
// Version 3.0
// Edited by Ryan Skidd
// Original Authors: Mike Trainor, Koceila Cherfouh
// Date: 03-05-2019
// Updated: 03-03-2020

#include "mbed.h"
#include "nRF24L01P.h"
#include "rtos.h"
#include "Serial.h"
#include "QEI.h"

//Functions
void kicker_actuate(int kick_vel);              //Function to control kick driver
void PID1(void const *argument);                //PID Control for motor 1
void PID2(void const *argument);                //PID Control for motor 2
void PeriodicInterruptISR(void);                //Periodic Timer Interrupt Service Routine for Motor 1
void PeriodicInterrupt2ISR(void);               //Periodic Timer Interrupt Service Routine for Motor 2
void AssignData();                              //function takes data from packet and assigns motor control and kick control

//Processes and Threads
osThreadId EncoderID, Encoder2ID, PiControlId, PiControl2Id; // Thread ID's
osThreadDef(PID1, osPriorityRealtime, 1024); // Declare PiControlThread as a thread, highest priority
osThreadDef(PID2, osPriorityRealtime, 1024); // Declare PiControlThread2 as a thread, highest priority

//Declare ports for communication
Serial pc(USBTX, USBRX); // tx, rx
nRF24L01P my_nrf24l01p(PTD2, PTD3, PTD1, PTD0, PTE0, PTD5);    // mosi, miso, sck, csn, ce, irq
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);

// Data Packets Requirements
#define StartBit        0xFF
#define IDbit           0x01
#define EndBit          0xFF
#define TRANSFER_SIZE   8
char txData[TRANSFER_SIZE], rxData[TRANSFER_SIZE];
int txDataCnt = 0;
int rxDataCnt = 0;


//============================Port Configurations===============================
//Motor Directions
DigitalOut dir_motor1(PTE4); //Direction Motor 1
DigitalOut dir_motor2(PTE5); //Direction Motor 2

//PWM Outputs
PwmOut PWM_motor1(PTE20);   //PWM Motor1
PwmOut PWM_motor2(PTE22);   //PWM Motor2

//Kick driver
DigitalOut actuate(PTE29);  //Kick command
AnalogOut CV(PTE30);        //control voltage for kick

//IR Sensor (Jacob Goudy - Value added)
DigitalIn IR_SENSE(PTC9);  //pin assigned to output of IR sensor (active low)

//Encoder
QEI motor1(PTD4, PTA12, NC, 16, QEI::X4_ENCODING); //QEI for motor 1
QEI motor2(PTA4, PTA5, NC, 16, QEI::X4_ENCODING); //QEI for motor 2
//==============================================================================
//Timer Interrupts
Ticker PeriodicInt; // Declare a timer interrupt: PeriodicInt
Ticker PeriodicInt2; // Declare a timer interrupt: PeriodicInt2

//Declare Global Variables (PID controller)
float des_vel = 0, mes_vel = 0, des_vel2 = 0, mes_vel2 = 0;
float e = 0, e_Prev = 0, e2 = 0, e_Prev2 = 0;
float I_Term = 0, I_Term2 = 0, D_Term = 0, D_Term2 = 0;
float Kp = 0.8, Kd = 0.1, Ki = 1.1; //Current Best is Kp = 5, Kd = 5, Ki = 1.05, Prev Best is Kp = 5, Kd = 5, Ki = 1.1, this is for step input
float u = 0, u2 = 0;
float r = 50, r2 = 50;
float ramp_start = 50, ramp_end = 260, ramp_rate = 8, ramp_rate2 = 8; //Ramp rate for motors could be as low as 0.2 or as high as 410?
float dt = 0.05; //dt best = 0.05?
int pulses_motor1, pulses_motor2;
float PWM_Period = 2000; //in micro-seconds
int pulses_difference = 0, PrevPulses_motor1 = 0;


//Blinking LED's
DigitalOut Communication_LED(PTB0);     //LED for Communication (will blink when communicating with master device)
DigitalOut Power_LED(PTB1);             //LED for Power


int main() {

    Power_LED = 1; //Turn on the LED

    pc.baud(19200);  // Set UART baud rate (must match vision system and master device's baud rate!)

    //Initializing Threads for interrupts
    PiControlId = osThreadCreate(osThread(PID1), NULL); //Create Thread for PID1
    PiControl2Id = osThreadCreate(osThread(PID2), NULL); //Create Thread for PID2
    
    //PWM Period
    PWM_motor1.period_us(PWM_Period); // This sets the PWM period to 2000 us = 500Hz
    PWM_motor2.period_us(PWM_Period); // This sets the PWM period to 2000 us = 500Hz

    //Periodic Interrupts
    PeriodicInt.attach(&PeriodicInterruptISR, dt);      //Periodic timer interrupt every 0.02 seconds
    PeriodicInt2.attach(&PeriodicInterrupt2ISR, dt);    //Periodic timer interrupt every 0.02 seconds

    //nRF24L01 power up settings
    my_nrf24l01p.powerUp();
    my_nrf24l01p.setAirDataRate(NRF24L01P_DATARATE_2_MBPS);

    // Display the (default) setup of the nRF24L01+ chip
    pc.printf( "nRF24L01+ Frequency    : %d MHz\r\n",  my_nrf24l01p.getRfFrequency() );
    pc.printf( "nRF24L01+ Output power : %d dBm\r\n",  my_nrf24l01p.getRfOutputPower() );
    pc.printf( "nRF24L01+ Data Rate    : %d kbps\r\n", my_nrf24l01p.getAirDataRate() );
    pc.printf( "nRF24L01+ TX Address   : 0x%010llX\r\n", my_nrf24l01p.getTxAddress() );
    pc.printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", my_nrf24l01p.getRxAddress() );


    my_nrf24l01p.setTransferSize( TRANSFER_SIZE );
    my_nrf24l01p.setReceiveMode();
    my_nrf24l01p.enable();

    //char input = NULL;      //used for test purposes (kick driver)
    //int kick_vel = 0x00;    //used for test purposes (kick driver)
    
    //Loop Forever
    while (1) {
        
        // If we've received anything in the nRF24L01+...
        if ( my_nrf24l01p.readable() ) {
            //read the data into the receive buffer
            rxDataCnt = my_nrf24l01p.read( NRF24L01P_PIPE_P0, rxData, sizeof( rxData ) );
            AssignData();                           //read data packet and assign commands to robot
            myled2 = !myled2;                       // Toggle LED2
            Communication_LED = !Communication_LED; //Blink the communication LED
        }
        /*
        //for test purpose (uncomment to manually actuate kicker):
        input = NULL;
        while(pc.readable()){
            input = pc.getc(); 
        }
        if(input == 'f'){//actuate kick mechanism
                kick_vel = 0x6D;  input = NULL;  //reset input for next loop
                kicker_actuate(kick_vel);
                pc.printf("just kicked\r\n");
                kick_vel = 0x00; input = ' ';
        }
        else if(input == ' '){kick_vel = 0x00; input = NULL;pc.printf("it worked\r\n");}
        */
        //pc.printf("%i\r\n\r\n",int(IR_SENSE));      //displays current state of IR sensor
    }//end of while loop
}

void AssignData(){              //function takes packet data and assigns to motors and kick driver
    if((rxData[0] == StartBit) &&  (rxData[1] == IDbit) &&  (rxData[7] == EndBit)){
        // Transmitted Data
        des_vel = rxData[2];
        dir_motor1=rxData[3];
        des_vel2 = rxData[4];
        dir_motor2=rxData[5];
        kicker_actuate(rxData[6]);
        //uncomment to view data packets:
        //pc.printf("Packet: vel1:%i|dir1:%i|vel2:%i|dir2:%i|txData[6]:%i\r\n\r\n",rxData[2],rxData[3],rxData[4],rxData[5],rxData[6]);
    }
}

void kicker_actuate(int kick_vel){      //function checks if IR sensor is tripped AND vision system tells robot to kick
    if((kick_vel>0) && (!IR_SENSE)){                            //if control voltage is not zero and the IR sensor trips (sensor is '0' when tripped)
        CV.write_u16((unsigned short) kick_vel << 0x8);         //send the DAC the appropriate control voltage
        wait_us(600000);                                        //wait to charge up the voltage (needs a long wait time because of hardware)
        actuate = 0xFF;                                         //actuate the kick driver for a short impulse
        wait_us(500000);                                        //hold output (needs a long wait time because of hardware)
        actuate = 0x00;                               //put output low to reset the kick driver
    }
}

// ******** PID1 Thread ********
void PID1(void const *argument){
    
    while(true){
        osSignalWait(0x1, osWaitForever); // Go to sleep until signal is received.
        pulses_motor1 =  abs(motor1.getPulses());
        mes_vel = (60*pulses_motor1) / (dt* 700 * 4); //Calculate the measured speed
    
        if(r > ramp_end){
            r = ramp_end;
        }
    
        //Ramping
        if(des_vel <= 30){
            ramp_rate = 8;   
        }
        else{
            ramp_rate = des_vel/20; //Faster Ramp Rate
        }
    
        if(r < des_vel){
            r = r + ramp_rate; //Ramp up
    
            if(r > des_vel){
                r = des_vel; //Limit if it over shoots the desired
            }
        }
        if(r > des_vel){
            r = r - ramp_rate; //Ramp down
    
            if(r < des_vel){
                r = des_vel; //Limit if it under shoots the desired
            }
        }
        
        e = des_vel - abs(mes_vel); //Error
        D_Term = (e - e_Prev) / (dt); //Slope
        u2 = abs(e*Kp + I_Term*Ki + D_Term*Kd);
    
        if(abs(u2) >= PWM_Period){
            u2 = PWM_Period;//Max Duty Cycle
        }
        else{
            if(des_vel != 0 && mes_vel != 0){
                I_Term = I_Term + e; //Previous error + error*time step, scaling factor 10e2
            }
        }
    
        PWM_motor1.pulsewidth_us(abs(u2)); //Set the Pulsewidth output
        e_Prev = e; //Previous Error
        motor1.reset(); //Reset the motor1 encoder    
    }
}


// ******** PID2 Thread ********
void PID2(void const *argument){
    
    while(true){
        osSignalWait(0x1, osWaitForever);               // Go to sleep until signal is received.
        pulses_motor2 = abs(motor2.getPulses());        //Get number of pulses from motor2
        mes_vel2 = (60*pulses_motor2) / (700 * dt * 4); // Motor Speed in RPM at the wheel
    
        //Ramping
        if(des_vel2 <= 30){
            ramp_rate2 = 8; //Special Case
        }
        else{
            ramp_rate2 = des_vel2/20; //Faster Ramp Rate
        }
    
        if(r2 > ramp_end){
            r2 = ramp_end;
        }
    
        if(r2 < des_vel2){
            r2 = r2 + ramp_rate2; //Ramp up
    
            if(r2 > des_vel2){
                r2 = des_vel2; //Limit if it over shoots the desired
            }
        }
        if(r2 > des_vel2){
            r2 = r2 - ramp_rate2; //Ramp down
    
            if(r2 < des_vel2){
                r2 = des_vel2; //Limit if it under shoots the desired
            }
        }
    
        e2 = des_vel2 - abs(mes_vel2); //Error, Ensure Negative Feedback
        D_Term2 = (e2 - e_Prev2)/dt; //Slope
    
        u = abs((e2*Kp + I_Term2*Ki + D_Term2*Kd));
    
        if(abs(u) >= PWM_Period){
            u = PWM_Period;//Max Duty Cycle
        }
        else{
            if(des_vel2 != 0 && mes_vel2 != 0){
                I_Term2 = I_Term2 + e2; //Previous error + error*time step, scaling factor 10e2
            }
        }
    
        PWM_motor2.pulsewidth_us(abs(u));
        e_Prev2 = e2; //Previous Error
        motor2.reset(); //Reset the motor1 encoder 
    }
}


// ******** Periodic Interrupt Handler 1********
void PeriodicInterruptISR(void){
    osSignalSet(PiControlId,0x1); // Activate the signal, PiControl, with each periodic timer interrupt.
}

// ******** Periodic Interrupt Handler 2********
void PeriodicInterrupt2ISR(void){
    osSignalSet(PiControl2Id,0x1); // Activate the signal, PiControl, with each periodic timer interrupt.
}