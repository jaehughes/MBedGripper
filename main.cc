#include "mbed.h"

Serial          pc(USBTX, USBRX); // tx, rx
BusOut          leds( LED4, LED3, LED2, LED1 );
PwmOut          mOut1(p21); // motor1 -
PwmOut          mOut2(p22); // motor1 +
AnalogIn        motor_current(p19); // 2.2ohm shunt
 
 bool drive_motor(float dir,float pwr,int timeout,float i_thresh){
    float motor_current_av = 0;
    
    //start motor
    mOut1.write(dir*pwr);
    mOut2.write((1.0f-dir)*pwr);
    //pc.printf("%f,%f\r\n",motor_current_av,i_thresh);
    wait(0.2);                                                              //overcome static friction
    
    //overcome static friction
    int n = 3;
    while(motor_current.read()>0.8*i_thresh && n>0){
        mOut1.write(0);
        mOut2.write(0);
        wait(0.05);
        mOut1.write(dir*pwr);
        mOut2.write((1.0f-dir)*pwr);
        wait(0.2); 
        n--;
    }
    
    //run until timeout or current thresh exceeded
    while(timeout>0 && motor_current_av<i_thresh){    
        motor_current_av = 0.5*(motor_current_av + motor_current.read());   //filter current
        //pc.printf("%f, %f\r\n",motor_current_av,i_thresh);
        
        timeout--;
        wait_ms(5);
    }    
    
    //stop motor
    mOut1.write(0.0f);
    mOut2.write(0.0f);
    return bool(timeout);
}
 
int main() {
    //init motors
    mOut1.write(0.0f);
    mOut2.write(0.0f);
    
    //init
    pc.printf("Pincher Gripper\r\n");
    char incoming_data[2];
    int n=0;
    char ipt;
    int TIMEOUT = 1000;                                                       //~5ms intervals
    float I_THRESH = 0.4;
    int x = 0;
    leds=0;
    
    //loop: read from pc until valid cmd, then execute
    while(1) {
        ipt = pc.getc();
        //pc.putc(ipt); //echo
        //store ipt if not eot
        if(ipt!='\n'){
            if(n>1){
                n=0;
                continue;    
            }
            incoming_data[n]=ipt;
            n++;
        }else{
            n=0;    
            pc.printf("received\r\n");
            switch(incoming_data[0]){
                case 'W': //Reply when ready
                    leds=0;
                    pc.printf("done\r\n");
                    break;
                case 'C': //Close gripper
                    leds=1;
                    x = incoming_data[1]-'0';
                    pc.printf("%c\r\n",drive_motor(0.0f,1.0f,x*TIMEOUT,I_THRESH));
                    pc.printf("done\r\n");
                    break;    
                case 'O': //Open gripper
                    leds=2;
                    x = incoming_data[1]-'0';
                    pc.printf("%c\r\n",drive_motor(1.0f,1.0f,x*TIMEOUT,I_THRESH));
                    drive_motor(0.0f,1.0f,0,I_THRESH);
                    pc.printf("done\r\n");
                    break;    
                case 'G': //Calibrate gripper
                    leds=3;
                    pc.printf("%c\r\n",drive_motor(1.0f,1.0f,10*TIMEOUT,I_THRESH));
                    drive_motor(0.0f,1.0f,0,I_THRESH);
                    pc.printf("done\r\n");
                    break;     
                default:
                    leds=0;
                    pc.printf("INVALID CMD\r\n");
                    pc.printf("done\r\n");
                    break; 
            }
        }
    }
}

