/*
 * File:   main.c
 * Author: Jose Castañeda
 *
 * Created on February 10, 2020, 1:40 AM
 */

// DSPIC33FJ128GP802 Configuration Bit Settings

// 'C' source line config statements

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
#pragma config IESO = ON             // Internal External Switch Over Mode (Start-up device with user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function (OSC2 pin has digital I/O function)
#pragma config IOL1WAY = ON            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are enabled)


// FPOR
#pragma config FPWRT = PWR1             // POR Timer Value (Disabled)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD2               // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = OFF              // JTAG Port Enable (JTAG is Enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


//Librerias utilizadas
#include <xc.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <libpic30.h>
#include <p33FJ128GP802.h>


#define AIN2   LATBbits.LATB9    
#define AIN1   LATBbits.LATB8  
#define STBY   LATBbits.LATB7  
#define PWMOUT LATBbits.LATB6


unsigned int pwm_counter = 0;   
unsigned int RPM = 0;      //Diferencia de tiempo entre las dos senales del encoder  




/***************************Interrupciones*********************/
//Timer1
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
    /* Interrupt Service Routine code goes here */
    IFS0bits.T1IF = 0; // Clear Timer 2 interrupt flag
}

//Timer 2
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
    /* Interrupt Service Routine code goes here */
    IFS0bits.T2IF = 0; // Clear Timer 2 interrupt flag
}

// Input Capture Interrupt
void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void)
{
    unsigned int t1,t2,DTEncoder;
    t1 = IC1BUF;              //Tiempo senal 1
    t2 = IC1BUF;              //Tiempo senal 2
    IFS0bits.IC1IF = 1;       //reiniciamos la captura del IC1
 
    //calculamos la diferencia de tiempo
    if(t2 > t1)               
        DTEncoder = t2-t1;
    else
        DTEncoder = (PR2 - t1) + t2;
    
    //Calculamos RPM actual de la llanta
    RPM = 1000 / DTEncoder * 12;
    
}





void init_osc(void);
void init_pins(void);
void init_timer(uint8_t timer, uint16_t count_time);
void init_pwm(void);
void init_InputCapture(void);




int main(void) {
    init_osc();
    init_pins();
    init_timer(2, 3999);
    init_pwm();
    init_InputCapture();

    while (1) {
        OC1RS = 2000;       //Duty cycle 50% 
    }
    return 0;
}

void
init_pins(void) {
    //Set up pines
    TRISA = 0x0;                //Salidas
    TRISBbits.TRISB0 = 0;       //IC1 - comparacion PWMA encoder
    TRISBbits.TRISB9 = 0;       //AIN2
    TRISBbits.TRISB8 = 0;       //AIN1
    TRISBbits.TRISB7 = 0;       //STBY
    TRISBbits.TRISB6 = 0;       //PWM motor
    
    //estados iniciales para los pines del driver
    AIN1 = 1;
    AIN2 = 0;
    STBY = 1;
    AD1PCFGL = 0b111111111;     //Pines analogicos como digitales
}

void
init_osc(void) {
    // Oscilador: 40Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*40/(2*2)=80Mhz para 8MHZ input del primario
    PLLFBD = 38; // M=40
    CLKDIVbits.PLLPOST = 0; // N1=2
    CLKDIVbits.PLLPRE = 0; // N2=2
    OSCTUN = 0; // Si se usa el FRC

    // Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Clock Switch --> PLL
    __builtin_write_OSCCONH(0x01); // NOSC = 001 para pasar a FRC + PLL
    __builtin_write_OSCCONL(0x01); // Switch
    while (OSCCONbits.COSC != 0b001); // Esperar hasta que este en PLL
    while (OSCCONbits.LOCK != 1) { };  // Esperar hasta que PLL esté activo
}

void 
init_pwm(void) {
    // Initialize Output Compare Module
    OC1CONbits.OCM = 0b000; // Desactivar modulo OC
    OC1R = 0; // Ciclo de trabajo de primer pulso
    OC1RS = 0; // Ciclo de trabajo de segundo pulso
    OC1CONbits.OCTSEL = 0; // Seleccionar Timer 2
    OC1R = 2000; // Se carga el valor del registro determina el duty cycle
    OC1CONbits.OCM = 0b110; // PWM + Activar
    RPOR3bits.RP6R = 0b10010; //RB6 conectado a salida PWM - modo output compare
}

void
init_timer(uint8_t timer, uint16_t count_time) {
    switch (timer) {

        //TIMER 1
        case 1:
            /*
               Segmento de codigo para configurar el registro TxCON
             */
            T1CONbits.TON = 0;      //apagamos timer
            T1CONbits.TCS = 0;      //Reloj interno (fosc/2)
            T1CONbits.TSIDL = 0;    
            T1CONbits.TCKPS = 0b00; //1:1 prescaler
            T1CONbits.TGATE = 0;    
            T1CONbits.TSYNC = 0;    //No sincronizar
            /*
               Segmento de codigo para configurar la duracion del timer
             */
            TMR1 = 0;
            PR1 = count_time;
            //Interrupcion cada: (500+1)*(1/FOSC)s

            /*
               Segmento de codigo para configurar interrupciones
               INTCON1 = INTCON2 = 0
             */
            IFS0bits.T1IF = 0;  //Limpiamos bandera de interrupcion
            IPC0bits.T1IP = 7;  //nivel de prioridad
            IEC0bits.T1IE = 1;  //Habilitamos interrupcion  Timer1
            
            //Se enciende el timerx
            T1CONbits.TON = 1;

        //TIMER 2
        case 2:
            /*
               Segmento de codigo para configurar el registro TxCON
             */
            T2CONbits.TON = 0;
            T2CONbits.TCS = 0;
            T2CONbits.TSIDL = 0;
            T2CONbits.TCKPS = 0b00; //1:1 prescaler
            T2CONbits.TGATE = 0;
            /*
               Segmento de codigo para configurar la duracion del timer
             */
            TMR2 = 0;
            PR2 = count_time;
            //Interrupcion cada: (500+1)*(1/FOSC)
            
            /*
               Segmento de codigo para configurar interrupciones
               INTCON1 = INTCON2 = 0
             */
            IFS0bits.T2IF = 0;
            IPC1bits.T2IP = 7; //0 deshabilitado, 7 maxima prioridad
            IEC0bits.T2IE = 1;
            //Se enciende el timerx
            T2CONbits.TON = 1;
    }
}


void init_InputCapture(void){
    // Initialize the Input Capture Module 1 - pin RP5 - Interrupcion cada flanco de subida
    IC1CONbits.ICM = 0b00;      // Disable Input Capture 1 module
    RPINR7bits.IC1R = 0b00101;  //Input capture tied to RP5 -IC1
    IC1CONbits.ICTMR = 1;       // Select Timer2 as the IC1 Time base
    IC1CONbits.ICI = 0b01;      // Interrupt on every second capture event
    IC1CONbits.ICM = 0b011;     // Generate capture event on every Rising edge
    
    //Habilitamos interrupcion de captura y timer 2
    IPC0bits.IC1IP = 0b100; // Setup IC1 interrupt priority level  nivel superior al OC
    IFS0bits.IC1IF = 0;     // Clear IC1 Interrupt Status Flag
    IEC0bits.IC1IE = 1;     // Enable IC1 interrupt
}




