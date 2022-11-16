/*
 * File:   proyecto2.c
 * Author: aldoa
 *
 * Created on 8 de noviembre de 2022, 09:16 AM
 */
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 500000      // Frecuencia interna del PIC
#define tmr0_value 241         // tiempo con el que se cargara el timer 0

int ADC;
int serv;
int ADC2;
int ADC3;
int cuenta;
void setup(void);
void setupINTOSC(void);
void setupADC(void);
void setupPWM(void);
void servo(int valor);


void __interrupt() isr (void){
    if(INTCONbits.T0IF){        //Se revisa la bandera del timer0
        cuenta++;               // cada vez se aumenta la variable cuenta
        if(cuenta <= ADC3){
            PORTCbits.RC3 = 1;  // cuando cuenta sea mayor que ADC3 se enciente
                                // la led
        }
        else{
            PORTCbits.RC3 = 0;  // de lo contrario se apaga
        }
        TMR0 = tmr0_value;
        INTCONbits.T0IF = 0;    // se apaga la bandera y se carga de nuevo el
                                // valor al timer
    }
    
    if (PIR1bits.ADIF){         // se revisa la bandera del ADC
        PORTCbits.RC3 = 1;      // si esta encendida se enciende el puerto
        PIR1bits.ADIF = 0;      // se apaga la bandera
    }
}
//******************************************************************************
// CÃ³digo Principal
//******************************************************************************
void main(void) {
    
    setup();
    setupINTOSC();
    setupPWM();
    setupADC();
    while(1){
//******************************************************************************
// ADC
//******************************************************************************
        ADCON0bits.CHS = 0b0000;        // usamos el canal 0
        __delay_us(100);
        ADCON0bits.GO = 1;  // enciendo la bandera
        while(ADCON0bits.GO == 1){
            ;
        }
        ADIF = 0;           // apago la bandera
        ADC = ADRESH;       // cargo el valor a ADC
        servo (ADC);        // llamo a la funcion e ingreso el valor de ADC
            CCPR1L = serv;  // cargo el valor de la conversion a CCPR1L
            __delay_us(100);
            
        ADCON0bits.CHS = 0b0001; // usamos el canal 1
        __delay_us(100);
        ADCON0bits.GO = 1;  // enciendo la bandera
        while(ADCON0bits.GO == 1){
            ;
        }
        ADIF = 0;           // apago la bandera
        ADC2 = ADRESH;      // cargo el valor a ADC2
        servo (ADC2);       // llamo a la funcion e ingreso el valor de ADC2
            CCPR2L = serv;  // cargo el valor de la conversion a CCPR2L
            __delay_us(100);
            
        ADCON0bits.CHS = 0b0010;    // usamos el canal 2
        __delay_us(100);    // cargo el valor a ADC3
        ADCON0bits.GO = 1;  // enciendo la bandera
        while(ADCON0bits.GO == 1){
            ;
        }
        ADIF = 0;           // apago la bandera
        ADC3 = ADRESH;
        
    }
    return;
}

void servo(int valor){
    serv = (unsigned short) (7+( (float)(13)/(255) ) * (valor-0));
    // realizo la conversion para el servomotor
}
//******************************************************************************
// FunciÃ³n para configurar GPIOs
//******************************************************************************
void setup(void){
    ANSELH = 0;
    TRISB = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    cuenta = 0;
    TRISCbits.TRISC3 = 0;       // inicializo todos los puertos
}
//******************************************************************************
// FunciÃ³n para configurar PWM
//******************************************************************************
void setupINTOSC(void){
    OSCCONbits.IRCF = 0b011;       // 500 KHz
    OSCCONbits.SCS = 1;
    INTCONbits.GIE = 1;             
    INTCONbits.TMR0IE = 1;          // activo la interrupcion del timer0
    INTCONbits.T0IF = 0;            // apago la bandera del timer0
    PIE1bits.ADIE = 1;              // activo la interrupcion del ADC
    PIR1bits.ADIF = 0;              // apago la bandera
    
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS = 0b011;
    
    TMR0 = tmr0_value;
}
//******************************************************************************
// FunciÃ³n para configurar ADC
//******************************************************************************
void setupADC(void){
    
    // Paso 1 Seleccionar puerto de entrada
    TRISAbits.TRISA0 = 1;
    ANSELbits.ANS0 = 1; 
    
    TRISAbits.TRISA1 = 1; 
    ANSELbits.ANS1 = 1; 
    
    TRISAbits.TRISA2 = 1;
    ANSELbits.ANS2 = 1; 
    
    // Paso 2 Configurar mÃ³dulo ADC
    
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;       // Fosc/ 8
    
    ADCON1bits.VCFG1 = 0;       // Ref VSS
    ADCON1bits.VCFG0 = 0;       // Ref VDD
    
    ADCON1bits.ADFM = 0;        // Justificado hacia izquierda
        
    //Canal
    ADCON0bits.CHS = 0b0000;        // Canal AN1
    //ADCON0bits.CHS = 0b0010;       // Canal AN2
    //ADCON0bits.CHS = 0b0011;
    
    ADCON0bits.ADON = 1;        // Habilitamos el ADC
    __delay_us(100);
    
}
//******************************************************************************
// FunciÃ³n para configurar PWM
//******************************************************************************
void setupPWM(void){
 
    // **********************************************************
    
    TRISCbits.TRISC2 = 1;           //encendemos el puerto del PWM
    TRISCbits.TRISC1 = 1;           //encendemos el puerto del PWM
    
    PR2 = 155;                      // Periodo de 20ms
    
    CCP1CONbits.P1M = 0b00;         
    
    
    CCP1CONbits.CCP1M = 0b1100;     

    
    CCP2CONbits.CCP2M = 0b1111;     
    
    //Calculos para 1.5ms de ancho de pulso
    CCP1CONbits.DC1B = 0b11;        //CCPxCON<5:4>
    CCPR1L = 11;                    //CCPR1L
    
    
    CCP2CONbits.DC2B1 = 0b1;        //CCPxCON<5:4>
    CCP2CONbits.DC2B0 = 0b1;
    CCPR2L = 11;                    //CCPR2L
    
    //Configutación del TMR2
    TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11;        //Preescaler de 1:16
    T2CONbits.TMR2ON = 1;           //Se habilita TMR2
    
    while(!PIR1bits.TMR2IF){       
        ;
    }
    TRISCbits.TRISC2=0;             //Habilitamos la salida del PWM.
    TRISCbits.TRISC1=0;             //Habilitamos la salida del PWM.
}