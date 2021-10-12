//#fuses NOMCLR,XTPLL,NOWDT,NOPROTECT,NOLVP,NODEBUG,USBDIV,PLL1,CPUDIV1,VREGEN,NOPBADEN
#INCLUDE <18F4550.H>
#DEVICE ADC=10
#include <stdlib.h>   
#include <stdio.h>
#include <string.h>
int contador=0;
long valorADC=0;
#fuses XT // XTPLL: Vamos a usar un cristal de 8.00Mhz.
#fuses CPUDIV1//USADO PARA 8MHZ, NO DIVIDE LA FRECUENCIA
#FUSES NOMCLR // NOMCLR: No vamos ha usar el PIN MCLR, el reset se hará por soft.
#FUSES PUT//ACTIVACACION DE BAJO VOLTAJE
#fuses NOWDT//No vamos a usar el perro guardian.
#fuses NOPROTECT //Memoria no protejida contra lecturas.
#fuses NOVREGEN // Habilita el regulador de 3.3 volts que usa el módulo USB, no lo usaremos por los momentos, se cambiará por NOVREGEN
#BYTE PORTB = 0xF81                      //PORTB cargado en la dirección de memoria del registro PORT B
#BYTE TRISB = 0xF93                      //PORTB cargado en la direeción de memoria del reistro  TRIS B

#USE DELAY (CLOCK= 8000000)//
#USE STANDARD_IO(B)
#USE STANDARD_IO(A)
#USE fast_io(D)
#use fast_io(b)                           //Define el Puerto D para respuesta rápida.
 //velocidad 9600, 8 bits, Sin paridad, TX C6, RX C7
#USE RS232(BAUD=9600,BITS=8,PARITY=N,XMIT=PIN_C6,RCV=PIN_C7,STREAM=PORT1)// rx tx
#define LCD_ENABLE_PIN PIN_D0
#define LCD_RS_PIN PIN_D1
#define LCD_RW_PIN PIN_D2
#define LCD_DATA4 PIN_D3
#define LCD_DATA5 PIN_D4
#define LCD_DATA6 PIN_D5
#define LCD_DATA7 PIN_D6
#include <lcd.c>
int desbordamiento=0;
 int S1=0;
 int S2=0;
 int M1=0;
 int M2=0;
 int H1=0;
 int H2=0;


  
   
 
  
 void configuracion(void)
 {
   //set_tris_d(0x07);                            //Configuramos como salidas los bits RD0, RD1 y RD2
  
 }
 //:::::::::: Función de Interrupción Timer0 :::::::::
 #int_timer0
 void timer_reloj(void)
 {
 if (input(pin_A3)==1){S2=0; output_low(PIN_B1);}

   desbordamiento++;
   if(desbordamiento==100)//100*0.01 10ms 10e-3s "0.1"
   { 
      desbordamiento=0;
      S1++;//hace referencia a 1s
      if(S1==10)// 10 segundos 
      { 
         S1=0;
         S2++;
         if(S2==20)// si aqui cuento 6 es un minuto 
         {output_high(PIN_B1);
  printf("tome agua \n\r");
  delay_ms(1000);//retardo 
            S2=0;
            M1++;
            if(M1==10)
            {//output_high(PIN_B1);delay_ms(1000); output_low(PIN_B1);
               M1=0;
               M2++;
               if(M2==6)
               {
                  M2=0;
                  H1++;
                  if(H1==10)
                  {
                     H2++;
                     H1=0;
                     if(H2==3)
                     {
                        H2=0;
                     }
                  }
               }
            }
         }
      }
   }
   set_timer0(217);// 10 MS CADA VEZ QUE SE EJECUTA
 }
 
 /**/

void main()
{
  //printf("tomeagua\n");
 setup_adc_ports(NO_ANALOGS, VSS_VDD);  //No se utilizará Entradas Analógicas.
   set_tris_b(0x00);                      //Configuramos el puerto B como salida.
   output_b(0x00);    //Ponemos a nivel bajo todos los bits del puerto B.
 
   enable_interrupts(GLOBAL);             //Habilitamos las interrupciones globales.

 lcd_init();// arrancar la lcd
 setup_adc_ports(AN0);// para habilitar puerto analogico 
 setup_adc(adc_clock_internal);// trabaja con el reloj interno 

//para el timer
 setup_timer_0(RTCC_INTERNAL|RTCC_DIV_256|RTCC_8_BIT);    //Preescaler 256 Timer0 
   set_timer0(217);                             //Establecemos el timer0 a 217 para obtener 10ms
  enable_interrupts(GLOBAL);  
  enable_interrupts(int_timer0);               //Habilitamos la interrupción del timer0

   while(TRUE)
   {  
    if (input(pin_A3)==0){
   
    output_high(PIN_B0);delay_ms(1000); output_low(PIN_B0);
     
    }
    
   set_adc_channel(0);
   delay_us(2);
   
   valorADC=read_adc();
  long tempC = read_adc();
  // Calculamos la temperatura con la fórmula
  tempC = (5.0 * tempC * 100.0)/1024.0; 
     
     lcd_gotoxy(1,2);// poner el cursos 
     printf(lcd_putc,"Temp:%Lu      ", tempC);
     
  if   (tempC>40) {output_high(PIN_A1);output_low(PIN_A2);
   lcd_gotoxy(1,2);
     printf(lcd_putc,"Esta caliente");}
   if  (tempC<30) {output_low(PIN_A1);output_high(PIN_A2);}
     
    if (input(pin_A3)==1){
    lcd_gotoxy(1,1);
    printf(lcd_putc," tomandoAgua  ");
    }
   if (input(pin_A3)==0){
   lcd_gotoxy(1,1);
     printf(lcd_putc,"SMART TERMAL  ");
 }
   
     
     
  }
}
