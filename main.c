/************************/
/* Echtzeituhr          **
** DCF-Synchronisierung **
** Ziel: atmega8        **
*************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>
#include "stdint.h"

#define PRESCALER 1024
#define TIMER_TICKS (F_CPU / PRESCALER * 10 / 1000)  // 10ms in Timer-Ticks
#define TIMER_PRELOAD (256 - TIMER_TICKS)

///////////////////////////////////////////////////////////////////////////////
//  Definitionen  /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#define DCF_T0    6             // T0:    1     .. DCF_T1-1 [s÷100]
#define DCF_T1   15             // T1: DCF_T0+1 .. DCF_T2-1 [s÷100]
#define DCF_T2   17             // T2: DCF_T1+1 .. DCF_T3-1 [s÷100]
#define DCF_T3   25             // T3: DCF_T2+1 .. DCF_T4-1 [s÷100]
#define DCF_T4   95             // T4: DCF_T3+1 .. DCF_T5-1 [s÷100]
#define DCF_T5  120             // T5: DCF_T4+1 .. DCF_T6-1 [s÷100]
#define DCF_T6  220             // T6: DCF_T5+1 ..   250    [s÷100]

#define H_PORT  PORTC
#define H_DDR   DDRC
#define M_PORT  PORTB
#define M_DDR   DDRB

#define DCF_DDR         DDRD
#define DCF_PORT        PORTD
#define DCF_PWR         (1<<4)
#define DCF_SIGNAL      (PIND & (1<<0))

///////////////////////////////////////////////////////////////////////////////
//  Typen  ////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
typedef enum                    // Ereignisse der DCF-PWM Abtastung
{
    DCF_0    = '0',             // · Null erkannt
    DCF_1    = '1',             // · Eins erkannt
    DCF_MARK = 'm',             // · Minutenanfang erkannt
    DCF_FAIL = 'e',             // · Abtastfehler
    DCF_NONE = 'x'              // · Kein Ereignis
} DCFEvent;

///////////////////////////////////////////////////////////////////////////////
//  Datenobjekte  /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
volatile DCFEvent dcfEvent;


volatile uint8_t second_flag = 0; // Sekundentakt
volatile uint8_t seconds = 0;
volatile uint8_t minutes = 0;
volatile uint8_t hours = 0;
volatile uint8_t dcf_sync = 0;  // 0 out of sync
                                // 1 synchronized
                                // 2 toggle DCF-circuitry pwr on, set dcf_sync to 0

void init_Timer0_10ms(void)
{
    TCCR0 = (1<<CS02) | (1<<CS00);  // Prescaler = 1024
    TCNT0 = (uint8_t)TIMER_PRELOAD; // Preload setzen
    TIFR |= (1<<TOV0);              // Overflow-Flag löschen
    TIMSK |= (1<<TOIE0);            // Overflow-Interrupt aktivieren
}
/*
void init_Timer1_10ms(void)
{
    // Timer 1: Clear Timer on Compare (CTC), Intervall: 10ms
    TCCR1A = 0;
    TCCR1B = (1<<WGM12) | (1<<CS10);
    OCR1A  = (uint8_t)(int16_t)-(F_CPU / 1024 * 10e-3 + 0.5);                 // 100 Hz
    TIFR |= (1<<OCF1A);
    TIMSK |= (1 << OCIE1A);
}
*/
void init_Timer2_RTC(void)
{
    ACSR |= (1<<ACD);                      //turn off analog comparator
    ASSR  |= (1 << AS2);                   // Externen 32.768 kHz Quarz nutzen
    TCCR2 |= (1 << CS22) | (1 << CS20);    // Vorteiler 128 → 1 Hz Overflow
    while ((ASSR & ((1 << OCR2UB) | (1 << TCR2UB) | (1<< TCN2UB))));
    TIFR |= (1<<TOV2);
    TIMSK |= (1 << TOIE2);                 // Timer Overflow Interrupt aktivieren
}

ISR(TIMER0_OVF_vect)                            // every 10ms
{
    typedef enum
    {
        TI = 0,
        HI = 1,
        LO = 2
    } Input;

    typedef struct
    {
        uint8_t  state;
        char     output;
    } ZetaValue;

    static const ZetaValue zeta[][3] PROGMEM =
    {
        //+--Ti--+ +--Hi--+ +--Lo--+ +--------+
        { { 0,'e'},{ 1,'a'},{ 0,'x'} }, // S0
        { { 2,'x'},{ 1,'x'},{ 0,'e'} }, // S1
        { { 7,'x'},{ 2,'x'},{ 3,'x'} }, // S2
        { { 4,'x'},{ 0,'e'},{ 3,'x'} }, // S3
        { { 5,'x'},{ 0,'e'},{ 4,'x'} }, // S4
        { { 6,'x'},{ 0,'e'},{ 5,'x'} }, // S5
        { {11,'0'},{ 0,'e'},{ 6,'x'} }, // S6
        { { 8,'x'},{ 7,'x'},{ 0,'e'} }, // S7
        { { 0,'e'},{ 8,'x'},{ 9,'x'} }, // S8
        { {10,'x'},{ 0,'e'},{ 9,'x'} }, // S9
        { {11,'1'},{ 0,'e'},{10,'x'} }, // S10
        { {12,'x'},{ 1,'a'},{11,'x'} }, // S11
        { { 0,'e'},{ 1,'m'},{12,'x'} }  // S12
    };
    static uint8_t  state = 0;      // Aktueller Zustand S = { 0,...,12 }
    static uint8_t  tenMs = 0;      // Hundertstelsekundenzähler [s÷100]
    Input           input;          // Eingabe: I = { TI,HI,LO }
    char            output;         // Ausgabe: O = { 'x','a','0','1','m','e' }

    if( tenMs == DCF_T0 ||
            tenMs == DCF_T1 ||
            tenMs == DCF_T2 ||
            tenMs == DCF_T3 ||
            tenMs == DCF_T4 ||
            tenMs == DCF_T5 ||
            tenMs == DCF_T6  ) input = TI;  // NB: Eingabe Ti
    else if(DCF_SIGNAL) input = HI;  //     dominiert Hi
    else                   input = LO;  //     und Lo
    output = pgm_read_byte(&(zeta[state][(uint8_t)input].output));
    state  = pgm_read_byte(&(zeta[state][(uint8_t)input].state));
    if( output == 'm' ) tenMs = 0;
    if     ( output == 'a' ) tenMs = 0;
    else if( output != 'x' ) dcfEvent = (DCFEvent)output;
    tenMs++;
    TCNT0 = (uint8_t)TIMER_PRELOAD;  // Preload-Wert neu setzen
}
/*
ISR(TIMER1_COMPA_vect)
{
    typedef enum
    {
        TI = 0,
        HI = 1,
        LO = 2
    } Input;

    typedef struct
    {
        uint8_t  state;
        char     output;
    } ZetaValue;

    static const ZetaValue zeta[][3] PROGMEM =
    {
        //+--Ti--+ +--Hi--+ +--Lo--+ +--------+
        { { 0,'e'},{ 1,'a'},{ 0,'x'} }, // S0
        { { 2,'x'},{ 1,'x'},{ 0,'e'} }, // S1
        { { 7,'x'},{ 2,'x'},{ 3,'x'} }, // S2
        { { 4,'x'},{ 0,'e'},{ 3,'x'} }, // S3
        { { 5,'x'},{ 0,'e'},{ 4,'x'} }, // S4
        { { 6,'x'},{ 0,'e'},{ 5,'x'} }, // S5
        { {11,'0'},{ 0,'e'},{ 6,'x'} }, // S6
        { { 8,'x'},{ 7,'x'},{ 0,'e'} }, // S7
        { { 0,'e'},{ 8,'x'},{ 9,'x'} }, // S8
        { {10,'x'},{ 0,'e'},{ 9,'x'} }, // S9
        { {11,'1'},{ 0,'e'},{10,'x'} }, // S10
        { {12,'x'},{ 1,'a'},{11,'x'} }, // S11
        { { 0,'e'},{ 1,'m'},{12,'x'} }  // S12
    };
    static uint8_t  state = 0;      // Aktueller Zustand S = { 0,...,12 }
    static uint8_t  tenMs = 0;      // Hundertstelsekundenzähler [s÷100]
    Input           input;          // Eingabe: I = { TI,HI,LO }
    char            output;         // Ausgabe: O = { 'x','a','0','1','m','e' }

    if( tenMs == DCF_T0 ||
            tenMs == DCF_T1 ||
            tenMs == DCF_T2 ||
            tenMs == DCF_T3 ||
            tenMs == DCF_T4 ||
            tenMs == DCF_T5 ||
            tenMs == DCF_T6  ) input = TI;  // NB: Eingabe Ti
    else if(DCF_SIGNAL) input = HI;  //     dominiert Hi
    else                   input = LO;  //     und Lo
    output = pgm_read_byte(&(zeta[state][(uint8_t)input].output));
    state  = pgm_read_byte(&(zeta[state][(uint8_t)input].state));
    if( output == 'm' ) tenMs = 0;
    if     ( output == 'a' ) tenMs = 0;
    else if( output != 'x' ) dcfEvent = (DCFEvent)output;
    tenMs++;
}
*/
ISR(TIMER2_OVF_vect)
{
    /*
    static int16_t time_error;
    // RTC Fehler korrigieren

    if (time_error>999)                 // RTC zu schnell
    {
        TCNT2 = 2;                      // Zähler einen Schritt zurück setzen (2 Takte Verzögerung!)
        time_error -= 1000;
    }
    else if (time_error<-999)           // RTC zu langsam
    {
        TCNT2 = 4;                      // Zähler einen Schritt vor setzen (2 Takte Verzögerung!)
        time_error += 1000;
    }

    time_error += T_ERROR;

    while (ASSR & (1<< TCN2UB));
    */
    second_flag = 1; // Hauptprogramm verarbeitet die Sekunde
}


void reset_cnt2()
{
    TCNT2=0;
    while ((ASSR & ((1 << OCR2UB) | (1 << TCR2UB) | (1<< TCN2UB))));
}

void update_display(void)
{
    H_PORT = hours;    // Stunden auf PORTB ausgeben (0–23)
    M_PORT = minutes;  // Minuten auf PORTC ausgeben (0–59)
}

///////////////////////////////////////////////////////////////////////////////
void ProceedDCFDecoding(void)
{
    static const uint8_t  FAIL    = 0xFF;
    static const uint8_t  BCD[]   = { 1, 2, 4, 8, 10, 20, 40, 80 };
    static uint8_t        parity  = 0;
    static uint8_t        dlsTime = 0;
    static uint8_t        bitNo   = 0;      // 0 .. 58 oder FAIL
    static uint8_t        minute  = 0;      // 0 .. 59
    static uint8_t        hour    = 0;      // 0 .. 23
    static uint8_t        day     = 1;      // 1 .. 31
    static uint8_t        month   = 1;      // 1 .. 12
    static uint8_t        year    = 0;      // (2000+) 0 .. 255
    uint8_t               dcfBit  = 0;
    DCFEvent              event;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        event    = dcfEvent;
        dcfEvent = DCF_NONE;
    }

    if( event == DCF_MARK && bitNo == 59 )
    {

        ATOMIC_BLOCK(ATOMIC_FORCEON)
        {
            seconds  = 0;
            minutes = minute;
            hours = hour;
            reset_cnt2();
            dcf_sync = 1;
            DCF_PORT ^= DCF_PWR;//pwr off DCF module
        }
        // Tag      = day;
        // Monat    = month;
        // Jahr     = 2000 + year;
        // MEZ/MESZ = dlsTime == 0/1
    }
    switch( event )
    {
    case DCF_NONE:
        return;
    case DCF_0:
        dcfBit = 0;
        break;
    case DCF_1:
        dcfBit = 1;
        break;
    default:
        bitNo  = FAIL;
    }
    parity ^= dcfBit;
    switch( bitNo )
    {
    case  0:                                              // -+
    case  1:                                              //  |
    case  2:                                              //  |
    case  3:                                              //  |
    case  4:                                              //  |
    case  5:                                              //  |
    case  6:                                              //  |
    case  7:                                              //  | Bits 0 - 16
    case  8:                                              //  | ignorieren
    case  9:                                              //  |
    case 10:                                              //  |
    case 11:                                              //  |
    case 12:                                              //  |
    case 13:                                              //  |
    case 14:                                              //  |
    case 15:                                              //  |
    case 16:
        break; // -+
    case 17:
        dlsTime = dcfBit;
        break; // MESZ
    case 18:
        if( dcfBit == dlsTime ) bitNo = FAIL;
        break; // MEZ
    case 19:
        break; // Schaltsekunde
    case 20:
        if( !dcfBit ) bitNo = FAIL;
        break; // Startbit (1)
    case 21:
        parity = dcfBit;
        minute = 0;                 // -+
    case 22:                                              //  |
    case 23:                                              //  | Minute
    case 24:                                              //  |
    case 25:                                              //  |
    case 26:                                              //  |
    case 27:
        if( dcfBit ) minute += BCD[bitNo-21];
        break; // -+
    case 28:
        if( parity ) bitNo = FAIL;
        break; // Parität Minute
    case 29:
        parity = dcfBit;
        hour = 0;                   // -+
    case 30:                                              //  |
    case 31:                                              //  | Stunde
    case 32:                                              //  |
    case 33:                                              //  |
    case 34:
        if( dcfBit ) hour += BCD[bitNo-29];
        break; // -+
    case 35:
        if( parity ) bitNo = FAIL;
        break; // Parität Stunde
    case 36:
        parity = dcfBit;
        day  = 0;                   // -+
    case 37:                                              //  | Kalender-
    case 38:                                              //  | tag:
    case 39:                                              //  | 1 .. 31
    case 40:                                              //  |
    case 41:
        if( dcfBit ) day += BCD[bitNo-36];
        break; // -+
    case 42:                                              // -+ Wochentag
    case 43:                                              //  |  ignorieren
    case 44:
        break; // -+
    case 45:
        month = 0;                                   // -+
    case 46:                                              //  | Monat:
    case 47:                                              //  |  1 .. 12
    case 48:                                              //  |
    case 49:
        if( dcfBit ) month += BCD[bitNo-45];
        break; // -+
    case 50:
        year = 0;                                    // -+
    case 51:                                              //  |
    case 52:                                              //  |
    case 53:                                              //  |  Jahr:
    case 54:                                              //  |  0 .. 99
    case 55:                                              //  |
    case 56:                                              //  |
    case 57:
        if( dcfBit ) year += BCD[bitNo-50];
        break; // -+
    case 58:
        if( parity ) bitNo = FAIL;
        break; // Parität Datum
    default:
        bitNo = FAIL;
    }
    bitNo++;
}

int main(void)
{
    H_DDR = 0xFF;  // PORTB als Ausgang für Stunden
    M_DDR = 0xFF;  // PORTC als Ausgang für Minuten
    DCF_DDR |= DCF_PWR;
    DCF_PORT |= DCF_PWR; //pwr supply for DCF77 circuitry
    SFIOR |= (1<<PUD);
    init_Timer0_10ms();
    init_Timer2_RTC();  // Interne Uhr aktivieren
    sei();              // Interrupts aktivieren

    while (1)
    {
        if (second_flag)
        {
            second_flag = 0;

            seconds++;
            if (seconds >= 60)
            {
                seconds = 0;
                minutes++;
                if (minutes >= 60)
                {
                    minutes = 0;
                    hours++;
                    if (hours >= 24)
                    {
                        hours = 0;
                    }
                }
            }
            update_display();
        }

        switch(dcf_sync)
        {
        case 0://sync
            ProceedDCFDecoding();
            break;
        case 1://synced
            if((hours == 5 && minutes == 45) || (hours == 18 && minutes == 48))//**Tägliche Synchronisation um 05:45 und 18:48 Uhr**
            {
                dcf_sync = 2;
            }
            break;
        case 2://sync invalid, resync
            DCF_PORT ^= DCF_PWR;//pwr on DCF module
            dcf_sync = 0;
            break;
        default:
            update_display();
        }
    }
}
