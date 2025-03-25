#include "dcf77.h"
#include <util/delay.h>  // _delay_ms falls benötigt

/* Globales DCF77-Ereignis – wird von der ISR gesetzt */
volatile DCFEvent dcfEvent = DCF_NONE;

/* Interne Statusvariablen für den DCF77-Decoder */
static volatile uint8_t dcf_sync = 0;    // 0 = out of sync, 1 = synchronisiert
static volatile uint8_t dcf_hours = 0;     // dekodierte Stunde
static volatile uint8_t dcf_minutes = 0;   // dekodierte Minute

/* Für Timer0: Berechnung des Preload-Wertes (10 ms Periode, Prescaler 1024)
   (Vorausgesetzt F_CPU ist extern definiert) */
#define PRESCALER_0 1024
#define TIMER_TICKS ((F_CPU / PRESCALER_0 * 10) / 1000)
#define TIMER0_PRELOAD (256 - TIMER_TICKS)

/* Initialisiert Timer0 zur DCF77-Decodierung (10‑ms-Periode) */
void init_TCNT0_DCF(void)
{
    TCCR0 = (1 << CS02) | (1 << CS00);   // Prescaler 1024
    TCNT0 = (uint8_t)TIMER0_PRELOAD;     // Preload setzen
    TIFR |= (1 << TOV0);                 // Overflow-Flag löschen
    TIMSK |= (1 << TOIE0);               // Timer0 Overflow-Interrupt aktivieren
}

/* ISR für Timer0-Overflow – führt die DCF77-Decodierung aus */
ISR(TIMER0_OVF_vect)
{
    typedef enum
    {
        TI = 0,
        HI = 1,
        LO = 2
    } Input;

    typedef struct
    {
        uint8_t state;
        char output;
    } ZetaValue;

    static const ZetaValue zeta[][3] PROGMEM =
    {
        { { 0, 'e' }, { 1, 'a' }, { 0, 'x' } }, // S0
        { { 2, 'x' }, { 1, 'x' }, { 0, 'e' } }, // S1
        { { 7, 'x' }, { 2, 'x' }, { 3, 'x' } }, // S2
        { { 4, 'x' }, { 0, 'e' }, { 3, 'x' } }, // S3
        { { 5, 'x' }, { 0, 'e' }, { 4, 'x' } }, // S4
        { { 6, 'x' }, { 0, 'e' }, { 5, 'x' } }, // S5
        { {11, '0' }, { 0, 'e' }, { 6, 'x' } }, // S6
        { { 8, 'x' }, { 7, 'x' }, { 0, 'e' } }, // S7
        { { 0, 'e' }, { 8, 'x' }, { 9, 'x' } }, // S8
        { {10, 'x' }, { 0, 'e' }, { 9, 'x' } }, // S9
        { {11, '1' }, { 0, 'e' }, {10, 'x' } }, // S10
        { {12, 'x' }, { 1, 'a' }, {11, 'x' } }, // S11
        { { 0, 'e' }, { 1, 'm' }, {12, 'x' } }  // S12
    };

    static uint8_t state = 0;   // aktueller Zustand S = 0 .. 12
    static uint8_t tenMs = 0;   // Zähler in 10‑ms-Schritten (entspricht s/100)
    Input input;
    char output;

    if ( tenMs == DCF_T0 || tenMs == DCF_T1 || tenMs == DCF_T2 ||
            tenMs == DCF_T3 || tenMs == DCF_T4 || tenMs == DCF_T5 ||
            tenMs == DCF_T6 )
    {
        input = TI;
    }
    else if (DCF_SIGNAL)
    {
        input = HI;
    }
    else
    {
        input = LO;
    }

    output = pgm_read_byte(&(zeta[state][(uint8_t)input].output));
    state  = pgm_read_byte(&(zeta[state][(uint8_t)input].state));

    if (output == 'm')
        tenMs = 0;
    if (output == 'a')
        tenMs = 0;
    else if (output != 'x')
        dcfEvent = (DCFEvent)output;

    tenMs++;
    TCNT0 = (uint8_t)TIMER0_PRELOAD;
}

/* Führt die DCF77-Decodierung aus – soll in der Hauptschleife aufgerufen werden */
void dcf_process(void)
{
    static const uint8_t FAIL = 0xFF;
    static const uint8_t BCD[] = { 1, 2, 4, 8, 10, 20, 40, 80 };
    static uint8_t parity = 0;
    static uint8_t dlsTime = 0;
    static uint8_t bitNo = 0;   // Bitnummer (0 bis 58 oder FAIL)
    static uint8_t minute = 0;
    static uint8_t hour = 0;
    static uint8_t day = 1;
    static uint8_t month = 1;
    static uint8_t year = 0;

    uint8_t dcfBit = 0;
    DCFEvent event;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        event = dcfEvent;
        dcfEvent = DCF_NONE;
    }

    if (event == DCF_MARK && bitNo == 59)
    {
        ATOMIC_BLOCK(ATOMIC_FORCEON)
        {
            /* Bei einem vollständigen Minuten-Rahmen werden
               die dekodierten Stunde und Minute übernommen und
               der RTC-Zähler (Timer2) zurückgesetzt */
            dcf_hours = hour;
            dcf_minutes = minute;
            set_dcf_sync(1);
        }
        // (Weitere Daten wie Tag, Monat, Jahr könnten hier verarbeitet werden)
    }

    switch (event)
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
        bitNo = FAIL;
    }
    parity ^= dcfBit;
    switch (bitNo)
    {
    case  0:
    case  1:
    case  2:
    case  3:
    case  4:
    case  5:
    case  6:
    case  7:
    case  8:
    case  9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
        break;
    case 17:
        dlsTime = dcfBit;
        break;
    case 18:
        if (dcfBit == dlsTime) bitNo = FAIL;
        break;
    case 19:
        break;
    case 20:
        if (!dcfBit) bitNo = FAIL;
        break;
    case 21:
        parity = dcfBit;
        minute = 0;
    // fall-through
    case 22:
    case 23:
    case 24:
    case 25:
    case 26:
    case 27:
        if (dcfBit) minute += BCD[bitNo - 21];
        break;
    case 28:
        if (parity) bitNo = FAIL;
        break;
    case 29:
        parity = dcfBit;
        hour = 0;
    // fall-through
    case 30:
    case 31:
    case 32:
    case 33:
    case 34:
        if (dcfBit) hour += BCD[bitNo - 29];
        break;
    case 35:
        if (parity) bitNo = FAIL;
        break;
    case 36:
        parity = dcfBit;
        day = 0;
    // fall-through
    case 37:
    case 38:
    case 39:
    case 40:
    case 41:
        if (dcfBit) day += BCD[bitNo - 36];
        break;
    case 42:
    case 43:
    case 44:
        break;
    case 45:
        month = 0;
    // fall-through
    case 46:
    case 47:
    case 48:
    case 49:
        if (dcfBit) month += BCD[bitNo - 45];
        break;
    case 50:
        year = 0;
    // fall-through
    case 51:
    case 52:
    case 53:
    case 54:
    case 55:
    case 56:
    case 57:
        if (dcfBit) year += BCD[bitNo - 50];
        break;
    case 58:
        if (parity) bitNo = FAIL;
        break;
    default:
        bitNo = FAIL;
    }
    bitNo++;
}

/* Liefert die dekodierte Uhrzeit zurück (nur gültig, wenn synchronisiert) */
void dcf_getTime(uint8_t *hours, uint8_t *minutes)
{
    if (get_dcf_sync() == 0) // Noch nicht synchronisiert
    {
        if (hours)   *hours = 0xFF;  // Fehlerwert
        if (minutes) *minutes = 0xFF;
    }
    else
    {
        if (hours)   *hours = dcf_hours;
        if (minutes) *minutes = dcf_minutes;
    }
}

void set_dcf_sync(uint8_t value)
{
    dcf_sync = value;
}

/* Liefert den aktuellen Synchronisationsstatus */
uint8_t get_dcf_sync(void)
{
    return dcf_sync;
}
