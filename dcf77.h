#ifndef DCF77_H
#define DCF77_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/pgmspace.h>
#include <stdint.h>

/* DCF77 Timing-Konstanten (in 10‑ms-Schritten) */
#define DCF_T0    6
#define DCF_T1   15
#define DCF_T2   17
#define DCF_T3   25
#define DCF_T4   95
#define DCF_T5  120
#define DCF_T6  220

/* DCF77 Hardware-Definitionen */
#define DCF_DDR         DDRD
#define DCF_PORT        PORTD
#define DCF_PWR         (1 << 4)
#define DCF_SIGNAL      (PIND & (1 << 0))

/* DCF77 Ereignistyp */
typedef enum
{
    DCF_0    = '0',  // Null erkannt
    DCF_1    = '1',  // Eins erkannt
    DCF_MARK = 'm',  // Minutenanfang
    DCF_FAIL = 'e',  // Abtastfehler
    DCF_NONE = 'x'   // Kein Ereignis
} DCFEvent;

/* Öffentliche Funktionen des DCF77-Moduls */
// Initialisiert Timer0 für eine 10‑ms-Periode zur DCF77‑Decodierung
void init_TCNT0_DCF(void);

// Sollte in der Hauptschleife periodisch aufgerufen werden, um den DCF77-Datenstrom auszuwerten
void dcf_process(void);

void set_dcf_sync(uint8_t value);
uint8_t get_dcf_sync(void);

// Liefert bei erfolgreicher Synchronisation die dekodierten Stunden und Minuten
void dcf_getTime(uint8_t *hours, uint8_t *minutes);

// Gibt den aktuellen Synchronisations-Status zurück:
// 0 = nicht synchron, 1 = synchronisiert, 2 = (optional) erneute Synchronisation erforderlich
uint8_t get_dcf_sync(void);

#endif // DCF77_H
