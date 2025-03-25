#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "dcf77.h"
#include "pwm.h"

/* Definition der Ausgänge für die Anzeige (z.B. Stunden und Minuten) */
#define H_PORT  PORTC
#define H_DDR   DDRC
#define M_PORT  PORTB
#define M_DDR   DDRB

#define MODUS_IDLE  (1<<0)
#define MODUS_DCF   (1<<1)
#define MODUS_PWM   (1<<2)
#define PWR_DCF     (1<<3)

/* RTC-Variablen (wird von Timer2-ISR aktualisiert) */
volatile uint8_t second_flag = 0;
volatile uint8_t rtc_seconds = 0;
volatile uint8_t rtc_minutes = 0;
volatile uint8_t rtc_hours = 0;

/* Initialisiert Timer2 als RTC (externer 32,768 kHz-Quarz) */
void init_TCNT2_RTC(void)
{
    ACSR |= (1 << ACD);                  // Analogkomparator ausschalten
    ASSR  |= (1 << AS2);                 // Externen 32,768 kHz-Quarz nutzen
    TCCR2 |= (1 << CS22) | (1 << CS20);    // Prescaler 128 → Overflow alle 1 s
    while (ASSR & ((1 << OCR2UB) | (1 << TCR2UB) | (1 << TCN2UB)));
    TIFR |= (1 << TOV2);
    TIMSK |= (1 << TOIE2);               // Timer2 Overflow-Interrupt aktivieren
}

/* Funktion, um den Timer2-Zähler (RTC) zurückzusetzen.
   (wird vom DCF77-Decoder bei erfolgreicher Synchronisation genutzt) */
static void reset_TCNT2(void)
{
    rtc_seconds = 0;
    TCNT2 = 0;
    while (ASSR & ((1 << OCR2UB) | (1 << TCR2UB) | (1 << TCN2UB)));
}

/* Timer2 Overflow-Interrupt: setzt eine Flagge für die RTC */
ISR(TIMER2_OVF_vect)
{
    second_flag = 1;
}

void enable_dcf_timer()
{
    DCF_PORT |= DCF_PWR;
    init_TCNT0_DCF(); // Timer0 für DCF77 neu initialisieren
}

void disable_dcf_timer()
{
    TIMSK &= ~(1 << TOIE0);  // Timer0-Interrupt deaktivieren
    TCCR0 = 0;               // Timer0 stoppen
}

void enable_pwm_timer()
{
    init_TCNT1_PWM(); // Timer1 für PWM neu initialisieren
}

void disable_pwm_timer()
{
    TIMSK &= ~(1 << TOIE1);  // Timer1-Interrupt deaktivieren
    TCCR1A = 0;
    TCCR1B = 0;
}

void goToSleep()
{
    sleep_enable(); // Sleep erlauben
    sei();          // Interrupts aktivieren

    sleep_cpu();    // In den Sleep-Modus wechseln

    // Hierher kommt der Code erst nach einem Interrupt zurück
    sleep_disable(); // Sleep deaktivieren (verhindert erneutes Einschlafen sofort nach Wake-Up)
}

/* Aktualisiert die Anzeige (hier beispielhaft: Stunden auf PORTC, Minuten auf PORTB) */
void update_display(uint8_t hours, uint8_t minutes)
{
    H_PORT = hours;
    M_PORT = minutes;
}

/* --- Zustandsmaschine --- */
typedef enum
{
    MODE_IDLE, // Nur RTC läuft (Binäranzeige oder sonstige Standardanzeige)
    MODE_PWM,  // PWM-Sequenz (z. B. "Aufzeichnung" tagsüber)
    MODE_DCF   // DCF-Synchronisation (bei Systemstart oder zu Sync-Zeiten)
} SystemMode;

volatile SystemMode currentMode = MODE_DCF;
volatile uint8_t ctrl = 0b00001010;//start in MODE_DCF & PWR_DCF

int main(void)
{
    /* Port-Konfiguration für Anzeige */
    H_DDR = 0xFF;
    M_DDR = 0xFF;

    /* Konfiguration des DCF77-Moduls:
       - DCF_PWR-Pin als Ausgang, DCF-Modul einschalten */
    DCF_DDR |= DCF_PWR;
    DCF_PORT |= DCF_PWR;

    /* PWM-Konfiguration (Details in pwm.h/pwm.cpp vorausgesetzt) */
    PWM_DDR |= PWM_KEYS;

    /* Globale Pull-ups deaktivieren */
    SFIOR |= (1 << PUD);

    /* Initialisierungen der Timer */
    init_TCNT0_DCF();    // DCF77-Modul Timer0 zur Decodierung, wird wie TCNT1 nur bei Bedarf aktiviert
//    init_TCNT1_PWM();    // (Aus PWM-Modul)
    init_TCNT2_RTC();    // RTC aktivieren

    /* Initiale PWM-Einstellungen */
    set_pwm(0,0,0);

    /* Schlafmodus aktivieren */
//    set_sleep_mode(SLEEP_MODE_IDLE);//urspgl. SLEEP_MODE_PWR_SAVE

    sei(); // Interrupts global aktivieren

    while (1)
    {
        /* RTC-Verarbeitung: Jede Sekunde wird second_flag in der ISR gesetzt */
        if (second_flag)
        {
            second_flag = 0;
            rtc_seconds++;
            if (rtc_seconds >= 60)
            {
                rtc_seconds = 0;
                rtc_minutes++;
                if (rtc_minutes >= 60)
                {
                    rtc_minutes = 0;
                    rtc_hours++;
                    if (rtc_hours >= 24)
                        rtc_hours = 0;
                }
                update_display(rtc_hours, rtc_minutes);//minute-wise
            }
        }
        switch(currentMode)
        {
        case MODE_IDLE:
            if(!get_dcf_sync())
            {
                /* in DCF77-Modus wechseln */
                ctrl ^= (MODUS_IDLE|MODUS_DCF);
                currentMode = MODE_DCF;
            }
            /*if(rtc_minutes%2) currentMode = MODE_PWM;*/
            /* Optional: Zu definierten Zeiten erneute Synchronisation anstoßen */
            if ((rtc_hours == 5 && rtc_minutes == 45) || (rtc_hours == 18 && rtc_minutes == 48))
            {
                set_dcf_sync(0);
                /* In einem erweiterten Design könnte man hier den Synchronisationsstatus zurücksetzen */
            }
//            if((rtc_hours < 6)||(rtc_hours > 17))
//            {
//
//            }
            break;
        case MODE_DCF:
            if(!(ctrl&PWR_DCF))
            {
                ATOMIC_BLOCK(ATOMIC_FORCEON)
                {
                    DCF_PORT |= DCF_PWR;//turn on pwr for DCF circuitry if it is off
                    enable_dcf_timer();
                    ctrl |= PWR_DCF;
                }
            }

            dcf_process(); // DCF-Daten auswerten

            uint8_t dcf_h, dcf_m;
            dcf_getTime(&dcf_h, &dcf_m);

            if (dcf_h != 0xFF && dcf_m != 0xFF) // Nur übernehmen, wenn valide
            {
                ATOMIC_BLOCK(ATOMIC_FORCEON)
                {
                    /* Aktualisiere die RTC-Uhr (hier beispielhaft direkt) */
                    rtc_hours = dcf_h;
                    rtc_minutes = dcf_m;
                    reset_TCNT2(); // RTC Timer zurücksetzen
                    disable_dcf_timer();
                    update_display(rtc_hours, rtc_minutes);
                    ctrl ^= (MODUS_IDLE|MODUS_DCF|PWR_DCF); //aufräumen: zurücksetzen der Modi, ausschalten PWR_DCF
                    DCF_PORT &= ~DCF_PWR;           //Strom aus
                    //set_dcf_sync(1);//ist schon gesetzt in ISR DCF
                    currentMode = MODE_IDLE;
                }
            }
            break;
        case MODE_PWM:
            if(!ctrl&MODUS_PWM)
            {
                ATOMIC_BLOCK(ATOMIC_FORCEON)
                {
                    enable_pwm_timer();
                    ctrl |= MODUS_PWM;
                }
            }
            /* Beispiel: Eine PWM-Sequenz ausführen */
            for(int it = 0; it < 5; it++)
            {
                for (int i = 0; i < 256; i++)
                {
                    set_pwm(0,i,0);
                    pwm_update();
                    //_delay_ms(20);
                }
            }

            ATOMIC_BLOCK(ATOMIC_FORCEON)
            {
                disable_pwm_timer();
                ctrl &= ~MODUS_PWM;
                currentMode = MODE_IDLE;
            }
            break;
        default:
            update_display(rtc_hours, rtc_minutes);
        }
        /* OPTIONAL Energiesparmodus: Sleep, bis ein Interrupt (RTC oder Timer0) erwacht */
    }
    return 0;
}
