#include "pwm.h"
#include <avr/interrupt.h>

// Definition der globalen Variablen:
uint16_t pwm_timing[PWM_CHANNELS+1];
uint16_t pwm_timing_tmp[PWM_CHANNELS+1];

uint8_t pwm_mask[PWM_CHANNELS+1];
uint8_t pwm_mask_tmp[PWM_CHANNELS+1];

uint8_t pwm_setting[PWM_CHANNELS];
uint8_t pwm_setting_tmp[PWM_CHANNELS+1];

volatile uint8_t pwm_cnt_max = 1;
volatile uint8_t pwm_sync;

uint16_t *isr_ptr_time  = pwm_timing;
uint16_t *main_ptr_time = pwm_timing_tmp;
uint8_t  *isr_ptr_mask  = pwm_mask;
uint8_t  *main_ptr_mask = pwm_mask_tmp;

// Interner Hilfsfunktionsprototyp – tauscht Zeiger zwischen ISR und Hauptprogramm
static inline void tausche_zeiger(void) {
    uint16_t *tmp_ptr16;
    uint8_t  *tmp_ptr8;

    tmp_ptr16 = isr_ptr_time;
    isr_ptr_time = main_ptr_time;
    main_ptr_time = tmp_ptr16;

    tmp_ptr8 = isr_ptr_mask;
    isr_ptr_mask = main_ptr_mask;
    main_ptr_mask = tmp_ptr8;
}

// PWM-Update-Funktion: Berechnet aus den aktuellen Einstellungen die neuen PWM-Zeit- und Maskenwerte
void pwm_update(void) {
    uint8_t i, j, k;
    uint8_t m1, m2, tmp_mask;
    uint8_t min, tmp_set;

    // Initiale Maske berechnen und PWM-Werte kopieren:
    m1 = 1;
    m2 = 0;
    for(i = 1; i <= PWM_CHANNELS; i++) {
        main_ptr_mask[i] = ~m1;               // Maske zum Löschen der PWM-Ausgänge
        pwm_setting_tmp[i] = pwm_setting[i-1];
        if (pwm_setting_tmp[i] != 0) {
            m2 |= m1;                       // Maske zum Setzen der Ausgänge
        }
        m1 <<= 1;
    }
    main_ptr_mask[0] = m2;

    // Sortieren der PWM-Werte (z. B. mit einem einfachen Auswahlverfahren):
    for(i = 1; i <= PWM_CHANNELS; i++) {
        min = PWM_STEPS - 1;
        k = i;
        for(j = i; j <= PWM_CHANNELS; j++) {
            if (pwm_setting_tmp[j] < min) {
                k = j;
                min = pwm_setting_tmp[j];
            }
        }
        if (k != i) {
            tmp_set = pwm_setting_tmp[k];
            pwm_setting_tmp[k] = pwm_setting_tmp[i];
            pwm_setting_tmp[i] = tmp_set;
            tmp_mask = main_ptr_mask[k];
            main_ptr_mask[k] = main_ptr_mask[i];
            main_ptr_mask[i] = tmp_mask;
        }
    }

    // Vereinigen gleicher PWM-Werte und Entfernen von Nullen:
    k = PWM_CHANNELS;
    i = 1;
    while(k > i) {
        while(((pwm_setting_tmp[i] == pwm_setting_tmp[i+1]) || (pwm_setting_tmp[i] == 0)) && (k > i)) {
            if (pwm_setting_tmp[i] != 0)
                main_ptr_mask[i+1] &= main_ptr_mask[i];
            for(j = i; j < k; j++) {
                pwm_setting_tmp[j] = pwm_setting_tmp[j+1];
                main_ptr_mask[j] = main_ptr_mask[j+1];
            }
            k--;
        }
        i++;
    }
    if (pwm_setting_tmp[i] == 0) k--;

    // Berechnung der Zeitdifferenzen für die PWM-Ausgabe:
    if (k == 0) { // Sonderfall: alle Kanäle auf 0
        main_ptr_time[0] = (uint16_t)T_PWM * PWM_STEPS / 2;
        main_ptr_time[1] = (uint16_t)T_PWM * PWM_STEPS / 2;
        k = 1;
    } else {
        i = k;
        main_ptr_time[i] = (uint16_t)T_PWM * (PWM_STEPS - pwm_setting_tmp[i]);
        tmp_set = pwm_setting_tmp[i];
        i--;
        for(; i > 0; i--) {
            main_ptr_time[i] = (uint16_t)T_PWM * (tmp_set - pwm_setting_tmp[i]);
            tmp_set = pwm_setting_tmp[i];
        }
        main_ptr_time[0] = (uint16_t)T_PWM * tmp_set;
    }

    // Warten, bis der ISR-Sync-Flag gesetzt wurde:
    pwm_sync = 0;
    while(pwm_sync == 0);

    cli();
    tausche_zeiger();
    pwm_cnt_max = k;
    sei();
}

// Setzt die PWM-Werte für alle Kanäle und aktualisiert die PWM-Ausgabe
void set_pwm(uint8_t val1, uint8_t val2, uint8_t val3) {
    pwm_setting[0] = val1;
    pwm_setting[1] = val2;
    pwm_setting[2] = val3;
    pwm_update();
}

// Timer1 Compare A Interrupt – generiert die PWM-Ausgabe:
ISR(TIMER1_COMPA_vect) {
    static uint8_t pwm_cnt = 0; // Zähler für PWM-Kanäle
    uint8_t tmp;

    OCR1A += isr_ptr_time[pwm_cnt];
    tmp = isr_ptr_mask[pwm_cnt];

    if (pwm_cnt == 0) {
        PWM_PORT = tmp; // Setzt die Ausgänge zu Beginn des PWM-Zyklus
        pwm_cnt++;
    } else {
        PWM_PORT &= tmp; // Löscht die entsprechenden Ausgänge
        if (pwm_cnt == pwm_cnt_max) {
            pwm_sync = 1; // Update möglich, Zyklus beendet
            pwm_cnt = 0;
        } else {
            pwm_cnt++;
        }
    }
    // Hinweis: Falls im ursprünglichen Code eine Variable "t" verwendet wurde,
    // entferne sie, falls sie nicht deklariert ist oder definiere sie korrekt.
}

// Initialisiert Timer1 für PWM (z. B. im CTC-Modus mit Prescaler 8)
void init_TCNT1_PWM(void) {
    TCCR1B = 2;              // Prescaler 8
    TIFR |= (1 << OCF1A);      // Löscht das Compare-Flag
    TIMSK |= (1 << OCIE1A);    // Enable Timer1 Compare A Interrupt
}
