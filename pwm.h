#ifndef PWM_H
#define PWM_H

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

// Parameter – an den Controller und die Anwendung anpassen:
#define F_PWM         50L               // PWM-Frequenz in Hz (20ms Intervall)
#define PWM_PRESCALER 8                 // Vorteiler für den Timer
#define PWM_STEPS     256               // PWM-Schritte pro Zyklus (1..256)
#define PWM_PORT      PORTD             // Port für PWM
#define PWM_DDR       DDRD              // Datenrichtungsregister für PWM
#define PWM_CHANNELS  3                 // Anzahl der PWM-Kanäle
#define PWM_KEYS      ((1 << 5) | (1 << 6) | (1 << 7))

#define MIN_PULSE_WIDTH      500        // kürzester Puls
#define MAX_PULSE_WIDTH     2500        // längster Puls
#define DEFAULT_PULSE_WIDTH 1500        // Standard-Puls
#define REFRESH_INTERVAL   20000        // Mindest-Refreshzeit

#define SMALL_DELAY _delay_ms(30.0)
#define BIG_DELAY   _delay_ms(50.0)
#define BIGGER_DELAY _delay_ms(200.0)

#ifndef F_CPU
  //#define F_CPU 4000000L  // Falls nicht extern definiert
#endif

// Berechnung des PWM-Takts: Systemtakte pro PWM-Takt
#define T_PWM (F_CPU/(PWM_PRESCALER*F_PWM*PWM_STEPS))

#if ((T_PWM*PWM_PRESCALER) < (111+5))
#error T_PWM zu klein, F_CPU muss vergrößert werden oder F_PWM bzw. PWM_STEPS verkleinert werden
#endif

#if ((T_PWM*PWM_STEPS) > 65535)
#error Periodendauer der PWM zu groß! F_PWM oder PWM_PRESCALER erhöhen.
#endif

// Globale Variablen – extern deklariert
extern uint16_t pwm_timing[PWM_CHANNELS+1];         // Zeitdifferenzen der PWM-Werte
extern uint16_t pwm_timing_tmp[PWM_CHANNELS+1];

extern uint8_t  pwm_mask[PWM_CHANNELS+1];           // Bitmasken für PWM-Ausgänge (zum Löschen)
extern uint8_t  pwm_mask_tmp[PWM_CHANNELS+1];

extern uint8_t  pwm_setting[PWM_CHANNELS];          // PWM-Einstellungen pro Kanal
extern uint8_t  pwm_setting_tmp[PWM_CHANNELS+1];    // Sortierte PWM-Werte

extern volatile uint8_t pwm_cnt_max;              // Zählergrenze (Initialwert 1 ist wichtig!)
extern volatile uint8_t pwm_sync;                   // Flag, dass ein Update möglich ist

// Pointer für wechselseitigen Zugriff (zwischen ISR und Hauptprogramm)
extern uint16_t *isr_ptr_time;
extern uint16_t *main_ptr_time;
extern uint8_t  *isr_ptr_mask;
extern uint8_t  *main_ptr_mask;

// Funktionsprototypen
void init_TCNT1_PWM(void);
void pwm_update(void);
void set_pwm(uint8_t val1, uint8_t val2, uint8_t val3);

#endif // PWM_H
