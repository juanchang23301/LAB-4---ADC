/************************************************************************
 Universidad del Valle de Guatemala
 IE2023: Programación de Microcontroladores
 Contador Binario 8 bits

 Autor: Juan René Chang Lam
 Descripción: Implementa un contador binario de 8 bits con 2 botones 
				y un contador hexadecimal con un potenciometro

 * Conexiones:
 * - PB1: Pulsador para incrementar (pin 9 en Arduino Nano)
 * - PB2: Pulsador para decrementar (pin 10 en Arduino Nano)
 * - PB3: LED indicador de comparación (pin 11 en Arduino Nano)
 * - PB4, PB5: LEDs para bits menos significativos (pins 12, 13 en Arduino Nano)
 * - PC0-PC5: LEDs para los 6 bits restantes (pins A0-A5 en Arduino Nano)
************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h> 

const uint8_t TABLE[16] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111, // 9
    0b01110111, // A
    0b01111100, // b
    0b00111001, // C
    0b01011110, // d
    0b01111001, // E
    0b01110001  // F
};

// Variables globales
volatile uint8_t count = 0;
volatile uint8_t btn_inc_prev = 1;  // Estado previo del botón incremento (1 = no presionado)
volatile uint8_t btn_dec_prev = 1;  // Estado previo del botón decremento (1 = no presionado)
volatile uint8_t display_actual = 0;   // Control de multiplexación (0=primer display, 1=segundo display)
volatile uint8_t valor_adc_hex = 0;    // Valor del ADC mapeado a 0-255 (para displays hex)
uint8_t contador = 0;                  // Valor del contador

// Función para inicializar los puertos
void init_ports(void) {
    // Configurar PB3, PB4 y PB5 como salidas
    DDRB |= (1 << PB3) | (1 << PB4) | (1 << PB5);
	
    // Mantener PB1 y PB2 como entradas
    DDRB &= ~((1 << PB1) | (1 << PB2));
    // Activar pull-up en PB1 y PB2
    PORTB |= (1 << PB1) | (1 << PB2);
    
    // Configurar PC0-PC5 como salidas para los bits restantes
    DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);
	
    // Configurar PB0 como salida (cátodo común del segundo display)
    DDRB |= (1 << PB0);
	
    DDRD = 0xFF; // PORTD salida

    // Inicializar salidas en bajo
    PORTB &= ~((1 << PB3) | (1 << PB4) | (1 << PB5));
    PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5));

    PORTD |= (1 << PD7);
    PORTB |= (1 << PB0);
}

// Función para mostrar el valor del contador en los LEDs
void display_count(uint8_t value) {
	
    uint8_t port_b_value = PORTB & ~((1 << PB4) | (1 << PB5));
	
    port_b_value |= ((value & 0x01) << PB4);  // Bit 0 a PB4
    port_b_value |= ((value & 0x02) << (PB5-1));  // Bit 1 a PB5
    PORTB = port_b_value;
    
    PORTC = (value >> 2) & 0x3F;  
}

// Función para comparar el contador binario con el valor hexadecimal del ADC
void compare_counters(uint8_t binary_count, uint8_t hex_value) {
    if (hex_value > binary_count) {
        // Encender LED en PB3 si el valor hex es mayor
        PORTB |= (1 << PB3);
    } else {
        // Apagar LED en PB3 si el valor hex no es mayor
        PORTB &= ~(1 << PB3);
    }
}

// Función para leer el estado de un botón con debounce
uint8_t read_button(uint8_t pin_number) {
	
    if (!(PINB & (1 << pin_number))) {   
        _delay_ms(5); 
		
        if (!(PINB & (1 << pin_number))) {
            return 0;  // Botón presionado 
        }
    }
    return 1;  // Botón no presionado
}

void init_timer0(void) {
    TCCR0A = 0;
    // Preescaler de 64
    TCCR0B = (1 << CS01) | (1 << CS00);
    
    TCNT0 = 131;
    
    // Habilitar interrupción por desbordamiento
    TIMSK0 = (1 << TOIE0);
}


ISR(TIMER0_OVF_vect) {
    TCNT0 = 131;
    // Displays inician apagados
    PORTD |= (1 << PD7); 
    PORTB |= (1 << PB0); 
    
    if (display_actual == 0) {
        // Mostrar dígito menos significativo (0-F)
        PORTD = (PORTD & 0x80) | (TABLE[valor_adc_hex & 0x0F] & 0x7F);
        
        // Activar el primer display 
        PORTB &= ~(1 << PB0);
        display_actual = 1;
    } else {
        PORTD = (PORTD & 0x80) | (TABLE[(valor_adc_hex >> 4) & 0x0F] & 0x7F);
        
        // Activar el segundo display 
        PORTD &= ~(1 << PD7);
        display_actual = 0;
    }
}

//Función para inicializar el ADC
void init_adc(void) {
    // Seleccionar AVCC como referencia de voltaje
    ADMUX = (1 << REFS0);
    
    // Seleccionar el canal para A6 
    ADMUX |= (1 << MUX1) | (1 << MUX2);
    
    // Habilitar ADC, establecer prescaler a 128 
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

//Función para leer el valor del pin ADC
uint16_t leer_adc(void) {
    const uint8_t NUM_MUESTRAS = 16;
    uint32_t suma_actual = 0;
    
    // Tomar varias muestras y sumarlas
    for (uint8_t i = 0; i < NUM_MUESTRAS; i++) {
        // Iniciar conversión
        ADCSRA |= (1 << ADSC);
        
        // Esperar hasta que la conversión termine
        while (ADCSRA & (1 << ADSC));
        
        // Sumar la lectura
        suma_actual += ADC;
        
        // Pequeño retraso entre muestras
        _delay_us(150);
    }
    
    // Devolver solo el promedio de las muestras actuales
    return (uint16_t)(suma_actual / NUM_MUESTRAS);
}

int main(void) {
    
    uint8_t incremento_presionado = 0;   // Flag para evitar incrementos múltiples
    uint8_t decremento_presionado = 0;   // Flag para evitar decrementos múltiples
    uint16_t valor_adc_actual = 0;       // Valor actual del ADC (0-1023)
    
    // Inicializar puertos
    init_ports();
    init_adc();
    init_timer0();
    
    // Habilitar interrupciones globales
    sei();
    
    // Mostrar el valor inicial
    display_count(count);
    
    while (1) {
        // Leer el valor del ADC
        valor_adc_actual = leer_adc();
        
        // Leer estado actual de los botones
        uint8_t btn_inc_current = read_button(PB1);
        uint8_t btn_dec_current = read_button(PB2);
        uint16_t valor_compensado;
        
        // Detectar flanco descendente del botón de incremento (1->0)
        if (btn_inc_prev == 1 && btn_inc_current == 0) {
            count++;
            display_count(count);
        }
        
        // Detectar flanco descendente del botón de decremento (1->0)
        if (btn_dec_prev == 1 && btn_dec_current == 0) {
            count--;
            display_count(count);
        }
        
        // Actualizar estados previos
        btn_inc_prev = btn_inc_current;
        btn_dec_prev = btn_dec_current;
        
        if (valor_adc_actual < 30) {
            valor_compensado = 0;
            valor_adc_hex = 0;
        }
        else if (valor_adc_actual > 1013) {
            valor_compensado = 1023;
            valor_adc_hex = 1023;
        }
        else {
            valor_compensado = ((uint32_t)(valor_adc_actual - 20) * 1023UL) / (1013UL - 10UL);
        }
        
        // Convertir a rango 0-255 para representación hexadecimal
        uint8_t nuevo_valor_hex = (uint8_t)((valor_compensado * 255UL) / 1023UL);
        
        if (abs((int16_t)nuevo_valor_hex - (int16_t)valor_adc_hex) > 5) {
            if (nuevo_valor_hex > valor_adc_hex) {
                valor_adc_hex += 1;
            } else {
                valor_adc_hex -= 1;
            }
        }
        
        // Comparar el contador binario con el valor hexadecimal
        compare_counters(count, valor_adc_hex);
        
        _delay_ms(150);
    }
    
    return 0;  
}