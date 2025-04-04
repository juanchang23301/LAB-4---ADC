/************************************************************************
 Universidad del Valle de Guatemala
 IE2023: Programación de Microcontroladores
 Contador Binario 8 bits

 Autor: Juan René Chang Lam
 Descripción: Implementa un contador binario de 8 bits con 2 botones
              utilizando interrupciones. 

 * Conexiones:
 * - PB1: Pulsador para incrementar (pin 9 en Arduino Nano)
 * - PB2: Pulsador para decrementar (pin 10 en Arduino Nano)
 * - PB4, PB5: LEDs para bits menos significativos (pins 12, 13 en Arduino Nano)
 * - PC0-PC5: LEDs para los 6 bits restantes (pins A0-A5 en Arduino Nano)
************************************************************************/
#include <avr/io.h>
#include <util/delay.h>

// Variables globales
volatile uint8_t count = 0;
volatile uint8_t btn_inc_prev = 1;  // Estado previo del botón incremento (1 = no presionado)
volatile uint8_t btn_dec_prev = 1;  // Estado previo del botón decremento (1 = no presionado)

// Función para inicializar los puertos
void init_ports(void) {
    // Configurar PB4 y PB5 como salidas para los bits menos significativos
    DDRB |= (1 << PB4) | (1 << PB5);
    // Mantener PB1 y PB2 como entradas
    DDRB &= ~((1 << PB1) | (1 << PB2));
    // Activar resistencias pull-up en PB1 y PB2
    PORTB |= (1 << PB1) | (1 << PB2);
    
    // Configurar PC0-PC5 como salidas para los bits restantes
    DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);
}

// Función para mostrar el valor del contador en los LEDs
void display_count(uint8_t value) {
	
    // Mostrar los 2 bits menos significativos en PB4 y PB5
    // Primero limpiamos solo esos bits manteniendo el resto
    uint8_t port_b_value = PORTB & ~((1 << PB4) | (1 << PB5));
	
    // Luego asignamos los 2 bits menos significativos
    port_b_value |= ((value & 0x01) << PB4);  // Bit 0 a PB4
    port_b_value |= ((value & 0x02) << (PB5-1));  // Bit 1 a PB5
    PORTB = port_b_value;
    
    // Mostrar los 6 bits restantes en PC0-PC5
    PORTC = (value >> 2) & 0x3F;  // Desplazar 2 bits y mostrar solo 6 bits
}

// Función para leer el estado de un botón con debounce
uint8_t read_button(uint8_t pin_number) {
	
    if (!(PINB & (1 << pin_number))) {   // Lógica invertida: 0 = presionado, 1 = no presionado
        _delay_ms(20);  // Debounce
		
        if (!(PINB & (1 << pin_number))) {
            return 0;  // Botón presionado (activo bajo)
        }
    }
    return 1;  // Botón no presionado
}

int main(void) {
    // Inicializar puertos
    init_ports();
    
    // Mostrar el valor inicial
    display_count(count);
    
    // Bucle principal
    while (1) {
        // Leer estado actual de los botones
        uint8_t btn_inc_current = read_button(PB1);
        uint8_t btn_dec_current = read_button(PB2);
        
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
        
        _delay_ms(10);
    }
    
    return 0;  
}