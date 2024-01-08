#include <avr/io.h>
#include <util/delay.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#define F_CPU 8000000UL  // Frequência do clock em Hz

SemaphoreHandle_t xSemaphore;
volatile uint16_t adcValue = 0;

void PWM_Init() {
	// Configuração do Timer1 para modo PWM de 8 bits
	//TCCR1A |= (1 << COM1A1) | (1 << WGM10) | (1 << WGM11);
	//TCCR1B |= (1 << CS11);  // Prescaler de 8

	// Configuração do Timer/Counter 1 para modo PWM de 8 bits
	TCCR1A |= (1 << COM1A1) | (1 << WGM10) | (1 << WGM11);
	TCCR1B |= (1 << CS11) | (1 << CS10);  // Prescaler de 64
	// Para outros valores do prescaler, ajuste CS11 e CS10 conforme necessário
}

void ADC_Init() {
	// Configuração do conversor analógico-digital
	ADMUX |= (1 << REFS0);  // Referência de tensão AVCC
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // Habilita ADC e define prescaler de 64
}

uint16_t ADC_Read(uint8_t channel) {
	// Seleciona o canal do ADC
	ADMUX &= 0xF0;
	ADMUX |= channel;

	// Inicia a conversão
	ADCSRA |= (1 << ADSC);

	// Aguarda a conversão ser concluída
	while (ADCSRA & (1 << ADSC));

	// Retorna o valor convertido
	return ADC;
}

void MotorControlTask(void *pvParameters) {
	while (1) {
		// Lê o valor do canal ADC0
		adcValue = ADC_Read(0);

		// Adquire o semáforo antes de acessar a variável compartilhada
		if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
			// Faça algo com o valor lido do ADC, por exemplo, ajustar o ciclo de trabalho do PWM
			// Neste exemplo, calcula-se um valor de PWM proporcional ao valor lido do ADC
			//uint16_t pwmValue = adcValue / 4;  // Assume que adcValue está na faixa de 0 a 1023
			uint16_t pwmValue = adcValue/1.1;
			OCR1A = pwmValue;

			// Libera o semáforo
			xSemaphoreGive(xSemaphore);
		}

		vTaskDelay(pdMS_TO_TICKS(50));  // Aguarda um curto período de tempo antes de ler novamente
	}
}

int main(void) {
	xSemaphore = xSemaphoreCreateMutex();

	// Configura o pino PD5 (OC1A) como saída
	DDRD |= (1 << PD5);

	PWM_Init();  // Inicializa o PWM
	ADC_Init();  // Inicializa o ADC

	xTaskCreate(MotorControlTask, "MotorControl", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	// Inicializa o FreeRTOS scheduler
	vTaskStartScheduler();

	return 0;
}