#include <avr/io.h>
#include <util/delay.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#define F_CPU 8000000UL


SemaphoreHandle_t xSemaphore;

void PWM_Init() {
	// Configura��o do Timer1 para modo PWM de 8 bits
	TCCR1A |= (1 << COM1A1) | (1 << WGM10) | (1 << WGM11);
	TCCR1B |= (1 << CS11);  // Prescaler de 8
}

void ADC_Init() {
	// Configura��o do conversor anal�gico-digital
	ADMUX |= (1 << REFS0);  // Refer�ncia de tens�o AVCC
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // Habilita ADC e define prescaler de 64
}

uint16_t ADC_Read(uint8_t channel) {
	// Seleciona o canal do ADC
	ADMUX &= 0xF0;
	ADMUX |= channel;

	// Inicia a convers�o
	ADCSRA |= (1 << ADSC);

	// Aguarda a convers�o ser conclu�da
	while (ADCSRA & (1 << ADSC));

	// Retorna o valor convertido
	return ADC;
}

void MotorControlTask(void *pvParameters) {
	while (1) {
		// L� o valor do trimpot conectado ao canal ADC0
		uint16_t potValue = ADC_Read(0);

		// Calcula o valor do duty cycle (0-255) com base no valor lido do trimpot
		uint8_t dutyCycle = (potValue / 4);

		// Adquire o sem�foro antes de acessar a vari�vel compartilhada
		if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
			// Configura o valor do duty cycle para controlar a velocidade do motor
			OCR1A = dutyCycle;

			// Libera o sem�foro
			xSemaphoreGive(xSemaphore);
		}

		vTaskDelay(pdMS_TO_TICKS(50));  // Aguarda um curto per�odo de tempo antes de ler novamente
	}
}

void vApplicationIdleHook(void) {
	// A fun��o Idle Task � chamada quando n�o h� tarefas para executar
}

int main(void) {
	xSemaphore = xSemaphoreCreateMutex();

	DDRD |= (1 << PD5);  // Configura o pino PD5 (OC1A) como sa�da

	PWM_Init();  // Inicializa o PWM
	ADC_Init();  // Inicializa o ADC

	xTaskCreate(MotorControlTask, "MotorControl", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	vTaskStartScheduler();

	return 0;
}