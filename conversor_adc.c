#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"

#include "ssd1306/ssd1306.h" 

// definições dos pinos
#define LED_RED_PIN 13 //led vermelho
#define LED_BLUE_PIN 12 //led azul
#define LED_GREEN_PIN 11//led verde
#define JOYSTICK_BUTTON_PIN 22 // Botão do joystick (toggle LED verde e borda)
#define BUTTON_A_PIN 5 // Botão A (ativa/desativa PWM nos LEDs)

//config i2c
#define I2C_PORT i2c1
#define I2C_SDA 14 //config pino de dados i2c
#define I2C_SCL 15 // config pino de clock i2c
#define endereco 0x3C //endereço do display

#define PWM_WRAP 4096 //definindo wrap globalmente

// resolução do display
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

// tamanho do quadrado
#define SQUARE_SIZE 8

// variaveis globais
volatile bool pwm_enabled = false;// armazena o estado de todos os leds conforme enunciado
volatile bool led_green_state = false; // variavel que armazena o estado do led verde

int border_style = 0; //variavel que armazena o estilo das bordas que aparecem no display ssd1306

ssd1306_t ssd; //display struct

// Função de callback para as IRQs dos botões
void gpio_callback(uint gpio, uint32_t events) {
    
    uint32_t current_ms = to_ms_since_boot(get_absolute_time());

    // variaveis necessarias para controlar o debouce
    static uint32_t last_time = 0; 
    uint32_t now = time_us_32();

    if((now - last_time) > 200000) { // debounce basico de 200ms
        last_time = now;

    
        if (gpio == JOYSTICK_BUTTON_PIN) {
            led_green_state = !led_green_state; //alterando o estado do led verde
            border_style++; //mudando o estilo da borda
            
        } 
        else if (gpio == BUTTON_A_PIN) { // caso botao A for pressionado todos os leds devem ser apagados ou desligados
            pwm_enabled = !pwm_enabled; // controla o estado de todos os leds
        }
    }
    
}

// função que inicia pwm num pino gpio
uint pwm_init_gpio(uint gpio, uint wrap){
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_enabled(slice_num, true);
    return slice_num;
}

// setup inicial do display ssd1306
void display_init(){
    // I2C Initialisation. Using it at 400Khz.
  i2c_init(I2C_PORT, 400 * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
  gpio_pull_up(I2C_SDA); // Pull up the data line
  gpio_pull_up(I2C_SCL); // Pull up the clock line
   // Inicializa a estrutura do display
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd); // Configura o display
  ssd1306_send_data(&ssd); // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);
}

void gpio_setup(){
    // inicialização dos leds com pwm
    uint red_led_slice = pwm_init_gpio(LED_RED_PIN, PWM_WRAP);
    uint blue_led_slice = pwm_init_gpio(LED_BLUE_PIN, PWM_WRAP);
    uint green_led_slice = pwm_init_gpio(LED_GREEN_PIN, PWM_WRAP);
    
    // seta botões com pull-up interno e entradas
    gpio_init(JOYSTICK_BUTTON_PIN);
    gpio_set_dir(JOYSTICK_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(JOYSTICK_BUTTON_PIN);
    
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);
}

void update_display(ssd1306_t *ssd, uint16_t vx_value, uint16_t vy_value, uint8_t *border_style) {
    int i;
    // Inverte os eixos para uma melhor estética no display
    uint8_t square_x = (vy_value * (DISPLAY_WIDTH - SQUARE_SIZE)) / 4095;
    uint8_t square_y = (vx_value * (DISPLAY_HEIGHT - SQUARE_SIZE)) / 4095;

    // Limpa o display preenchendo toda a RAM do display com 0 (apagando todos os pixels)
    ssd1306_fill(ssd, false);

    // escolhe qual borda ira ser mostrada no dispaly 1306
    switch (*border_style) { // analisando o conteudo de border_style com ponteiro
        case 0:
            // desenho borda simples
            ssd1306_rect(ssd, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, true, false);
            break;
        case 1:
            // desenho de borda dupla
            ssd1306_rect(ssd, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, true, false);
            ssd1306_rect(ssd, 2, 2, DISPLAY_WIDTH - 4, DISPLAY_HEIGHT - 4, true, false);
            break;
        case 2:
            // desenho borda pontilhada
            for (i = 0; i < DISPLAY_WIDTH; i += 4) {
                ssd1306_pixel(ssd, i, 0, true);
                ssd1306_pixel(ssd, i, DISPLAY_HEIGHT - 1, true);
            }
            // for (i = 0; i < DISPLAY_HEIGHT; i += 4) {
            //     ssd1306_pixel(ssd, 0, i, true);
            //     ssd1306_pixel(ssd, DISPLAY_WIDTH - 1, i, true);
            // }
            break;
        default:
            // faz com que a bordas se repitam
            if (*border_style > 2) {
                *border_style = 0;
            } else if (*border_style < 0) {
                *border_style = 2;
            }
            break;
    }

    // Desenha o quadrado de 8x8 pixels na posição calculada
    ssd1306_rect(ssd, square_y, square_x, SQUARE_SIZE, SQUARE_SIZE, true, true);

    // Envia os dados para o display
    ssd1306_send_data(ssd);
}

void pwm_leds_config(uint16_t vx_value,uint16_t vy_value){
        uint green_led_brightness = led_green_state ? 4095 : 0;
        pwm_set_gpio_level(LED_GREEN_PIN, green_led_brightness);
        
        if(pwm_enabled){ // caso botao A for pressionado todos os leds vao acender ou apagar
            pwm_set_gpio_level(LED_RED_PIN, 4095);
            pwm_set_gpio_level(LED_BLUE_PIN, 4095);
            pwm_set_gpio_level(LED_GREEN_PIN, 4095);
        } else { // 
            // leds recebem valores de intensidade (controlado por pwm) do joystick conforme sua posição
            pwm_set_gpio_level(LED_RED_PIN, vy_value);
            pwm_set_gpio_level(LED_BLUE_PIN, vx_value);
            // limite de wrap para o led ficar apagado conforme enunciado do projeto (wrap de 2048 - leds apagados)
            // coloquei wrap num intervalo pois meu joystick nao esta calibrado ficando entre 2000 e 2300 (com folga de 100 nos limites para o projeto ficar apresentavel)
            if(vy_value > 2000 && vy_value < 2300) pwm_set_gpio_level(LED_RED_PIN, 0);
            if(vx_value > 2000 && vx_value < 2300) pwm_set_gpio_level(LED_BLUE_PIN, 0);  
        }
}


int main() {
    // Inicialização básica
    stdio_init_all();

    // inicializa adc
    adc_init();
    adc_gpio_init(26);  // configura adc nos pinos do joystick (eixo x)
    adc_gpio_init(27);  // eixo y

    // inicialização dos pinos gpio
    gpio_setup();
    
    // configuração das interrupções dos botoes 
    gpio_set_irq_enabled_with_callback(JOYSTICK_BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(BUTTON_A_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    
    //inicialização do display
    display_init();
    
    while (true) {
        adc_select_input(0); // seleciona o canal 0, gpio 26 do adc
        uint16_t vx_value = adc_read(); // le o valor do joystick no eixo x (0 a 4095)
        adc_select_input(1); // seleciona o canal 1, gpio 27 do adc
        uint16_t vy_value = adc_read(); // le o valor do joystick no eixo y (0 a 4095)
        //função que configura os leds com pwm conforme enunciado
        pwm_leds_config(vx_value, vy_value);
        // função que atualiza o display (quadrado flutuante e bordas)
        update_display(&ssd, vx_value, vy_value, &border_style);

        sleep_ms(10);
    }
    
    return 0;
}
