/**
 * @file gigantes_mdf.ino
 * @brief Firmware do robo "Gigante de MDF" (ATmega328P, 16 MHz).
 *
 * Controle tipo tanque (cada lado com frente/re), motores por PWM
 * (Timer1 -> D9, Timer2 -> D3), laser piscando 1 s ligado / 1 s
 * desligado pelo Timer0, LDR no ADC0 com deteccao de acerto,
 * 3 LEDs de vida e botao de reset por INT0.
 *
 * Mapa de pinos:
 *  - D2  (PD2): botao reset, apertado = HIGH (pull-down externo 10k), INT0
 *  - D3  (PD3): EN_B, PWM do motor B (OC2B)
 *  - D4  (PD4): LED 1a vida
 *  - D5  (PD5): controle - esquerda para tras
 *  - D6  (PD6): controle - esquerda para frente
 *  - D7  (PD7): LED 2a vida
 *  - D8  (PB0): LED 3a vida
 *  - D9  (PB1): EN_A, PWM do motor A (OC1A)
 *  - D10 (PB2): controle - direita para frente
 *  - D11 (PB3): controle - direita para tras
 *  - D12 (PB4): laser (via BC547)
 *  - A0  (PC0): LDR (mais luz = leitura menor)
 *  - A1/A2 (PC1/PC2): IN1/IN2 do motor A
 *  - A3/A4 (PC3/PC4): IN3/IN4 do motor B
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#if F_CPU != 16000000UL
#error "Este codigo foi configurado para clock de 16 MHz."
#endif

// ==================== CONFIGURACOES GERAIS ====================

#define VELOCIDADE_NORMAL       255
#define VELOCIDADE_GIRO         190

/** 1 = motor A e o lado esquerdo do robo; 0 = motor A e o lado direito. */
#define MOTOR_A_ESQUERDO          1

/** Coloque 1 se o motor girar ao contrario do esperado. */
#define INVERTER_MOTOR_A          0
#define INVERTER_MOTOR_B          0

#define TEMPO_BLOQUEIO_MS       5000UL
#define TEMPO_GIRO_180_MS        700UL

#define TEMPO_LASER_MS          1000U

#define DEBOUNCE_CONTROLE_MS      25UL
#define DEBOUNCE_RESET_MS         30UL

/** 0 = com mais luz a leitura do ADC cai (LDR no GND, resistor no VCC). */
#define LUZ_AUMENTA_ADC            0
#define LIMIAR_ACERTO            100
#define HISTERESE_LDR             30

#define AMOSTRAS_CALIBRACAO      200U
#define AMOSTRAS_CONFIRMACAO       3U
#define AMOSTRAS_REARME           10U

// Bits do controle (controle tipo tanque)
#define BOTAO_ESQ_FRENTE           0   // D6
#define BOTAO_ESQ_RE               1   // D5
#define BOTAO_DIR_FRENTE           2   // D10
#define BOTAO_DIR_RE               3   // D11

// ==================== ESTADOS DO ROBO ====================

/** Estados da maquina de estados principal. */
typedef enum {
    CALIBRANDO,
    ATIVO,
    BLOQUEADO,
    GIRANDO,
    ELIMINADO
} EstadoRobo;

EstadoRobo estado = CALIBRANDO;

uint8_t vidas = 3;
uint32_t inicioEstado = 0;

// ==================== VARIAVEIS DAS INTERRUPCOES ====================

volatile uint32_t contadorMs = 0;

volatile uint16_t valorAdc = 0;
volatile uint8_t novaLeituraAdc = 0;

volatile uint8_t pedidoReset = 0;

volatile uint8_t laserHabilitado = 0;
volatile uint16_t contadorLaser = 0;

// ==================== VARIAVEIS DO LDR ====================

uint32_t somaCalibracao = 0;
uint16_t quantidadeCalibracao = 0;
uint16_t referenciaLdr = 0;

uint8_t receptorArmado = 0;
uint8_t contadorAcerto = 0;
uint8_t contadorRearme = 0;

// ==================== VARIAVEIS DO CONTROLE ====================

uint8_t controleAnterior = 0;
uint8_t controleEstavel = 0;

uint32_t mudancaControle = 0;

uint8_t resetPendente = 0;
uint32_t inicioReset = 0;

// ==================== PROTOTIPOS ====================

void configurarPinos(void);
void configurarTimers(void);
void configurarAdc(void);
void configurarInterrupcaoReset(void);

uint32_t tempoAtual(void);
uint8_t obterAdc(uint16_t *valor);

void atualizarVidas(void);
void habilitarLaser(uint8_t habilitar);

void motorA(int16_t comando);
void motorB(int16_t comando);
void pararMotores(void);

uint8_t lerControle(void);
void atualizarControle(uint32_t agora);
int16_t comandoLado(uint8_t frente, uint8_t re);
void movimentarPeloControle(void);

void iniciarPartida(uint32_t agora);
void verificarReset(uint32_t agora);

void registrarAcerto(uint32_t agora);
void processarLdr(uint16_t leitura, uint32_t agora);
void atualizarRobo(uint32_t agora);

// ==================== CONFIGURACAO DOS PINOS ====================

/** @brief Configura direcao e pull-ups de todos os pinos usados. */
void configurarPinos(void) {
    // Saidas comecam em nivel baixo
    PORTB &= ~(_BV(PB0) | _BV(PB1) | _BV(PB4));
    PORTC &= ~(_BV(PC1) | _BV(PC2) |
               _BV(PC3) | _BV(PC4));
    PORTD &= ~(_BV(PD3) | _BV(PD4) | _BV(PD7));

    DDRB |= _BV(PB0) | _BV(PB1) | _BV(PB4);
    DDRC |= _BV(PC1) | _BV(PC2) |
            _BV(PC3) | _BV(PC4);
    DDRD |= _BV(PD3) | _BV(PD4) | _BV(PD7);

    // Entradas
    DDRB &= ~(_BV(PB2) | _BV(PB3));
    DDRD &= ~(_BV(PD2) | _BV(PD5) | _BV(PD6));

    // Controle: botao liga o pino ao GND -> pull-up interno
    PORTB |= _BV(PB2) | _BV(PB3);
    PORTD |= _BV(PD5) | _BV(PD6);

    // Reset: pull-down externo de 10k -> SEM pull-up interno
    PORTD &= ~_BV(PD2);

    // LDR: entrada analogica sem pull-up
    DDRC &= ~_BV(PC0);
    PORTC &= ~_BV(PC0);
}

// ==================== CONFIGURACAO DOS TIMERS ====================

/** @brief Timer0 = base de 1 ms; Timer1 e Timer2 = PWM dos motores. */
void configurarTimers(void) {
    TIMSK0 = 0;
    TIMSK1 = 0;
    TIMSK2 = 0;

    // Timer0: CTC, prescaler 64, interrupcao a cada 1 ms.
    TCCR0A = _BV(WGM01);
    TCCR0B = 0;
    TCNT0 = 0;
    OCR0A = 249;

    TIFR0 = _BV(OCF0A) | _BV(OCF0B) | _BV(TOV0);
    TIMSK0 = _BV(OCIE0A);

    TCCR0B = _BV(CS01) | _BV(CS00);

    // Timer1: Fast PWM de 8 bits, prescaler 8, aproximadamente 7,8 kHz.
    TCCR1A = _BV(WGM10);
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 0;
    OCR1B = 0;

    TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);

    TCCR1B = _BV(WGM12) | _BV(CS11);

    // Timer2: Fast PWM de 8 bits, prescaler 8, aproximadamente 7,8 kHz.
    TCCR2A = _BV(WGM20) | _BV(WGM21);
    TCCR2B = 0;
    TCNT2 = 0;
    OCR2A = 0;
    OCR2B = 0;

    TIFR2 = _BV(OCF2A) | _BV(OCF2B) | _BV(TOV2);

    TCCR2B = _BV(CS21);
}

// ==================== CONFIGURACAO DO ADC ====================

/** @brief ADC: referencia AVCC, canal A0, prescaler 128 (125 kHz). */
void configurarAdc(void) {
    ADMUX = _BV(REFS0);
    ADCSRB = 0;
    DIDR0 |= _BV(ADC0D);

    ADCSRA = _BV(ADEN) | _BV(ADIE) |
             _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

    ADCSRA |= _BV(ADSC);
}

// ==================== INTERRUPCAO DO RESET ====================

/** @brief INT0 na borda de SUBIDA (botao apertado = HIGH). */
void configurarInterrupcaoReset(void) {
    EIMSK &= ~_BV(INT0);

    EICRA = (EICRA & ~(_BV(ISC00) | _BV(ISC01))) |
            _BV(ISC01) | _BV(ISC00);

    EIFR = _BV(INTF0);
    EIMSK |= _BV(INT0);
}

// ==================== LEITURAS COMPARTILHADAS ====================

/** @brief Le o contador de milissegundos de forma atomica. */
uint32_t tempoAtual(void) {
    uint8_t sregAnterior = SREG;
    cli();

    uint32_t copia = contadorMs;

    SREG = sregAnterior;
    return copia;
}

/** @brief Retorna 1 e copia a leitura se houver amostra nova do ADC. */
uint8_t obterAdc(uint16_t *valor) {
    uint8_t sregAnterior = SREG;
    cli();

    uint8_t disponivel = novaLeituraAdc;

    if (disponivel) {
        *valor = valorAdc;
        novaLeituraAdc = 0;
    }

    SREG = sregAnterior;
    return disponivel;
}

// ==================== VIDAS DO ROBO ====================

/** @brief Acende um LED por vida restante. */
void atualizarVidas(void) {
    if (vidas >= 1)
        PORTD |= _BV(PD4);
    else
        PORTD &= ~_BV(PD4);

    if (vidas >= 2)
        PORTD |= _BV(PD7);
    else
        PORTD &= ~_BV(PD7);

    if (vidas >= 3)
        PORTB |= _BV(PB0);
    else
        PORTB &= ~_BV(PB0);
}

// ==================== CONFIGURACAO DO LASER ====================

/** @brief Liga/desliga o pisca do laser (controlado pelo Timer0). */
void habilitarLaser(uint8_t habilitar) {
    uint8_t sregAnterior = SREG;
    cli();

    laserHabilitado = habilitar;
    contadorLaser = 0;

    if (habilitar)
        PORTB |= _BV(PB4);
    else
        PORTB &= ~_BV(PB4);

    SREG = sregAnterior;
}

// ==================== CONTROLE DOS MOTORES ====================

/**
 * @brief Aciona o motor A.
 * @param comando -255 (re total) a +255 (frente total); 0 = parado.
 */
void motorA(int16_t comando) {
    static int16_t anterior = 32767;

    if (INVERTER_MOTOR_A)
        comando = -comando;

    if (comando > 255) comando = 255;
    if (comando < -255) comando = -255;

    if (comando == anterior)
        return;

    anterior = comando;

    TCCR1A &= ~(_BV(COM1A1) | _BV(COM1A0));
    PORTB &= ~_BV(PB1);

    PORTC &= ~(_BV(PC1) | _BV(PC2));

    if (comando == 0)
        return;

    if (comando > 0)
        PORTC |= _BV(PC1);
    else
        PORTC |= _BV(PC2);

    OCR1A = (uint8_t)(comando > 0 ? comando : -comando);
    TCCR1A |= _BV(COM1A1);
}

/**
 * @brief Aciona o motor B.
 * @param comando -255 (re total) a +255 (frente total); 0 = parado.
 */
void motorB(int16_t comando) {
    static int16_t anterior = 32767;

    if (INVERTER_MOTOR_B)
        comando = -comando;

    if (comando > 255) comando = 255;
    if (comando < -255) comando = -255;

    if (comando == anterior)
        return;

    anterior = comando;

    TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
    PORTD &= ~_BV(PD3);

    PORTC &= ~(_BV(PC3) | _BV(PC4));

    if (comando == 0)
        return;

    if (comando > 0)
        PORTC |= _BV(PC3);
    else
        PORTC |= _BV(PC4);

    OCR2B = (uint8_t)(comando > 0 ? comando : -comando);
    TCCR2A |= _BV(COM2B1);
}

/** @brief Para os dois motores. */
void pararMotores(void) {
    motorA(0);
    motorB(0);
}

// ==================== BOTOES DO CONTROLE ====================

/** @brief Le os 4 botoes do controle (apertado = pino em GND). */
uint8_t lerControle(void) {
    uint8_t botoes = 0;

    if (!(PIND & _BV(PD6))) botoes |= _BV(BOTAO_ESQ_FRENTE);
    if (!(PIND & _BV(PD5))) botoes |= _BV(BOTAO_ESQ_RE);
    if (!(PINB & _BV(PB2))) botoes |= _BV(BOTAO_DIR_FRENTE);
    if (!(PINB & _BV(PB3))) botoes |= _BV(BOTAO_DIR_RE);

    return botoes;
}

/** @brief Debounce do controle: aceita a leitura estavel por 25 ms. */
void atualizarControle(uint32_t agora) {
    uint8_t leitura = lerControle();

    if (leitura != controleAnterior) {
        controleAnterior = leitura;
        mudancaControle = agora;
    }

    if ((uint32_t)(agora - mudancaControle) >=
        DEBOUNCE_CONTROLE_MS) {
        controleEstavel = leitura;
    }
}

/** @brief Converte o par frente/re de um lado em comando de motor. */
int16_t comandoLado(uint8_t frente, uint8_t re) {
    if (frente && !re) return  VELOCIDADE_NORMAL;
    if (re && !frente) return -VELOCIDADE_NORMAL;
    return 0;   // nenhum ou os dois apertados: para esse lado
}

/** @brief Controle tipo tanque: cada lado comanda o seu motor. */
void movimentarPeloControle(void) {
    int16_t esquerdo = comandoLado(
        (controleEstavel >> BOTAO_ESQ_FRENTE) & 1,
        (controleEstavel >> BOTAO_ESQ_RE) & 1);

    int16_t direito = comandoLado(
        (controleEstavel >> BOTAO_DIR_FRENTE) & 1,
        (controleEstavel >> BOTAO_DIR_RE) & 1);

#if MOTOR_A_ESQUERDO
    motorA(esquerdo);
    motorB(direito);
#else
    motorA(direito);
    motorB(esquerdo);
#endif
}

// ==================== INICIO E RESET DA PARTIDA ====================

/** @brief Restaura 3 vidas e recomeca pela calibracao do LDR. */
void iniciarPartida(uint32_t agora) {
    pararMotores();
    habilitarLaser(0);

    vidas = 3;
    atualizarVidas();

    estado = CALIBRANDO;
    inicioEstado = agora;

    somaCalibracao = 0;
    quantidadeCalibracao = 0;
    referenciaLdr = 0;

    receptorArmado = 0;
    contadorAcerto = 0;
    contadorRearme = 0;

    controleAnterior = lerControle();
    controleEstavel = 0;
    mudancaControle = agora;

    uint8_t sregAnterior = SREG;
    cli();

    novaLeituraAdc = 0;

    SREG = sregAnterior;
}

/** @brief Trata o pedido do INT0 com debounce de 30 ms. */
void verificarReset(uint32_t agora) {
    uint8_t sregAnterior = SREG;
    cli();

    uint8_t solicitacao = pedidoReset;
    pedidoReset = 0;

    SREG = sregAnterior;

    if (solicitacao && !resetPendente) {
        resetPendente = 1;
        inicioReset = agora;
    }

    if (resetPendente &&
        (uint32_t)(agora - inicioReset) >=
        DEBOUNCE_RESET_MS) {

        resetPendente = 0;

        // Botao ainda apertado (HIGH) -> reset confirmado
        if (PIND & _BV(PD2))
            iniciarPartida(agora);
    }
}

// ==================== DETECCAO DE ACERTOS ====================

/** @brief Perde uma vida, para os motores e bloqueia por 5 s. */
void registrarAcerto(uint32_t agora) {
    if (estado != ATIVO || vidas == 0)
        return;

    pararMotores();

    vidas--;
    atualizarVidas();

    receptorArmado = 0;
    contadorAcerto = 0;
    contadorRearme = 0;

    estado = BLOQUEADO;
    inicioEstado = agora;

    if (vidas == 0)
        habilitarLaser(0);
}

/** @brief Calibra o LDR e detecta acertos com confirmacao e histerese. */
void processarLdr(uint16_t leitura, uint32_t agora) {
    if (estado == CALIBRANDO) {
        somaCalibracao += leitura;
        quantidadeCalibracao++;

        if (quantidadeCalibracao >= AMOSTRAS_CALIBRACAO) {
            referenciaLdr =
                somaCalibracao / quantidadeCalibracao;

            receptorArmado = 1;
            estado = ATIVO;
            inicioEstado = agora;

            habilitarLaser(1);
        }

        return;
    }

    if (estado != ATIVO)
        return;

    int16_t diferenca;

    if (LUZ_AUMENTA_ADC)
        diferenca = (int16_t)leitura - (int16_t)referenciaLdr;
    else
        diferenca = (int16_t)referenciaLdr - (int16_t)leitura;

    if (!receptorArmado) {
        if (diferenca < (LIMIAR_ACERTO - HISTERESE_LDR)) {
            if (contadorRearme < AMOSTRAS_REARME)
                contadorRearme++;

            if (contadorRearme >= AMOSTRAS_REARME) {
                receptorArmado = 1;
                contadorRearme = 0;
            }
        } else {
            contadorRearme = 0;
        }

        return;
    }

    if (diferenca >= LIMIAR_ACERTO) {
        if (contadorAcerto < AMOSTRAS_CONFIRMACAO)
            contadorAcerto++;

        if (contadorAcerto >= AMOSTRAS_CONFIRMACAO)
            registrarAcerto(agora);
    } else {
        contadorAcerto = 0;
    }
}

// ==================== SEQUENCIA DO ROBO ====================

/** @brief Maquina de estados: ativo, bloqueado 5 s, giro 180, eliminado. */
void atualizarRobo(uint32_t agora) {
    switch (estado) {
        case CALIBRANDO:
            pararMotores();
            break;

        case ATIVO:
            movimentarPeloControle();
            break;

        case BLOQUEADO:
            pararMotores();

            if ((uint32_t)(agora - inicioEstado) >=
                TEMPO_BLOQUEIO_MS) {

                estado = GIRANDO;
                inicioEstado = agora;

                motorA(VELOCIDADE_GIRO);
                motorB(-VELOCIDADE_GIRO);
            }
            break;

        case GIRANDO:
            if ((uint32_t)(agora - inicioEstado) >=
                TEMPO_GIRO_180_MS) {

                pararMotores();

                if (vidas == 0) {
                    estado = ELIMINADO;
                } else {
                    estado = ATIVO;
                    receptorArmado = 0;
                    contadorAcerto = 0;
                    contadorRearme = 0;
                }

                inicioEstado = agora;
            }
            break;

        case ELIMINADO:
            pararMotores();
            break;
    }
}

// ==================== INTERRUPCOES ====================

/** @brief Timer0 (1 ms): relogio, disparo do ADC a cada 5 ms e pisca do laser. */
ISR(TIMER0_COMPA_vect) {
    contadorMs++;

    static uint8_t divisorAdc = 0;

    divisorAdc++;

    if (divisorAdc >= 5) {
        divisorAdc = 0;

        if (!(ADCSRA & _BV(ADSC)))
            ADCSRA |= _BV(ADSC);
    }

    if (laserHabilitado) {
        contadorLaser++;

        if (contadorLaser >= TEMPO_LASER_MS) {
            contadorLaser = 0;
            PORTB ^= _BV(PB4);
        }
    }
}

/** @brief Fim de conversao do ADC. */
ISR(ADC_vect) {
    valorAdc = ADC;
    novaLeituraAdc = 1;
}

/** @brief Botao de reset (borda de subida). */
ISR(INT0_vect) {
    pedidoReset = 1;
}

// ==================== PROGRAMA PRINCIPAL ====================

int main(void) {
    cli();

    configurarPinos();
    configurarTimers();
    configurarAdc();
    configurarInterrupcaoReset();

    iniciarPartida(0);

    sei();

    uint32_t ultimoProcessamento = UINT32_MAX;

    while (1) {
        uint32_t agora = tempoAtual();

        if (agora == ultimoProcessamento)
            continue;

        ultimoProcessamento = agora;

        verificarReset(agora);
        atualizarControle(agora);

        uint16_t leitura;

        if (obterAdc(&leitura))
            processarLdr(leitura, agora);

        atualizarRobo(agora);
    }
}
