.section .iv, "a"

_start:

interrupt_vector:
    .org 0x00
        b RESET_HANDLER
    .org 0x08
        b SVC_HANDLER
    .org 0x18
        b IRQ_HANDLER

    @ inicio do codigo
    .org 0x100

.text

    @@@@@@@@@@@@@
    @ Constants @
    @@@@@@@@@@@@@

    @ GPT addresses
    .set GPT_CR,                0x53FA0000
    .set GPT_PR,                0x53FA0004
    .set GPT_SR,                0x53FA0008
    .set GPT_IR,                0x53FA000C
    .set GPT_OCR_ONE,           0x53FA0010

    @ GPIO addresses
    .set GPIO_DR,               0x53F84000
    .set GPIO_GDIR,             GPIO_DR + 0x04
    .set GPIO_PSR,              GPIO_DR + 0x08

    @ GPIO masks
    .set GDIR_MASK,             0b11111111111111000000000000111110
    .set SONAR_ID_MASK,         0b00000000000000000000000000111100
    .set SONAR_DATA_MASK,       0b00000000000000111111111111000000
    .set TRIGGER_MASK,          0b00000000000000000000000000000010
    .set MOTOR_0_MASK,          0b00000001111111000000000000000000
    .set MOTOR_1_MASK,          0b11111110000000000000000000000000
    .set ENABLE_ISOLATOR,       0b11111111111111111111111111111110
    .set SONAR_DATA_ISOLATOR,   0b01111111111111000000000000111111

    @ TZIC addresses
    .set TZIC_BASE,             0x0FFFC000
    .set TZIC_INTCTRL,          0x0
    .set TZIC_INTSEC1,          0x84
    .set TZIC_ENSET1,           0x104
    .set TZIC_PRIOMASK,         0xC
    .set TZIC_PRIORITY9,        0x424

    @ System constants
    .set MAX_SPEED,             63
    .set MAX_ALARMS,            8
    .set MAX_CALLBACKS,         8
    .set TIME_SZ,               2000
    .set MIN_SENSOR_ID,         0
    .set MAX_SENSOR_ID,         15

    @ Stack constants
    .set STACK_SIZE,            1024

    @@@@@@@@@@@@@@@@@@@@
    @ System Initiator @
    @@@@@@@@@@@@@@@@@@@@

    RESET_HANDLER:

        @ set system time as 0
        ldr r2, =SYSTEM_TIME
        mov r0, #0
        str r0, [r2]

        @ configures interrupt table
        ldr r0, =interrupt_vector
        mcr p15, 0, r0, c12, c0, 0

        SET_GPT:

            @ enable control register and configures clock cycles counter
            ldr r2, =GPT_CR
            mov r3, #0x00000041
            str r3, [r2]

            @ set prescaler to 0
            ldr r2, =GPT_PR
            mov r3, #0
            str r3, [r2]

            @ set max couting value
            ldr r2, =GPT_OCR_ONE
            mov r3, #TIME_SZ
            str r3, [r2]

            @ enables output compare channel 1
            ldr r2, =GPT_IR
            mov r3, #1
            str r3, [r2]

        SET_GPIO:

            @ set GDIR values according to hardware specifications
            ldr r2, =GPIO_GDIR
            ldr r3, =GDIR_MASK
            str r3, [r2]

        SET_TZIC:

            @ r1 <= TZIC_BASE
            ldr	r1, =TZIC_BASE

            @ configures interrupt 39 as unsafe
            mov	r0, #(1 << 7)
            str	r0, [r1, #TZIC_INTSEC1]

            @ enables interrupt39 (GPT)
            @ reg1 bit 7 (gpt)
            mov	r0, #(1 << 7)
            str	r0, [r1, #TZIC_ENSET1]

            @ configures interrupt39 priority to 1
            @ reg9, byte 3
            ldr r0, [r1, #TZIC_PRIORITY9]
            bic r0, r0, #0xFF000000
            mov r2, #1
            orr r0, r0, r2, lsl #24
            str r0, [r1, #TZIC_PRIORITY9]

            @ set PRIOMASK as 0
            eor r0, r0, r0
            str r0, [r1, #TZIC_PRIOMASK]

            @ enables interruption controller
            mov	r0, #1
            str	r0, [r1, #TZIC_INTCTRL]

        SET_STACK_POINTERS:

            @ Set stacks
            ldr r1, =STACK_POINTER
            ldr r2, =STACK_SIZE
            ldr r2, [r2]

            add r1, r1, r2
            msr CPSR_c, #0x12       @ IRQ mode
            mov sp, r1

            add r1, r1, r2
            msr CPSR_c, #0x1F       @ SYSTEM mode
            mov sp, r1

            add r1, r1, r2
            msr CPSR_c, #0x13       @ Supervisor mode
            mov sp, r1

        RETURN_USER:

            ldr r0, =0x77802000     @ default start section of the code
            msr CPSR_c, #0x104      @ change to USER mode
            bx r0                   @ start program

    @@@@@@@@@@@@
    @ Handlers @
    @@@@@@@@@@@@

    SVC_HANDLER:

        @ le e realiza a syscall desejada
        cmp r8, #16
        beq READ_SONAR

        cmp r8, #17
        beq REGISTER_PROXIMITY_CALLBACK

        cmp r8, #18
        beq SET_MOTOR_SPEED

        cmp r8, #19
        beq SET_MOTORS_SPEED

        cmp r8, #20
        beq GET_TIME

        cmp r8, #21
        bleq SET_TIME

        cmp r8, #22
        beq SET_ALARM

        cmp r8, #23
        beq RETURN_TO_IRQ

        @ retorna o fluxo
        sub lr, lr, #4
        movs pc, lr

        READ_SONAR:
            stmfd sp!, {r1-r5, lr}

            @ verfica erros
            mov r1, #0
            cmp r0, r1
            blt erro_rs                                             @ valor do sonar invalido
            mov r1, #15
            cmp r0, r1
            bgt erro_rs                                             @ valor do sonar invalido

            @ realiza a leitura de um sonar
            ldr r4, =GPIO_DR                                        @ carrega o endereco do registrador DR em r4
            ldr r3, [r4]                                            @ carrega o valor contido no registrador DR em r3

            bic r3, r3, #SONAR_ID_MASK                              @ remove o identificador do sonar atual
            lsl r0, r0, #2                                          @ desloca o identificador do sonar para se adequar a DR
            orr r3, r3, r0                                          @ insere o novo identificador do sonar no valor resultante de DR
            str r3, [r4]                                            @ atualiza o valor de DR

            trigger_activator:
                ldr r3, [r4]                                        @ carrega o o valor de DR em r3
                bic r3, r3, #TRIGGER_MASK                           @ desativa trigger
                str r3, [r4]                                        @ atualiza o valor de DR
                @ delay de 15ms
                orr r3, r3, #TRIGGER_MASK                           @ ativa trigger
                str r3, [r4]                                        @ atualiza o valor de DR
                @ delay de 15ms
                bic r3, r3, #TRIGGER_MASK                           @ desativa trigger
                str r3, [r4]                                        @ atualiza o valor de DR

            flag_activator:
                @ delay de 10ms
                ldr r3, [r4]                                        @ carrega novamente o valor de DR em r3
                bic r3, r3, #ENABLE_ISOLATOR                        @ restaura apenas o valor de 'enable'
                cmp r5, #1                                          @ verifica se enable esta ativo
                bne flag_activator                                  @ se nao estiver, continua esperando
                ldr r5, =SONAR_DATA_ISOLATOR                        @ se estiver,
                bic r3, r3, r5                                      @ restaura o valor de 'sonar data'
                lsr r3, r3, #6                                      @ desloca o valor de 'sonar data'
                mov r0, r3                                          @ move o valor lido para o registrador de retorno r0
                b fim_rs                                            @ pula para o fim da syscall

            @ trata erros
            erro_rs:
                mov r0, #-1

            @ termina syscall
            fim_rs:
                ldmfd sp!, {r1-r5, lr}
                movs pc, lr

        REGISTER_PROXIMITY_CALLBACK:
            stmfd sp!, {r4-r11, lr}

            @ verifica se ha espaco
            ldr r4, =CALLBACKS_COUNT
            ldr r5, [r4]

            cmp r5, #MAX_CALLBACKS @ COUNT <= MAX
            blt CALLBACKS_AVAILABLE

            @ retornar -1 em caso de estouro
            mov r0, #-1
            movs pc, lr

            CALLBACKS_AVAILABLE:
            @ verificar validade do id do sensor
            cmp r0, #MIN_SENSOR_ID
            bge VALID_GTMIN

            @ retornar -2 em caso de sensor invalido
            mov r0, #-2
            movs pc, lr

            VALID_GTMIN:
            cmp r0, #MAX_SENSOR_ID
            ble VALID_LEMAX

            @ retornar -2 em caso de sensor invalido
            mov r0, #-2
            movs pc, lr

            VALID_LEMAX:

            ldr r6, =CALLBACKS_SON_ID
            ldr r7, =CALLBACKS_PTR
            ldr r8, =CALLBACKS_DIST
            mov r9, #-1 @indice
            callbacks_find_free:
              add r9, r9, #1 @incrementa indice
              ldr r10, [r7, r9, lsl #3] @carrega ponteiro
              cmp r10, #0 @verifica se o ponteiro eh invalido
              bne alarm_find_free

            str r0, [r6, r9, lsl #3] @ CALLBACKS_SON_ID + 32 * CALLBACKS_COUNT = r0
            str r2, [r7, r9, lsl #3] @ CALLBACKS_PTR + 32 * CALLBACKS_COUNT = r0
            str r1, [r8, r9, lsl #3] @ CALLBACKS_DIST + 32 * CALLBACKS_COUNT = r0

            add r5, r5, #1 @ incrementa o contador de callbacks
            str r5, [r4]

            ldmfd sp!, {r4-r11, lr}

            movs pc, lr

        SET_MOTOR_SPEED:
            stmfd sp!, {r1-r4, lr}

            @ verifica erros
            mov r2, #0
            cmp r0, r2
            blt erro_sms_mot                                        @ valor de motor invalido
            mov r2, #1
            cmp r0, r2
            bgt erro_sms_mot                                        @ valor de motor invalido
            mov r2, #MAX_SPEED
            cmp r1, r2
            bgt erro_sms_vel                                        @ velocidade invalida

            @ atualiza valores de de velocidade
            ldr r4, =GPIO_DR                                        @ carrega o endereco do registrador DR em r4
            ldr r3, [r4]                                            @ carrega o valor contido no registrador DR em r3

            cmp r0, #1                                              @ verifica qual motor esta sendo modificado
            beq motor1_sms                                          @ salta para a instrucao de configuracao do motor1

            bic r3, r3, #MOTOR_0_MASK                               @ remove a velocidade atual do motor 0
            lsl r1, r1, #19                                         @ desloca a velocidade desejada para se adequar a DR
            orr r3, r3, r1                                          @ insere a nova velocidade no valor resultante de DR
            str r3, [r4]                                            @ atualiza o valor de DR
            b fim_sms                                               @ salta para o fim da syscall

            motor1_sms:
            bic r3, r3, #MOTOR_1_MASK                               @ remove a velocidade atual do motor 0
            lsl r1, r1, #26                                         @ desloca a velocidade desejada para se adequar a DR
            orr r3, r3, r1                                          @ insere a nova velocidade no valor resultante de DR
            str r3, [r4]                                            @ atualiza o valor de DR
            b fim_sms                                               @ salta para o fim da syscall

            @ trata erros
            erro_sms_mot:
                mov r0, #-1
                b fim_sms
            erro_sms_vel:
                mov r0, #-2

            @ termina syscall
            fim_sms:
                ldmfd sp!, {r1-r4, lr}
                movs pc, lr

        SET_MOTORS_SPEED:
            stmfd sp!, {r1-r4, lr}

            @ verifica erros
            mov r2, #MAX_SPEED
            cmp r0, r2
            blt erro_smss_1                                         @ velocidade invalida
            mov r2, #MAX_SPEED
            cmp r1, r2
            blt erro_smss_2                                         @ velocidade invalida

            @ atualiza valores de velocidade
            ldr r4, =GPIO_DR                                        @ carrega o endereco do registrador DR em r4
            ldr r3, [r4]                                            @ carrega o valor contido no registrador DR em r3

            bic r3, r3, #MOTOR_0_MASK                               @ remove a velocidade atual do motor 0
            bic r3, r3, #MOTOR_1_MASK                               @ remove a velocidade atual do motor 1
            lsl r1, r1, #19                                         @ desloca a velocidade desejada para se adequar a DR (motor 0)
            orr r3, r3, r1                                          @ insere a nova velocidade do motor 0 no valor resultante de DR
            lsl r1, r1, #26                                         @ desloca a velocidade desejada para se adequar a DR (motor 1)
            orr r3, r3, r1                                          @ insere a nova velocidade do motor 1 no valor resultante de DR
            str r3, [r4]                                            @ atualiza o valor de DR
            b fim_smss                                              @ salta para o fim da syscall

            @ trata erros
            erro_smss_1:
                mov r0, #-1
                b fim_smss
            erro_smss_2:
                mov r0, #-2

            @ termina syscall
            fim_smss:
                ldmfd sp!, {r1-r4, lr}
                movs pc, lr

        GET_TIME:
            stmfd sp!, {lr}

            mov r0, #SYSTEM_TIME
            ldr r0, [r0]

            ldmfd sp!, {lr}
            movs pc, lr

        SET_TIME:
            stmfd sp!, {r1, lr}

            ldr r1, =SYSTEM_TIME
            str r0, [r1]

            ldmfd sp!, {r1, lr}
            movs pc, lr

        SET_ALARM: @ r0: ponteiro, r1: tempo
            stmfd sp!, {r4-r11, lr}

            @ verifica se ja atingimos o numero maximo de alarmes
            ldr r6, =ALARMS_COUNT
            ldr r7, [r6]
            cmp r7, #MAX_ALARMS
            blt ALARMS_AVAILABLE
            @ retornar -1 em caso de estouro
            mov r0, #-1
            movs pc, lr

            ALARMS_AVAILABLE:
            @ impede que o tempo seja menor que o tempo do sistema
            ldr r4, =SYSTEM_TIME
            ldr r5, [r4]
            cmp r1, r5 @ TIME >= SYSTEM_TIME?
            bhs VALID_TIME
            @ retornar -2 em caso de tempo invalido
            mov r0, #-2
            movs pc, lr

            VALID_TIME:

            ldr r4, =ALARMS_PTR @ vetor de ponteiros
            mov r8, #-1 @indice do vetor
            alarm_find_free:
              add r8, r8, #1 @incrementa indice
              ldr r9, [r4, r8, lsl #3] @carrega ponteiro
              cmp r9, #0 @ verifica se o ponteiro eh invalido
              bne alarm_find_free

            str r0, [r4, r8, lsl #3] @ guarda o ptr na posicao ALARMS_PTR + 32 * ALARMS_COUNT

            ldr r5, =ALARMS_TIME @ vetor de tempos
            str r1, [r5, r8, lsl #3] @ guarda o tempo na posicao ALARMS_TIME + 32 * ALARMS_COUNT

            add r7, r7, #1 @ incrementa o contador de alarmes
            str r7, [r6]

            ldmfd sp!, {r4-r11, lr}

            @ retorna o fluxo
            movs pc, lr

        RETURN_TO_IRQ:
            stmfd sp!, {lr}

            msr CPSR_c, #0x12

            ldmfd sp!, {lr}
            mov pc, lr

    IRQ_HANDLER:
        stmfd sp!, {r0-r12, lr}

        mrs r0, SPSR
        stmfd sp!, {r0} @ salva modo SPSR na stack

        @ informa que o processador sabe sobre a ocorrencia da interrupcao
        ldr r2, =GPT_SR
        mov r3, #1
        str r3, [r2]

        @ incrementa contador
        ldr r0, =SYSTEM_TIME
        ldr r1, [r0]
        add r1, r1, #1
        str r1, [r0]

        @ TRATAMENTO DE ALARMES:
        ldr r2, =ALARMS_TIME @carrega ponteiro do vetor de tempo dos alarmes
        ldr r5, =ALARMS_PTR @carrega ponteiro do vetor de funcoes dos alarmes
        ldr r8, =ALARMS_COUNT
        mov r3, #0 @indice
        handle_alarms:
            cmp r3, #MAX_ALARMS @verifica se chegamos ao final da lista de alarmes
            bhi end_alarms
            ldr r4, [r2, r3, lsl #3] @carrega tempo do alarme
            cmp r4, #0 @compara com tempo com 0, se igual, o alarme esta vazio
            beq handle_alarms
            @alarme existe, verificar se estamos em tempo de chamar a funcao
            cmp r4, #SYSTEM_TIME
            bhs handle_alarms
            @alarme deve ser ativado
            ldr r6, [r5, r3, lsl #3] @carrega o ponteiro para funcao
            stmfd sp!, {r0 - r4, lr}
            msr CPSR_c, #0x10 @muda para modo usuario
            blx r6 @chama a funcao
            mov r7, #23 @volta para o modo irq
            svc 0x0
            ldmfd sp!, {r0 - r4, lr}
            ldr r9, [r8] @carrega o contador de alarmes
            sub r9, r9, #1 @subtrai 1 do contador
            str r9, [r8] @guarda o novo valor do contador
            mov r10, #0
            str r10, [r2, r3, lsl #3] @limpa o tempo do alarme
            b handle_alarms

        end_alarms:

        @ TRATAMENTO DE SENSOR CALLBACKS
        ldr r1, =CALLBACKS_COUNT
        ldr r2, =CALLBACKS_PTR
        ldr r3, =CALLBACKS_DIST
        ldr r4, =CALLBACKS_SON_ID
        mov r5, #0 @indice
        handle_callbacks:
            cmp r5, #MAX_CALLBACKS @verifica se chegamos ao final da lista de callbacks
            bhi end_callbacks
            ldr r6, [r3, r5, lsl #3] @carrega a distancia
            cmp r6, #0 @se for 0, o callback esta vazio
            beq handle_callbacks
            @callback existe, verificar distancia
            ldr r0, [r4, r5, lsl #3] @carrega o id do sonar
            ldr r7, [r3, r5, lsl #3] @carrega a distancia
            bl READ_SONAR
            cmp r0, r7
            bgt handle_callbacks
            @distancia menor q a limiar, chamar funcao
            ldr r7, [r2, r5, lsl #3] @carrega o ponteiro para funcao
            stmfd sp!, {r0 - r4, lr}
            msr CPSR_c, #0x10 @muda para modo usuario
            blx r7 @chama a funcao
            mov r7, #23 @volta para o modo irq
            svc 0x0
            ldmfd sp!, {r0 - r4, lr}
            ldr r8, [r1] @carrega o contador de alarmes
            sub r8, r8, #1 @decrementa o contador
            str r8, [r1] @guarda o novo valor
            mov r10, #0
            str r10, [r3, r5, lsl #3] @limpa a distancia
            b handle_callbacks

        end_callbacks:

        ldmfd sp!, {r0} @ recupera o modo SPSR
        msr SPSR, r0

        ldmfd sp!, {r0-r12, lr}

        @ retorna o fluxo
        sub lr, lr, #4
        movs pc, lr

.data
    @ alocacao da variavel para o tempo do sistema
    SYSTEM_TIME: .word 0

    @ alocacao das variaveis para tratamento de alarmes
    ALARMS_COUNT:   .word 0
    ALARMS_PTR:     .fill MAX_ALARMS, 8, 0
    ALARMS_TIME:    .fill MAX_ALARMS, 8, 0

    @ alocacao das variaveis de pilhas
    STACK_POINTER:   .skip STACK_SIZE * 4

    @ alocacao das variaveis para tratamento de callbacks
    CALLBACKS_COUNT:  .word 0
    CALLBACKS_PTR:    .fill MAX_CALLBACKS, 8, 0
    CALLBACKS_SON_ID: .fill MAX_CALLBACKS, 8, 0
    CALLBACKS_DIST:   .fill MAX_CALLBACKS, 8, 0
