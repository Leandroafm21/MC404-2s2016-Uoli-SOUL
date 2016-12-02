.section .iv, "a"

_start:

interrupt_vector:
    .org 0x00
        b RESET_HANDLER
    .org 0x08
        b SVC_HANDLER
    .org 0x18
        b IRQ_HANDLER

.data
    .align 4

    @ alocacao da variavel para o tempo do sistema
    SYSTEM_TIME: .word 0

    @ alocacao de limiares
    .set MAX_SPEED,                     #63
    .set MAX_ALARMS,                    #8
    .set MAX_CALLBACKS                  #8
    .set TIME_SZ                        #2000
    .set MIN_SENSOR_ID                  #0
    .set MAX_SENSOR_ID                  #15

    @ alocacao das variaveis para tratamento de alarmes
    ALARMS_COUNT:   .word 0
    ALARMS_PTR:     .skip 32 * MAX_ALARMS
    ALARMS_TIME:    .skip 32 * MAX_ALARMS

    @ alocacao das variaveis para tratamento de callbacks
    CALLBACKS_COUNT:  .word 0
    CALLBACKS_PTR:    .skip 32 * MAX_CALLBACKS
    CALLBACKS_SON_ID: .skip 32 * MAX_CALLBACKS
    CALLBACKS_DIST:   .skip 32 * MAX_CALLBACKS

    @ inicio do codigo
    .org 0x100

.text
    RESET_HANDLER:
        @ inicializa o tempo do sistema com o valor 0
        ldr r2, =SYSTEM_TIME
        mov r0, #0
        str r0, [r2]

        @ configura a tabela de interrupcao
        ldr r0, =interrupt_vector
        mcr p15, 0, r0, c12, c0, 0

        SET_GPT:
            @ constantes para os enderecos do GPT
            .set GPT_CR,                0x53FA0000
            .set GPT_PR,                0x53FA0004
            .set GPT_SR,                0x53FA0008
            .set GPT_IR,                0x53FA000C
            .set GPT_OCR_ONE,           0x53FA0010

            @ habilita registro de controle e configura contador de ciclos de relogio
            ldr r2, =GPT_CR
            mov r3, #0x00000041
            str r3, [r2]

            @ zera o prescaler
            ldr r2, =GPT_PR
            mov r3, #0
            str r3, [r2]

            @ insere o valor maximo da contagem
            ldr r2, =GPT_OCR_ONE
            mov r3, =TIME_SZ
            str r3, [r2]

            @ habilita o canal de comparacao de saida
            ldr r2, =GPT_IR
            mov r3, #1
            str r3, [r2]

        SET_GDIR:
            .set GPIO_DR                0x53F84000
            .set GPIO_GDIR             GPIO_DR + 0x04
            .set GPIO_PSR              GPIO_DR + 0x08

        SET_TZIC:
            @ constantes para os enderecos do TZIC
            .set TZIC_BASE,             0x0FFFC000
            .set TZIC_INTCTRL,          0x0
            .set TZIC_INTSEC1,          0x84
            .set TZIC_ENSET1,           0x104
            .set TZIC_PRIOMASK,         0xC
            .set TZIC_PRIORITY9,        0x424

            @ liga o controlador de interrupcoes
            @ R1 <= TZIC_BASE
            ldr	r1, =TZIC_BASE

            @ configura interrupcao 39 do GPT como nao segura
            mov	r0, #(1 << 7)
            str	r0, [r1, #TZIC_INTSEC1]

            @ habilita interrupcao 39 (GPT)
            @ reg1 bit 7 (gpt)
            mov	r0, #(1 << 7)
            str	r0, [r1, #TZIC_ENSET1]

            @ configura a prioridade da interrupcao 39 para 1
            @ reg9, byte 3
            ldr r0, [r1, #TZIC_PRIORITY9]
            bic r0, r0, #0xFF000000
            mov r2, #1
            orr r0, r0, r2, lsl #24
            str r0, [r1, #TZIC_PRIORITY9]

            @ configura PRIOMASK como 0
            eor r0, r0, r0
            str r0, [r1, #TZIC_PRIOMASK]

            @ habilita o controlador de interrupcoes
            mov	r0, #1
            str	r0, [r1, #TZIC_INTCTRL]

            @ habilita interrupcoes
            msr  CPSR_c, #0x13       @ SUPERVISOR mode, IRQ/FIQ enabled

            @ aguarda interrupcao
            loop:
                b loop

    SVC_HANDLER:
        @ habilita interrupcoes
        msr  CPSR_c, #0x13       @ SUPERVISOR mode, IRQ/FIQ enabled

        @ le e realiza a syscall desejada
        mov r8, #16
        cmp r8, r7
        bleq READ_SONAR

        mov r8, #17
        cmp r8, r7
        bleq REGISTER_PROXIMITY_CALLBACK

        mov r8, #18
        cmp r8, r7
        bleq SET_MOTOR_SPEED

        mov r8, #19
        cmp r8, r7
        bleq SET_MOTORS_SPEED

        mov r8, #20
        cmp r8, r7
        bleq GET_TIME

        mov r8, #21
        cmp r8, r7
        bleq SET_TIME

        mov r8, #22
        cmp r8, r7
        bleq SET_ALARM

        @ retorna o fluxo
        sub lr, lr, #4
        movs pc, lr

        READ_SONAR:
            stmfd sp!, {r4-r11, lr}

            @ verfica erros
            mov r1, #0
            cmp r0, r1
            blt erro_rs
            mov r1, #15
            cmp r0, r1
            bgt erro_rs

            @ realiza a leitura de um sonar
            ldr r4, =GPIO_DR                                        @ carrega o endereco do registrador DR em r4
            ldr r3, [r4]                                            @ carrega o valor contido no registrador DR em r3

            bic r3, r3, #0b00000000000000111111111111111101         @ remove o identificador e o valor de leitura do sonar atual e desativa 'enable'
            lsl r0, r0, #27                                         @ desloca o identificador do sonar para se adequar a DR
            add r3, r3, r1                                          @ insere o novo identificador do sonar no valor resultante de DR
            str r3, [r4]                                            @ atualiza o valor de DR

            trigger_activator:
                ldr r7, =GPIO_PSR                                   @ carrega o endereco do registrador PSR em r5
                bic r3, r3, #0b00000000000000000000000000000010     @ desativa trigger
                str r3, [r4]                                        @ atualiza o valor de DR
                @ delay de 15ms
                add r3, r3, #0b00000000000000000000000000000010     @ ativa trigger
                str r3, [r4]                                        @ atualiza o valor de DR
                @ delay de 15ms
                bic r3, r3, #0b00000000000000000000000000000010     @ desativa trigger
                str r3, [r4]                                        @ atualiza o valor de DR

                ldr r5, [r7]                                        @ carrega o valor contido no registrador PSR em r5
                bic r5, r5, #0b11111111111111111111111111111110     @ restaura apenas o valor de 'enable'
                cmp r5, #1                                          @ verifica se enable esta ativo
                bne trigger_activator                               @ se nao estiver, realiza outra ativacao do trigger
                ldr r6, [r7]                                        @ se estiver, carrega novamente o valor contido no registrador PSR, agora em r6
                bic r6, r6, #0b01111111111111000000000000111111     @ restaura apenas o valor de 'sonar data'
                lsr r6, r6, #6                                      @ desloca o valor de 'sonar data'
                mov r0, r6                                          @ move o valor lido para o registrador de retorno r0
                b fim_rs                                            @ pula para o fim da syscall

            @ trata erros
            erro_rs:
                mov r0, #-1

            @ termina syscall
            fim_rs:
                ldmfd sp!, {r4-r11, lr}
                sub lr, lr, #4
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
            str r0, [r6, r5, lsl #5] @ CALLBACKS_SON_ID + 32 * CALLBACKS_COUNT = r0

            ldr r7, =CALLBACKS_PTR
            str r2, [r7, r5, lsl #5] @ CALLBACKS_PTR + 32 * CALLBACKS_COUNT = r0

            ldr r8, =CALLBACKS_DIST
            str r1, [r8, r5, lsl #5] @ CALLBACKS_DIST + 32 * CALLBACKS_COUNT = r0

            add r5, r5, #1 @ incrementa o contador de callbacks
            str r5, [r4]

            ldmfd sp!, {r4-r11, lr}

            movs pc, lr

        SET_MOTOR_SPEED:
            stmfd sp!, {r4-r11, lr}

            @ verifica erros
            mov r2, #0
            cmp r0, r2
            blt erro_sms_mot
            mov r2, #1
            cmp r0, r2
            bgt erro_sms_mot
            mov r2, MAX_SPEED
            cmp r1, r2
            bgt erro_sms_vel

            @ atualiza valores de de velocidade
            ldr r4, =GPIO_DR                                        @ carrega o endereco do registrador DR em r4
            ldr r3, [r4]                                            @ carrega o valor contido no registrador DR em r3

            cmp r0, #1                                              @ verifica qual motor esta sendo modificado
            beq motor1_sms                                          @ salta para a instrucao de configuracao do motor1

            bic r3, r3, #0b00000001111111000000000000000000         @ remove a velocidade atual do motor 0
            lsl r1, r1, #18                                         @ desloca a velocidade desejada para se adequar a DR
            add r3, r3, r1                                          @ insere a nova velocidade no valor resultante de DR
            str r3, [r4]                                            @ atualiza o valor de DR
            b fim_sms                                               @ salta para o fim da syscall

            motor1_sms:
            bic r3, r3, #0b11111110000000000000000000000000         @ remove a velocidade atual do motor 0
            lsl r1, r1, #25                                         @ desloca a velocidade desejada para se adequar a DR
            add r3, r3, r1                                          @ insere a nova velocidade no valor resultante de DR
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
                ldmfd sp!, {r4-r11, lr}
                sub lr, lr, #4
                movs pc, lr

        SET_MOTORS_SPEED:
            stmfd sp!, {r4-r11, lr}

            @ verifica erros
            mov r2, MAX_SPEED
            cmp r0, r2
            blt erro_smss_1
            mov r2, MAX_SPEED
            cmp r1, r2
            blt erro_smss_2

            @ atualiza valores de velocidade
            ldr r4, =GPIO_DR                                        @ carrega o endereco do registrador DR em r4
            ldr r3, [r4]                                            @ carrega o valor contido no registrador DR em r3

            bic r3, r3, #0b11111111111111000000000000000000         @ remove a velocidade atual de ambos os motores
            lsl r1, r1, #18                                         @ desloca a velocidade desejada para se adequar a DR (motor 0)
            add r3, r3, r1                                          @ insere a nova velocidade do motor 0 no valor resultante de DR
            lsl r1, r1, #25                                         @ desloca a velocidade desejada para se adequar a DR (motor 1)
            add r3, r3, r1                                          @ insere a nova velocidade do motor 1 no valor resultante de DR
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
                ldmfd sp!, {r4-r11, lr}
                sub lr, lr, #4
                movs pc, lr

        GET_TIME:
            stmfd sp!, {r4-r11, lr}

            mov r0, =SYSTEM_TIME

            ldmfd sp!, {r4-r11, lr}
            sub lr, lr, #4
            movs pc, lr

        SET_TIME:
            stmfd sp!, {r4-r11, lr}

            ldr r1, =SYSTEM_TIME
            str r0, [r1]

            ldmfd sp!, {r4-r11, lr}
            sub lr, lr, #4
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
            str r0, [r4, r7, lsl #5] @ guarda o ptr na posicao ALARMS_PTR + 32 * ALARMS_COUNT

            ldr r5, =ALARMS_TIME @ vetor de tempos
            str r1, [r5, r7, lsl #5] @ guarda o tempo na posicao ALARMS_TIME + 32 * ALARMS_COUNT

            add r7, r7, #1 @ incrementa o contador de alarmes
            str r7, [r6]

            stmfd sp!, {r4-r11, lr}

            @ retorna o fluxo
            movs pc, lr

    IRQ_HANDLER:

        @ informa que o processador sabe sobre a ocorrencia da interrupcao
        ldr r2, =GPT_SR
        mov r3, #1
        str r3, [r2]

        @ incrementa contador
        ldr r0, =CONTADOR
        ldr r1, [r0]
        add r1, r1, #1
        str r1, [r0]

        @ retorna o fluxo
        sub lr, lr, #4
        movs pc, lr
