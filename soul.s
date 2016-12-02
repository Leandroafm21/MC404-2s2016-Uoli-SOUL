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

<<<<<<< HEAD
    @ alocacao de limiares
    .set MAX_SPEED,                     #63
    .set MAX_ALARMS,                    #8
    .set MAX_CALLBACKS                  #8
    .set TIME_SZ                        #2000
    .set MIN_SENSOR_ID                  #0
    .set MAX_SENSOR_ID                  #15

=======
>>>>>>> 225ff5e50f2af675ebb0f3cf6bdf278a920a72c4
    @ alocacao das variaveis para tratamento de alarmes
    ALARMS_COUNT:   .word 0
    ALARMS_PTR:     .skip 32 * MAX_ALARMS
    ALARMS_TIME:    .skip 32 * MAX_ALARMS
<<<<<<< HEAD

    @ alocacao das variaveis para tratamento de callbacks
    CALLBACKS_COUNT:  .word 0
    CALLBACKS_PTR:    .skip 32 * MAX_CALLBACKS
    CALLBACKS_SON_ID: .skip 32 * MAX_CALLBACKS
    CALLBACKS_DIST:   .skip 32 * MAX_CALLBACKS

=======
    
>>>>>>> 225ff5e50f2af675ebb0f3cf6bdf278a920a72c4
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
    .set GPIO_DR                0x53F84000
    .set GPIO_GDIR              GPIO_DR + 0x04
    .set GPIO_PSR               GPIO_DR + 0x08
    
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
            mov r3, =TIME_SZ
            str r3, [r2]

            @ enables output compare channel 1
            ldr r2, =GPT_IR
            mov r3, #1
            str r3, [r2]

        SET_GPIO:
            
            @ set GDIR values according to hardware specifications
            ldr r2, =GPIO_GDIR
            mov r3, =0b11111111111111000000000000111110
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

            @ enables interruptions
            msr  CPSR_c, #0x13       @ SUPERVISOR mode, IRQ/FIQ enabled
    
    SET_STACK_POINTERS:
        @ continuar
    
    RETURN_USER:
        @ continuar
    
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

        @ retorna o fluxo
        sub lr, lr, #4
        movs pc, lr

        READ_SONAR:
            stmfd sp!, {r1-r4, lr}

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

            bic r3, r3, #0b00000000000000111111111111111100         @ remove o identificador e o valor de leitura do sonar atual
            lsl r0, r0, #2                                          @ desloca o identificador do sonar para se adequar a DR
            orr r3, r3, r0                                          @ insere o novo identificador do sonar no valor resultante de DR
            str r3, [r4]                                            @ atualiza o valor de DR

            trigger_activator:
                ldr r3, [r4]                                        @ carrega o o valor de DR em r3
                bic r3, r3, #0b00000000000000000000000000000010     @ desativa trigger
                str r3, [r4]                                        @ atualiza o valor de DR
                @ delay de 15ms
                orr r3, r3, #0b00000000000000000000000000000010     @ ativa trigger
                str r3, [r4]                                        @ atualiza o valor de DR
                @ delay de 15ms
                bic r3, r3, #0b00000000000000000000000000000010     @ desativa trigger
                str r3, [r4]                                        @ atualiza o valor de DR
            
            flag_activator:
                @ delay de 10ms
                ldr r3, [r4]                                        @ carrega novamente o valor de DR em r3
                bic r3, r3, #0b11111111111111111111111111111110     @ restaura apenas o valor de 'enable'
                cmp r5, #1                                          @ verifica se enable esta ativo
                bne flag_activator                                  @ se nao estiver, continua esperando
                bic r3, r3, #0b01111111111111000000000000111111     @ se estiver, restaura o valor de 'sonar data'
                lsr r3, r3, #6                                      @ desloca o valor de 'sonar data'
                mov r0, r3                                          @ move o valor lido para o registrador de retorno r0
                b fim_rs                                            @ pula para o fim da syscall

            @ trata erros
            erro_rs:
                mov r0, #-1

            @ termina syscall
            fim_rs:
                ldmfd sp!, {r1-r4, lr}
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
            stmfd sp!, {r1-r4, lr}

            @ verifica erros
            mov r2, #0
            cmp r0, r2
            blt erro_sms_mot                                        @ valor de motor invalido
            mov r2, #1
            cmp r0, r2
            bgt erro_sms_mot                                        @ valor de motor invalido
            mov r2, MAX_SPEED
            cmp r1, r2
            bgt erro_sms_vel                                        @ velocidade invalida

            @ atualiza valores de de velocidade
            ldr r4, =GPIO_DR                                        @ carrega o endereco do registrador DR em r4
            ldr r3, [r4]                                            @ carrega o valor contido no registrador DR em r3

            cmp r0, #1                                              @ verifica qual motor esta sendo modificado
            beq motor1_sms                                          @ salta para a instrucao de configuracao do motor1

            bic r3, r3, #0b00000001111111000000000000000000         @ remove a velocidade atual do motor 0
            lsl r1, r1, #19                                         @ desloca a velocidade desejada para se adequar a DR
            orr r3, r3, r1                                          @ insere a nova velocidade no valor resultante de DR
            str r3, [r4]                                            @ atualiza o valor de DR
            b fim_sms                                               @ salta para o fim da syscall

            motor1_sms:
            bic r3, r3, #0b11111110000000000000000000000000         @ remove a velocidade atual do motor 0
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
            mov r2, MAX_SPEED
            cmp r0, r2
            blt erro_smss_1                                         @ velocidade invalida
            mov r2, MAX_SPEED
            cmp r1, r2
            blt erro_smss_2                                         @ velocidade invalida

            @ atualiza valores de velocidade
            ldr r4, =GPIO_DR                                        @ carrega o endereco do registrador DR em r4
            ldr r3, [r4]                                            @ carrega o valor contido no registrador DR em r3

            bic r3, r3, #0b11111111111111000000000000000000         @ remove a velocidade atual de ambos os motores
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
                ldmfd sp!, r1-r4 lr}
                movs pc, lr

        GET_TIME:
            stmfd sp!, {lr}

            mov r0, =SYSTEM_TIME
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
