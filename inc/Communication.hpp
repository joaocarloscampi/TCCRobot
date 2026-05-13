#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "inc/Motor.hpp"
#include "inc/Odometry.hpp"

// ================= CONFIG =================
#define START_HEADER 0x39
#define END_HEADER   0x55

#define RX_BUFFER_SIZE 256
#define FRAME_SIZE 8

typedef enum {
    MSG_START_HEADER    = 0,
    MSG_ID              = 1,
    MSG_BYTE1           = 2,
    MSG_BYTE2           = 3,
    MSG_BYTE3           = 4,
    MSG_BYTE4           = 5,
    MSG_CRC             = 6,
    MSG_END_HEADER      = 7
} msg_index;

enum ParserState {
    WAIT_START,
    READ_FRAME
};

typedef enum {
    SETPOINT_M1         = 0x00,
    SETPOINT_M2         = 0x01,
    SETPOINT_M3         = 0x02,
    SETPOINT_M4         = 0x03,
    SETPOINT_ROBOT      = 0x04,
    ACK_SETPOINT_M1     = 0x05,
    ACK_SETPOINT_M2     = 0x06,
    ACK_SETPOINT_M3     = 0x07,
    ACK_SETPOINT_M4     = 0x08,
    ACK_SETPOINT_ROBOT  = 0x09,
    REQUEST_M1          = 0x0A,
    REQUEST_M2          = 0x0B,
    REQUEST_M3          = 0x0C,
    REQUEST_M4          = 0x0D,
    SPEED_M1            = 0x0E,
    SPEED_M2            = 0x0F,
    SPEED_M3            = 0x10,
    SPEED_M4            = 0x11,
    REQUEST_ODOM        = 0x12,
    ODOM_LOCAL          = 0x13,
    ERROR_MSG           = 0xFF
} IDs;

// Definição da struct
struct BroadCast_Control {
    bool speed_M1 = false;
    bool speed_M2 = false;
    bool speed_M3 = false;
    bool speed_M4 = false;
    bool odom_local = false;
};

void parse_byte(uint8_t byte);
void process_frame();
void handle_message(uint8_t id, uint8_t* data);

class Communication {
public:
    /**
     * @brief Construtor da classe Encoder
     * 
     * @param spi           Objeto SPI responsavel pela comunicação com a placa
     * @param CS_Pin        Pino CS responsável por habilitar a comunicação SPI
     * 
     */
    Communication(uart_inst_t *uart);

    /**
     * @brief aaa
     * 
     * @param None
     * 
     * @return None
     * 
     */
    void init();
    void send(uint8_t id, uint8_t* data);
    void handle_message(uint8_t id, uint8_t* data);
    void process_frame();
    void parse_byte(uint8_t byte);
    void update();
    void add_motor(Motor* motor_address, uint8_t ID_MOTOR);
    void get_odometry(Odometry* odom);
    void broadcast_manager();

private:
    uart_inst_t* uart_handler_;

    // Buffer Circular
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint16_t head = 0;
    uint16_t tail = 0;

    ParserState state_ = WAIT_START;

    uint8_t frame_[8];
    uint8_t index_;

    void buffer_push(uint8_t byte);
    bool buffer_available();
    uint8_t buffer_pop();
    void on_uart_rx();

    static Communication* instance_;
    static void irq_handler_wrapper();

    Motor* motor1 = nullptr;
    Motor* motor2 = nullptr;
    Motor* motor3 = nullptr;
    Motor* motor4 = nullptr;

    Odometry* odometry = nullptr;

    BroadCast_Control cnt_broad;

};


#endif