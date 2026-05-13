#include "Communication.hpp"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include <string.h>
#include "inc/Hardware.h"

bool led_state_comm = false;

// ================= CONFIG =================
#define RX_BUFFER_SIZE 256

// ================= CRC-8 =================
static uint8_t crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0x00;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ================= INSTÂNCIA ESTÁTICA =================
Communication* Communication::instance_ = nullptr;

// ================= ISR WRAPPER =================
void Communication::irq_handler_wrapper() {
    if (instance_) {
        instance_->on_uart_rx();
    }
}

// ================= CLASSE =================

Communication::Communication(uart_inst_t* uart) {
    uart_handler_ = uart;
    head = 0;
    tail = 0;
}

void Communication::init() {

    instance_ = this;

    uart_set_irq_enables(uart_handler_, true, false);

    if (uart_handler_ == uart0) {
        irq_set_exclusive_handler(UART0_IRQ, irq_handler_wrapper);
        irq_set_enabled(UART0_IRQ, true);
    } else {
        irq_set_exclusive_handler(UART1_IRQ, irq_handler_wrapper);
        irq_set_enabled(UART1_IRQ, true);
    }
}

// ================= PARSER =================

void Communication::update() {
    while (buffer_available()) {
        uint8_t byte = buffer_pop();
        parse_byte(byte);
    }
}

void Communication::parse_byte(uint8_t byte) {
    switch (state_) {

        case WAIT_START:
            if (byte == START_HEADER) {
                index_ = 0;
                frame_[index_++] = byte;
                state_ = READ_FRAME;
            }
            break;

        case READ_FRAME:
            frame_[index_++] = byte;

            if (index_ >= FRAME_SIZE) {
                process_frame();
                state_ = WAIT_START;
            }
            break;

        default:
            state_ = WAIT_START;
            break;
    }
}

// ================= PROCESSAMENTO =================

void Communication::process_frame() {
    if (frame_[7] != END_HEADER)
        return;

    uint8_t crc_calc = crc8(&frame_[1], 5);

    if (crc_calc != frame_[6])
        return;

    uint8_t id = frame_[1];
    uint8_t* data = &frame_[2];

    handle_message(id, data);
}

// ================= HANDLER =================

void Communication::handle_message(uint8_t id, uint8_t* data) {
    switch (id)
    {
        case SETPOINT_M1: {
            float value;
            memcpy(&value, data, 4);
            motor1->set_control_setpoint(value);
            send(ACK_SETPOINT_M1, data);
            
            break;
        }
        case SETPOINT_M2: {
            float value;
            memcpy(&value, data, 4);
            motor2->set_control_setpoint(value);
            send(ACK_SETPOINT_M2, data);
            
            break;
        }
        case SETPOINT_M3: {
            float value;
            memcpy(&value, data, 4);
            motor3->set_control_setpoint(value);
            send(ACK_SETPOINT_M3, data);
            
            break;
        }
        case SETPOINT_M4: {
            float value;
            memcpy(&value, data, 4);
            motor4->set_control_setpoint(value);
            send(ACK_SETPOINT_M4, data);
            
            break;
        }

        case SETPOINT_ROBOT: {
            float v_linear = float(data[0]) + float(data[1])/100;
            float v_angular = float(data[2]) + float(data[3])/100;

            float c = 0.1;
            float r = 0.06;

            float vL = (v_linear - c*v_angular)/r;
            float vR = (v_linear + c*v_angular)/r;

            motor1->set_control_setpoint(vR);
            motor2->set_control_setpoint(vR);
            motor3->set_control_setpoint(vL);
            motor4->set_control_setpoint(vL);
            
            break;
        }

        case REQUEST_M1: 
        {
            bool enable_broad = data[3] & 0x01;               
            cnt_broad.speed_M1 = enable_broad;
            break;
        }

        case REQUEST_M2: 
        {
            bool enable_broad = data[3] & 0x01;               
            cnt_broad.speed_M2 = enable_broad;
            break;
        }

        case REQUEST_M3: 
        {
            bool enable_broad = data[3] & 0x01;               
            cnt_broad.speed_M3 = enable_broad;
            break;
        }

        case REQUEST_M4: 
        {
            bool enable_broad = data[3] & 0x01;               
            cnt_broad.speed_M4 = enable_broad;
            break;
        }

        case REQUEST_ODOM: 
        {
            bool enable_broad = data[3] & 0x01;               
            cnt_broad.odom_local = enable_broad;
            break;
        }

        default:
            break;
    }
}

// ================= ENVIO =================

void Communication::send(uint8_t id, uint8_t* data) {

    uint8_t frame_send[FRAME_SIZE];

    frame_send[0] = START_HEADER;
    frame_send[1] = id;

    for (int i = 0; i < 4; i++) {
        frame_send[2 + i] = data[i];
    }

    frame_send[6] = crc8(&frame_send[1], 5);
    frame_send[7] = END_HEADER;

    uart_write_blocking(uart_handler_, frame_send, FRAME_SIZE);

    /*
    for(int i=0; i<FRAME_SIZE; i++)
    {
        uart_write_blocking(uart_handler_, &frame_send[i], 1);
        uart_puts(uart_handler_, "\n\r");
    }
    uart_puts(uart_handler_, "----\n\r");
    */
    
}

// ================= BUFFER =================

void Communication::buffer_push(uint8_t byte) {
    uint16_t next = (head + 1) % RX_BUFFER_SIZE;

    if (next != tail) {
        rx_buffer[head] = byte;
        head = next;
    }
}

bool Communication::buffer_available() {
    return head != tail;
}

uint8_t Communication::buffer_pop() {
    uint8_t b = rx_buffer[tail];
    tail = (tail + 1) % RX_BUFFER_SIZE;
    return b;
}

// ================= ISR REAL =================

void Communication::on_uart_rx() 
{
    while (uart_is_readable(uart_handler_)) {
        buffer_push(uart_getc(uart_handler_));
    }
}

// ================= HANDLE_MOTOR =================

void Communication::add_motor(Motor* motor_address, uint8_t ID_MOTOR)
{
    switch (ID_MOTOR)
    {
    case 1:
        motor1 = motor_address;
        break;
    case 2:
        motor2 = motor_address;
        break;
    case 3:
        motor3 = motor_address;
        break;
    case 4:
        motor4 = motor_address;
        break;
    
    default:
        return;
    }
}

void Communication::get_odometry(Odometry* odom)
{
    odometry = odom;
}

void Communication::broadcast_manager()
{
    if(cnt_broad.speed_M1)
    {
        unsigned char bytes[4];
        float value = motor1->get_speed();
        memcpy(bytes, &value, 4);
        send(SPEED_M1, bytes);
    }

    if(cnt_broad.odom_local)
    {
        uint8_t bytes[4];
        float x_pos = odometry->get_x_local();
        float y_pos = odometry->get_y_local();
        float theta_pos = odometry->get_theta_local();
        
        
        bytes[0] = uint8_t(x_pos);
        bytes[1] = uint8_t((x_pos-bytes[0])*100);
        bytes[2] = uint8_t(theta_pos);
        bytes[3] = uint8_t((theta_pos-bytes[2])*100);
        

        send(ODOM_LOCAL, bytes);
    }
}