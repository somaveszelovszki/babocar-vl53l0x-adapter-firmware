#include <cfg_board.hpp>

// INTERRUPT CALLBACKS - Must be defined in a task's source file!

extern void micro_PanelLink_Uart_RxCpltCallback();

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == uart_PanelLink.handle) {
        micro_PanelLink_Uart_RxCpltCallback();
    }
}
