# Seeed-LoRa-E5 - Ground water sensor application

When working with this project, we highly recommend using git, so that you can revert the changes made by the automatic code generator.

- UART2
    - Despite being configured in the `.ioc`, it is out of commission, as the first prototype PCB connects the UART pins to ground.
    
- When generating files, ignore all changes that are due to automatic code generation in the following files:
    - `Core\Src\main.c` (comment out in `main`: `MX_LoRaWAN_Process();`)
    - `Core\Src\usart_if.c` (comment out in `vcom_Resume`:)
        ```
        if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
        {
            Error_Handler();
        }
        ```
    - `LoRaWAN\App\app_lorawan.c` (comment out in `MX_LoRaWAN_Init`: `SystemApp_Init();`)
    - `LoRaWAN\App\lora_app.c`
