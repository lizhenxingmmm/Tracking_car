#include "drv_uart.h"

//默认所有uart接收内存为2x200
#define UART_RX_BUF_LEN (200)

//开辟uart接收双缓冲区
volatile uint8_t uart1_rx_buf[2][UART_RX_BUF_LEN] = {0};
volatile uint8_t uart2_rx_buf[2][UART_RX_BUF_LEN] = {0};
volatile uint8_t uart3_rx_buf[2][UART_RX_BUF_LEN] = {0};
volatile uint8_t uart4_rx_buf[2][UART_RX_BUF_LEN] = {0};
volatile uint8_t uart5_rx_buf[2][UART_RX_BUF_LEN] = {0};
volatile uint8_t uart6_rx_buf[2][UART_RX_BUF_LEN] = {0};

// uart接收数据包有效长度
int uart1_rx_data_frame_len;
int uart2_rx_data_frame_len;
int uart3_rx_data_frame_len;
int uart4_rx_data_frame_len;
int uart5_rx_data_frame_len;
int uart6_rx_data_frame_len;

static void uart_rx_idle_callback(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma,
                                                     uint32_t SrcAddress,
                                                     uint32_t DstAddress,
                                                     uint32_t SecondMemAddress,
                                                     uint32_t DataLength);
/**
 * @brief uart初始化dma双缓冲接收,开启空闲中断
 */
void USART_Init(UART_HandleTypeDef *huart) //开启空闲中断，配置DMA相关参数，使能DMA接收
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);          //先清楚空闲中断标志位，防止开启中断时立马进入中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); //使能空闲中断

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR); //将串口对应的DMA打开
    if (huart->Instance == USART1)
    {
        DMAEx_MultiBufferStart_NoIT(huart->hdmarx,
                                    (uint32_t)&huart->Instance->DR,
                                    (uint32_t)uart1_rx_buf[0],
                                    (uint32_t)uart1_rx_buf[1],
                                    UART_RX_BUF_LEN);
    }
    if (huart->Instance == USART2)
    {
        DMAEx_MultiBufferStart_NoIT(huart->hdmarx,
                                    (uint32_t)&huart->Instance->DR,
                                    (uint32_t)uart2_rx_buf[0],
                                    (uint32_t)uart2_rx_buf[1],
                                    UART_RX_BUF_LEN);
    }
    if (huart->Instance == USART3)
    {
        DMAEx_MultiBufferStart_NoIT(huart->hdmarx,
                                    (uint32_t)&huart->Instance->DR,
                                    (uint32_t)uart3_rx_buf[0],
                                    (uint32_t)uart3_rx_buf[1],
                                    UART_RX_BUF_LEN);
    }
    if (huart->Instance == UART4)
    {
        DMAEx_MultiBufferStart_NoIT(huart->hdmarx,
                                    (uint32_t)&huart->Instance->DR,
                                    (uint32_t)uart4_rx_buf[0],
                                    (uint32_t)uart4_rx_buf[1],
                                    UART_RX_BUF_LEN);
    }
    if (huart->Instance == UART5)
    {
        DMAEx_MultiBufferStart_NoIT(huart->hdmarx,
                                    (uint32_t)&huart->Instance->DR,
                                    (uint32_t)uart5_rx_buf[0],
                                    (uint32_t)uart5_rx_buf[1],
                                    UART_RX_BUF_LEN);
    }
    if (huart->Instance == USART6)
    {
        DMAEx_MultiBufferStart_NoIT(huart->hdmarx,
                                    (uint32_t)&huart->Instance->DR,
                                    (uint32_t)uart6_rx_buf[0],
                                    (uint32_t)uart6_rx_buf[1],
                                    UART_RX_BUF_LEN);
    }
}
/**
 * @brief 中断请求函数
 * @note 在stm32f4xx_it.c文件调用，如在USART6_IRQHandler函数中调用
 */
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
        __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE)) //判断是否为空闲中断
    {
        uart_rx_idle_callback(huart);
    }
}

/**
 * @brief dma双缓冲内存0搬运完成回调函数，并将目标内存指定为内存1
 * @note 并不是接收完成系统自动调用，在空闲中断中手动调用
 */
static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
    hdma->Instance->CR |= (uint32_t)(DMA_SxCR_CT); // 将当前目标内存设置为Memory1
                                                   /*处理数据*/

    /*处理数据*也可以不在这里处理*/
}
/**
 * @brief dma双缓冲内存1搬运完成回调函数，并将目标内存指定为内存0
 * @note 并不是接收完成系统自动调用，在空闲中断中手动调用
 */
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
    hdma->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT); // 将当前目标内存设置为Memory0
                                                    /*处理数据*/

    /*处理数据*也可以不在这里处理*/
}
/**
 * @brief uart接收中断回调函数
 */
static void uart_rx_idle_callback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_DMA_DISABLE(huart->hdmarx);
    //计算接收数据长度
    if (huart->Instance == USART1)
    {
        uart1_rx_data_frame_len = UART_RX_BUF_LEN - huart->hdmarx->Instance->NDTR;
    }
    if (huart->Instance == USART2)
    {
        uart2_rx_data_frame_len = UART_RX_BUF_LEN - huart->hdmarx->Instance->NDTR;
    }
    if (huart->Instance == USART3)
    {
        uart3_rx_data_frame_len = UART_RX_BUF_LEN - huart->hdmarx->Instance->NDTR;
    }
    if (huart->Instance == UART4)
    {
        uart4_rx_data_frame_len = UART_RX_BUF_LEN - huart->hdmarx->Instance->NDTR;
    }
    if (huart->Instance == UART5)
    {
        uart5_rx_data_frame_len = UART_RX_BUF_LEN - huart->hdmarx->Instance->NDTR;
    }
    if (huart->Instance == USART6)
    {
        uart6_rx_data_frame_len = UART_RX_BUF_LEN - huart->hdmarx->Instance->NDTR;
    }
    //判断接收内存区域
    //并且在执行对应内存区域的回调函数dma失能，不会出现接收错乱导致程序跑死的问题
    if (huart->hdmarx->Instance->CR & DMA_SxCR_CT)
        huart->hdmarx->XferM1CpltCallback(huart->hdmarx);
    else
        huart->hdmarx->XferCpltCallback(huart->hdmarx);
    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, UART_RX_BUF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma,
                                                     uint32_t SrcAddress,
                                                     uint32_t DstAddress,
                                                     uint32_t SecondMemAddress,
                                                     uint32_t DataLength) // HAL库stm32f4xx_hal_dma_ex.c中有个类似函数HAL_DMAEx_MultiBufferStart_IT,这里做了修改，关闭了DMA的中断
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Memory-to-memory transfer not supported in double buffering mode */
    if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
    {
        hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
        return HAL_ERROR;
    }

    /* Set the UART DMA transfer complete callback */
    /* Current memory buffer used is Memory 1 callback */
    // dma_m0_rxcplt_callback内存0搬运完成的回调函数
    hdma->XferCpltCallback = dma_m0_rxcplt_callback;
    /* Current memory buffer used is Memory 0 callback */
    // dma_m1_rxcplt_callback内存1搬运完成的回调函数
    hdma->XferM1CpltCallback = dma_m1_rxcplt_callback;

    /* Check callback functions */
    if ((NULL == hdma->XferCpltCallback) || (NULL == hdma->XferM1CpltCallback))
    {
        hdma->ErrorCode = HAL_DMA_ERROR_PARAM;
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(hdma);

    if (HAL_DMA_STATE_READY == hdma->State)
    {
        /* Change DMA peripheral state */
        hdma->State = HAL_DMA_STATE_BUSY;

        /* Initialize the error code */
        hdma->ErrorCode = HAL_DMA_ERROR_NONE;

        /* Enable the Double buffer mode */
        hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;

        /* Configure DMA Stream destination address */
        hdma->Instance->M1AR = SecondMemAddress;

        /* Configure DMA Stream data length */
        hdma->Instance->NDTR = DataLength;

        /* Peripheral to Memory */
        if ((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
        {
            /* Configure DMA Stream destination address */
            hdma->Instance->PAR = DstAddress;

            /* Configure DMA Stream source address */
            hdma->Instance->M0AR = SrcAddress;
        }
        /* Memory to Peripheral */
        else
        {
            /* Configure DMA Stream source address */
            hdma->Instance->PAR = SrcAddress;

            /* Configure DMA Stream destination address */
            hdma->Instance->M0AR = DstAddress;
        }

        /* Clear TC flags */
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
        /* Enable TC interrupts*/
        // hdma->Instance->CR  |= DMA_IT_TC;
        //使能中断被注释
        /* Enable the peripheral */
        __HAL_DMA_ENABLE(hdma);
    }
    else
    {
        /* Process unlocked */
        __HAL_UNLOCK(hdma);

        /* Return error status */
        status = HAL_BUSY;
    }
    /* Process unlocked */
    __HAL_UNLOCK(hdma);

    return status;
}
