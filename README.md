# bare-metal-stm32#include <stdint.h>

/******************************************************************************/
/*                                                                            */
/*                    CLOCK CONFIG. Controller                     */
/*                                                                            */
/******************************************************************************/
#define EXTI_BASE 0x40013C00UL
#define RCC_BASE    0x40023800UL
#define RCC         ((RCC_TypeDef *)  RCC_BASE)
#define AHB1PERIPH_BASE       0x40020000UL
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL) // 0x40020400
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL) // 0x40020800
#define EXTI_BASE             0x40013C00UL
#define EXTI                  ((EXTI_TypeDef *) EXTI_BASE)


/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO register  *****************/

#define GPIOB      ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC      ((GPIO_TypeDef *) GPIOC_BASE)
#define BSRR_SET(pin)    (1UL << (pin))
#define BSRR_RESET(pin)  (1UL << ((pin) + 16))
/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR register  *******************/
#define EXTI_IMR_MR0_Pos          (0U)
#define EXTI_IMR_MR0_Msk          (0x1UL << EXTI_IMR_MR0_Pos)                   /*!< 0x00000001 */
#define EXTI_IMR_MR0              EXTI_IMR_MR0_Msk                             /*!< Interrupt Mask on line 0 */
/******************  Bit definition for EXTI_RTSR register  *******************/
#define EXTI_RTSR_TR0_Pos         (0U)
#define EXTI_RTSR_TR0_Msk         (0x1UL << EXTI_RTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_RTSR_TR0             EXTI_RTSR_TR0_Msk                            /*!< Rising trigger event configuration bit of line 0 */

/******************  Bit definition for EXTI_FTSR register  *******************/
#define EXTI_FTSR_TR0_Pos         (0U)
#define EXTI_FTSR_TR0_Msk         (0x1UL << EXTI_FTSR_TR0_Pos)                  /*!< 0x00000001 */
#define EXTI_FTSR_TR0             EXTI_FTSR_TR0_Msk                            /*!< Falling trigger event configuration bit of line 0 */
/******************  Bit definition for EXTI_SWIER register  ******************/
#define EXTI_SWIER_SWIER0_Pos     (0U)
#define EXTI_SWIER_SWIER0_Msk     (0x1UL << EXTI_SWIER_SWIER0_Pos)              /*!< 0x00000001 */
#define EXTI_SWIER_SWIER0         EXTI_SWIER_SWIER0_Msk                        /*!< Software Interrupt on line 0 */
                    /*!< Software Interrupt on line 4 */
/*******************  Bit definition for EXTI_PR register  ********************/
#define EXTI_PR_PR0_Pos           (0U)
#define EXTI_PR_PR0_Msk           (0x1UL << EXTI_PR_PR0_Pos)                    /*!< 0x00000001 */
#define EXTI_PR_PR0               EXTI_PR_PR0_Msk                              /*!< Pending bit for line 0 */
/* Reference Defines */
#define  EXTI_EMR_EM0                        EXTI_EMR_MR0

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_ID0_Pos                 (0U)
#define GPIO_IDR_ID0_Msk                 (0x1UL << GPIO_IDR_ID0_Pos)            /*!< 0x00000001 */
#define GPIO_IDR_ID0                     GPIO_IDR_ID0_Msk

/* Legacy defines */
#define GPIO_IDR_IDR_0                   GPIO_IDR_ID0


/**
  * @brief External Interrupt/Event Controller
  */
typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
} GPIO_TypeDef;

/**
  * @brief System clock
  */

typedef struct {
    volatile uint32_t CR;            // 0x00
    volatile uint32_t PLLCFGR;       // 0x04
    volatile uint32_t CFGR;          // 0x08
    volatile uint32_t CIR;           // 0x0C
    volatile uint32_t AHB1RSTR;      // 0x10
    volatile uint32_t AHB2RSTR;      // 0x14
    volatile uint32_t AHB3RSTR;      // 0x18
    uint32_t          RESERVED0;     // 0x1C
    volatile uint32_t APB1RSTR;      // 0x20
    volatile uint32_t APB2RSTR;      // 0x24
    uint32_t          RESERVED1[2];  // 0x28
    volatile uint32_t AHB1ENR;       // 0x30
    volatile uint32_t AHB2ENR;       // 0x34
    uint32_t          RESERVED2[2];  // 0x38
    volatile uint32_t APB1ENR;       // 0x40
    volatile uint32_t APB2ENR;       // 0x44  <-- Needed for SYSCFG
} RCC_TypeDef;



/**
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
 volatile uint32_t IMR;
 volatile uint32_t EMR;
 volatile uint32_t RTSR;
 volatile uint32_t FTSR;
 volatile uint32_t SWIER;
 volatile uint32_t PR;
} EXTI_TypeDef;
#define EXTI ((EXTI_TypeDef *) EXTI_BASE)
#define SYSCFG_BASE 0x40013800UL
#define SYSCFG ((SYSCFG_TypeDef*)SYSCFG_BASE)

typedef struct {
 volatile uint32_t MEMRMP;
 volatile uint32_t PMC;
 volatile uint32_t EXTICR[4];
} SYSCFG_TypeDef;



/**
  * @brief GPIO OUTPUT/INPUT
  */
typedef enum
{
    GPIO_MODE_INPUT  = 0x00,
    GPIO_MODE_OUTPUT = 0x01
} GPIO_Mode_t;

/**
  * @brief GPIO OUTPUT/INPUT FUNCTION
  */
void GPIO_SetMode(GPIO_TypeDef *GPIOx, uint8_t pin, GPIO_Mode_t mode)
{
    GPIOx->MODER &= ~(0x3UL << (pin * 2));
    GPIOx->MODER |= ((uint32_t)mode << (pin * 2));
}
void deley()
{
	 for(volatile int i = 0; i < 500000; i++);
}
// Define the callback type
typedef void (*EXTI_Callback_t)(void);

// Global pointer to hold the user function
static EXTI_Callback_t exti0_callback = 0;

// Function to register the callback
void EXTI0_RegisterCallback(EXTI_Callback_t callback) {
    exti0_callback = callback;
}
void EXTI0_IRQHandler(void) {
    // 1. Check if the pending bit for Line 0 is set
    if (EXTI->PR & EXTI_PR_PR0) {

        // 2. Clear the pending bit (In STM32, you write '1' to clear it)
        EXTI->PR = EXTI_PR_PR0;

        // 3. Execute the user callback if it exists
        if (exti0_callback != (void*)0) {
            exti0_callback();
        }
    }
}
void my_button_press_logic(void) {
    // Port B input triggered this.
    // Now toggle LED on Port C Pin 13
	GPIOC->BSRR = BSRR_SET(13);
	deley();
	GPIOC->BSRR = BSRR_RESET(13);

}

void Interrupt_Init(void) {
    // 1. Enable Clocks
    RCC->AHB1ENR |= (1 << 1) | (1 << 2); // Enable Port B and Port C
    RCC->APB2ENR |= (1 << 14);           // Enable SYSCFG Clock

    // 2. Configure GPIO
    GPIOB->PUPDR &= ~(3<<(0*2));
    GPIOB->PUPDR |=  (1<<(0*2));   // pull-up
    GPIO_SetMode(GPIOB, 0, GPIO_MODE_INPUT);  // PB0 as Input
    GPIO_SetMode(GPIOC, 13, GPIO_MODE_OUTPUT); // PC13 as LED Output

    // 3. Map EXTI0 to PORT B
    //EXTICR[0] bits 3:0;
    SYSCFG->EXTICR[0] &= ~(0xF << 0);
   SYSCFG->EXTICR[0] |= (0x1 << 0);

    // 4. Configure EXTI Line 0
    EXTI->IMR  |= EXTI_IMR_MR0;
    EXTI->RTSR |= EXTI_RTSR_TR0;

    // 5. Register Callback and Enable NVIC
    EXTI0_RegisterCallback(my_button_press_logic);

    // NVIC Enable for EXTI0 (Interrupt #6)
    *((volatile uint32_t *)(0xE000E100)) |= (1 << 6);
}
int main()
{

	Interrupt_Init();
	 while(1);
}
