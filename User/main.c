#include "FreeRTOS.h"
#include "app_main.h"
#include "core_riscv.h"
#include "math.h"
#include "task.h"

static void DefaultTask(void* pvParameters)
{
  (void)(pvParameters);
  app_main();
  while (1)
  {
    vTaskDelay(1000);
  }
}

int main(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  SystemInit();
  SystemCoreClockUpdate();
  __enable_irq();
  xTaskCreate(DefaultTask, "DefaultTask", 6000, NULL, 3, NULL);
  vTaskStartScheduler();
  return 0;
}
