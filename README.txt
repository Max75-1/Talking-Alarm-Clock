Это проект говорящего будильника. В этом проекте используется SPI3, чтобы считать голосовые файлы с microSD
 * и озвучить их посредством DAC с использованием DMA во время пробуждения по будильнику.
 * Время для RTC задается сперва текущее в main() а затем, также в main() задается время пробуждения.
 * В момент пробуждения срабатывает прерывание RTC-Alarm'a, обработка которого происходит в файле
 * stm32lxx_it.c посредством функций FreeRTOS. До этого момента крутится таск FirstTask, выводящий
 * посекундно время на PuTTY с помощью USART,а в момент прерывания вызывается SoundTask, озвучивающий
 * время пробуждения и выводящий сообщение "ALARM !!! ALARM !!! ALARM !!!" также с помощью USART
 * на экран PuTTY. Прервать звонок будильника можно с помощью голубой кнопки User Button,
 * расположенной на плате Nucleo-L152RE. В этот момент срабатывает другое прерывание, повторно вызывающее
 * FirstTask, обработка которого происходит в stm32lxx_it.c с помощью функций FreeRTOS.
 *
 * Чтобы переназначить время, надо повторно скомпилировать проект. Подсоединение проводов между
 * платой Nucleo-L152RE, платой microSD и колонками смотри на приложенных в папке проекта фото.

This is a project of a talking alarm clock. This project uses SPI3 to read voice files from microSD
 * and have them spoken through the DAC using DMA during the wake-up call.
 * The time for the RTC is set first to the current time in main() and then, also in main(), the wake-up time is set.
 * At the moment of awakening, the RTC-Alarm interrupt is triggered, the processing of which occurs in the file
 * stm32lxx_it.c via FreeRTOS functions. Until this moment, the FirstTask task is working, displaying
 * per second time on PuTTY using USART, and at the moment of interruption, SoundTask is called, voicing
 * wake up time and display the message "ALARM !!! ALARM !!! ALARM !!!" also with USART
 * to the PuTTY screen. You can interrupt the alarm with the blue User Button, located on the Nucleo-L152RE board. At this point, another interrupt is triggered, re-call
 * FirstTask, which is processed in stm32lxx_it.c using FreeRTOS functions.

To reassign the time, you must recompile the project. Connecting wires between
 * Nucleo-L152RE board, microSD board and speakers, see the photos attached in the project folder.