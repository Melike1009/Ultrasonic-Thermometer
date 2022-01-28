
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <time.h>

// XDCtools Header files
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* TI-RTOS Header files */
#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Idle.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/GPIO.h>
#include <ti/net/http/httpcli.h>
#include <ti/drivers/I2C.h>
#include <math.h>

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "Board.h"

#include <sys/socket.h>
#include <arpa/inet.h>
//-----new libraries for ultrasonic sensor------
#include "inc/hw_ints.h"
#include "driverlib/interrupt.c"
#include "driverlib/sysctl.c"
#include "driverlib/Timer.c"
#include "driverlib/gpio.c"
#include"inc/tm4c1294ncpdt.h"

#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"

#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/udma.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include <string.h>

void inputInt();
void Captureinit();

#define USER_AGENT        "HTTPCli (ARM; TI-RTOS)"
#define SOCKETTEST_IP     "192.168.1.105"
#define TIME_IP           "128.138.140.44"

#define OUTGOING_PORT     5011
#define INCOMING_PORT     5030
#define TASKSTACKSIZE     4096

extern Semaphore_Handle semaphore1, semaphore2, semaphore3, semaphore4;
extern Swi_Handle swi0;
extern Mailbox_Handle mailbox0;
extern Event_Handle event0;

char temperature[1000];
int realtime;
char takenTime[32];
uint32_t ctr=0 ;

Void Timer_ISR(UArg arg1)
{
    Semaphore_post(semaphore1);
    Semaphore_post(semaphore2);
}

Void TIME_CALC()
{
    while(1){
        Semaphore_pend(semaphore1, BIOS_WAIT_FOREVER);
        realtime  = takenTime[0]*16777216 +  takenTime[1]*65536 + takenTime[2]*256 + takenTime[3];
        realtime += 10800;
        realtime += ctr++;
    }
}

//This is to avoid doing the math everytime you do a reading
const double temp = 1.0/80.0; // 1MHZ olmasi icin asagida 80MHZ ile calistiriliyor

//Stores the pulse length
volatile uint32_t pulse_A2=0; //falling edge zamani  --total time= falling edge-risingedge

//Stores the rising edge time
volatile uint32_t Port_A_Pin_2_Rising = 0;//rising edge zamani

Void Loop(UArg arg1, UArg arg2){

    while(1)
    {
    //Does the required pulse of 10uS
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);//trigger on
    SysCtlDelay(266);//10us delay
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ~GPIO_PIN_3);//trigger off
    while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) != GPIO_PIN_2);//wait for echo
    if ( GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == GPIO_PIN_2){ //echo on
        Port_A_Pin_2_Rising = TimerValueGet(TIMER2_BASE,TIMER_A);
        }
    while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == GPIO_PIN_2); //wait for echo off
        pulse_A2 = TimerValueGet(TIMER2_BASE,TIMER_A);
        if ( pulse_A2 < Port_A_Pin_2_Rising ) pulse_A2 = pulse_A2 + (0xffffffff -1);
        pulse_A2 = pulse_A2 - Port_A_Pin_2_Rising;

    //Converts the counter value to cm.
    pulse_A2 =(uint32_t)(temp * pulse_A2);
    pulse_A2 = pulse_A2 / 58;
    if(pulse_A2 <= 50){
        if(pulse_A2 <= 10){
        sprintf(temperature,"Person is within 10cm \n");
        printf("Person is within 10cm \n");
        Semaphore_post(semaphore3);//start measuring temperature
        }
        else if(pulse_A2 <= 30){
            sprintf(temperature,"Person is within 30cm \n");
        printf("Person is within 30cm \n");
        }
        else{
            sprintf(temperature,"Person detected with ultrasonic range \n");
            printf("Person detected with ultrasonic range\n");
        }
        Event_post(event0, Event_Id_01); // Hercules writing
    }
    //wait about 1000ms until the next reading.
    SysCtlDelay(40000000);
    }
}
    void Captureinit(){ // Timer settings
    /*
    Set the timer to be periodic.
    */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlDelay(3);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);
    TimerEnable(TIMER2_BASE,TIMER_A);
    }

Void TempIn(UArg arg1, UArg arg2){

        Semaphore_pend(semaphore4, BIOS_WAIT_FOREVER);
        uint8_t        temperature;
        uint8_t         txBuffer[1];
        uint16_t         rxBuffer[2];
        I2C_Handle      i2c;
        I2C_Params      i2cParams;
        I2C_Transaction i2cTransaction;

        /* Create I2C for usage */
        I2C_Params_init(&i2cParams);
        i2cParams.bitRate = I2C_100kHz;

        while(1){
        i2c = I2C_open( Board_I2C_TMP , &i2cParams);

        if (i2c == NULL) {
            System_abort("Error Initializing I2C\n");
            System_flush();
        }
        txBuffer[0] = 0x01;
        i2cTransaction.slaveAddress = 0x5A ;
        i2cTransaction.writeBuf = txBuffer;
        i2cTransaction.writeCount = 1;
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = 2;
            if (((I2C_transfer(i2c, &i2cTransaction))!=0)&&((rxBuffer[0]-145)>30)) {
                temperature = (rxBuffer[0]-145);
                Mailbox_post(mailbox0, &temperature, BIOS_NO_WAIT);
                I2C_close(i2c);
            }
            else{
                I2C_close(i2c);
            }
            I2C_close(i2c);
        }
}
Void tempGet (UArg arg1, UArg arg2){
    while(1){
                Semaphore_pend(semaphore3, BIOS_WAIT_FOREVER);
                uint8_t tempVal;
                uint8_t tempAverage = 0;
                uint16_t totalTemp=0;
                float sendThisTemp = 0;
                int i;
                    for (i = 0; i < 100; i++) {
                    Semaphore_post(semaphore4);
                    Mailbox_pend(mailbox0, &tempVal, BIOS_WAIT_FOREVER);
                    if(tempVal > 50){
                        i--;  // prevent fluctuation. remeasure temperature above 50 degrees
                    }
                    else{
                    totalTemp = totalTemp + tempVal;
                    }
                }
                tempAverage = totalTemp /100;
                sendThisTemp = (float)(tempAverage);
                if (sendThisTemp > 36.7){
                    sprintf(temperature,"Triggering temperature measurement \n CAUTION! Temperature is %2.1f: \n",sendThisTemp);
                    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3); // Red led on
                }
                else {
                    sprintf(temperature,"Triggering temperature measurement \n Temperature is %2.1f: \n",sendThisTemp); // Send to hercules
                    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 0); // Red led off
                }
                printf("\tTemperature measured is %2.1f C \n",sendThisTemp); // This program
                Event_post(event0, Event_Id_00);
                }
}
/*
 *  ======== printError ========
 */
void printError(char *errString, int code)
{
    System_printf("Error! code = %d, desc = %s\n", code, errString);
    BIOS_exit(code);
}
bool sendData2Server(char *serverIP, int serverPort, char *data, int size)
{
    int sockfd, connStat, numSend;
    bool retval=false;
    struct sockaddr_in serverAddr;

    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd == -1) {
        System_printf("Socket not created");
        close(sockfd);
        return false;
    }

    memset(&serverAddr, 0, sizeof(serverAddr));  // clear serverAddr structure
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);     // convert port # to network order
    inet_pton(AF_INET, serverIP, &(serverAddr.sin_addr));

    connStat = connect(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if(connStat < 0) {
        System_printf("sendData2Server::Error while connecting to server\n");
    }
    else {
        numSend = send(sockfd, data, size, 0);       // send data to the server
        if(numSend < 0) {
            System_printf("sendData2Server::Error while sending data to server\n");
        }
        else {
            retval = true;
        }
    }
    System_flush();
    close(sockfd);
    return retval;
}
Void clientSocketTask()
{
    char time[64];
    while(1) {
        // wait for the event that httpTask() will signal

        Event_pend(event0, Event_Id_NONE,Event_Id_00 + Event_Id_01, BIOS_WAIT_FOREVER);
          sprintf(time, "%s", ctime(&realtime));
          strcat(temperature, time); // concatenate temp and time
     //   GPIO_write(Board_LED0, 1); // turn on the LED

        // connect to SocketTest program on the system with given IP/port

        if(sendData2Server(SOCKETTEST_IP, OUTGOING_PORT, temperature, strlen(temperature))) {
        }

     //   GPIO_write(Board_LED0, 0);  // turn off the LED
    }
}
void recvTimeStamptFromNTP(char *serverIP, int serverPort, char *data, int size)
{
        System_printf("recvTimeStamptFromNTP start\n");
        System_flush();

        int sockfd, connStat, tri;
        struct sockaddr_in serverAddr;

        sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sockfd == -1) {
            System_printf("Socket not created");
            BIOS_exit(-1);
        }
        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(37);
        inet_pton(AF_INET, serverIP , &(serverAddr.sin_addr));

        connStat = connect(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
        if(connStat < 0) {
            System_printf("sendData2Server::Error while connecting to server\n");
            if(sockfd>0) close(sockfd);
            BIOS_exit(-1);
        }

        tri = recv(sockfd, takenTime, sizeof(takenTime), 0);
        if(tri < 0) {
            System_printf("Error while receiving data from server\n");
            if (sockfd > 0) close(sockfd);
            BIOS_exit(-1);
        }
        if (sockfd > 0) close(sockfd);
}
Void socketTask(){

        Semaphore_pend(semaphore2, BIOS_WAIT_FOREVER);

    //    GPIO_write(Board_LED0, 1);

        recvTimeStamptFromNTP(TIME_IP, 37,realtime, strlen(realtime));

    //    GPIO_write(Board_LED0, 0);
}
bool createTasks(void)
{
    static Task_Handle taskHandle1,taskHandle2, taskHandle3;
    Task_Params taskParams;
    Error_Block eb;

    Error_init(&eb);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 1;
    taskHandle2 = Task_create((Task_FuncPtr)clientSocketTask, &taskParams, &eb);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 1;
    taskHandle1 = Task_create((Task_FuncPtr)socketTask, &taskParams, &eb);

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 1;
    taskHandle3 = Task_create((Task_FuncPtr)TIME_CALC, &taskParams, &eb);

    if (taskHandle1 == NULL || taskHandle2 == NULL || taskHandle3 == NULL) {
        printError("netIPAddrHook: Failed to create HTTP, Socket and Server Tasks\n", -1);
        return false;
    }
    return true;
}
//  This function is called when IP Addr is added or deleted
//
void netIPAddrHook(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{
    // Create a HTTP task when the IP address is added
    if (fAdd) {
        createTasks();
    }
}

int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initEMAC();
    Board_initI2C();

    //Set system clock to 80Mhz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    //Configures the timer
    Captureinit();

    //Configure Trigger pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(3);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);

    //Configure Echo pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(3);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2,GPIO_BOTH_EDGES);

    //GPIO for red led
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlDelay(3);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);

    /* Start BIOS */
    BIOS_start();

    return (0);
}
