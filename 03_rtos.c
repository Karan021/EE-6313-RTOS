// RTOS Framework - Spring 2019
// J Losh

// Student Name:Karan Gandhi
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Green:  PE2
// Yellow: PE3
// Orange: PE4
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // off-board orange LED
#define PB0_PA2      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define PB1_PA3      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) // off-board red LED
#define PB2_PA4      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) // off-board green LED
#define PB3_PA5      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) // off-board yellow LED
#define PB4_PA6      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) // off-board orange LED

//-----------------------------------------------------------------------------
// Defines and Kernel Variables
//-----------------------------------------------------------------------------

#define YIELD __asm(" SVC #100")
#define SLEEP __asm(" SVC #101")
#define WAIT  __asm(" SVC #102")
#define POST  __asm(" SVC #103")
#define DELETE_THREAD __asm(" SVC #104")
#define SET_PRIORITY __asm(" SVC #105")
#define UPDATE_PERCENT __asm(" SVC #106")
#define SYS_YIELD 'd'
#define SYS_SLEEP 'e'
#define SYS_WAIT 'f'
#define SYS_POST 'g'
#define SYS_SET_PRIORITY 'i'
#define SYS_DELETE_THREAD 'h'
#define SYS_UPDATE 'j'
#define Highest_priority -8
uint16_t s=0;
uint16_t sem=0;
uint16_t count=0;
#define MAX_CHAR 80
#define MAX_ARG 5
char str[MAX_CHAR+1];
char buff[MAX_CHAR+1];
char history[10][MAX_CHAR+1];
uint8_t argcount=0;
uint8_t pos[MAX_ARG];
char type[MAX_ARG];
uint32_t systemSP;
uint16_t skipcount[10];
uint16_t load[10];
uint8_t j=0,i=0xFF,d=0;
uint16_t waitlog[5] = {0,0,0,0,0};
bool PI=true;
bool pre=true;
bool PS=true;
char n;
uint32_t time[10];
uint32_t T=0;
uint16_t timecount=0;
uint16_t timecount2=0;
uint16_t tp=0;
uint16_t hist=0;

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char name[16];
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread

struct _tcb
{
    uint32_t percent[5];
    uint32_t time;
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // -8=highest to 7=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void rtosInit()
{

    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;                                                       //Clearing all the tasks by making them invalid and the PID zero
        tcb[i].pid = 0;
    }
    // REQUIRED: initialize systick for 1ms system timer
    NVIC_ST_RELOAD_R = 0X00009C3F;                                                          // Reload value decimal 39999 to make system timer 1ms
    NVIC_ST_CURRENT_R = 0X01;                                                               // Making counter value 1 to clear the register
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN |NVIC_ST_CTRL_ENABLE;       // Using system clock, enabling the interrupts and enabling the systick timer

}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {

        if(PS)                                                                             //Priority Scheduling bit high
        {
            task++;
            if (task >= taskCount)
                task = 0;                                                                   // if task count exceeds max tasks get task back to zero

            if(skipcount[task]==0 && (tcb[task].state==STATE_READY || tcb[task].state==STATE_UNRUN )) //checking skip count for each task and checking its state if its ready or un-run
            {
                skipcount[task]=tcb[task].currentPriority + 8;                                          //Adding Bias to make the negative priority positive for skip count
                ok=true;
            }
            else if((tcb[task].state==STATE_READY || tcb[task].state==STATE_UNRUN ) && skipcount[task]!=0) //Only decrement skip count if it is non-zero in order to keep the skip count within range
            {
                skipcount[task]--;
                ok=false;

            }
        }
        else                                                                             //Round robin scheduling
        {
            task++;
            if (task >= taskCount)
                task = 0;

            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
    }

    return task;
}
//-----------------------------------------------------------------------------
// Stack Pointer Functions
//-----------------------------------------------------------------------------
void setSP(void* S)
{
    __asm(" MOV SP, R0");                                                                   //Moving register R0 to SP
}

uint32_t getSP()
{
    __asm(" MOV R0,SP");                                                                    //Moving register SP to R0
}
//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------
void rtosStart()
{
    // REQUIRED: add code to call the first task to be run
    systemSP=getSP();                                                                       //Saving the system stack pointer
    _fn fn;
    taskCurrent = rtosScheduler();                                                          //scheduling the first task
    // Add code to initialize the SP with tcb[task_current].sp;
    setSP((void*)(tcb[taskCurrent].sp));                                                    //setting the stack pointer to the task stack
    tcb[taskCurrent].state = STATE_READY;                                                   //Making the task state ready to run

    fn =(_fn)(tcb[taskCurrent].pid);                                                        //giving the task address to run the task
    tcb[taskCurrent].time++;                                                                //Incrementing the number of times the current task is being scheduled
    (*fn)();                                                                                // running the first task

}

bool createThread(_fn fn, char name[], int priority)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED: store the thread name
    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);                                                   //making sure if the task was already in the tcb directory
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}                                    //checking if the task that wasnt found if any state is invalid
            tcb[i].state = STATE_UNRUN;                                                     //Making the task state un-run
            tcb[i].pid = fn;                                                                //Setting the function to PID
            tcb[i].sp = &stack[i][255];                                                     //initializing the task stack of size 1024
            tcb[i].priority = priority;                                                     //setting the task base priority
            tcb[i].currentPriority = priority;                                              //setting the current priority to base priority
            // increment task count
            skipcount[i]=tcb[i].currentPriority + 8;                                        //initializing the skipcount with Bias to make negative priority to positive count
            for(j=0;j<16;j++)
            {
                if(name[j]==0)
                {
                    tcb[i].name[j]='\0';
                    break;
                }
                tcb[i].name[j]=tolower(name[j]);                                             //storing the name of the task in the tcb structure directory
            }
            taskCount++;                                                                     //incrementing the task count to store the next task

            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    DELETE_THREAD;                                                                              //SVC call for deleting a thread
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    SET_PRIORITY;                                                                               //SVC call for setting the priority
}

struct semaphore* createSemaphore(uint8_t count, char name[])
{
    struct semaphore *pSemaphore = 0;                                                           //Creating the structure pointer
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];                                             //storing the address of semaphore to the pointer
        pSemaphore->count = count;                                                              //Updating the semaphore count to the struct pointer
        for(j=0;j<16;j++)
        {
            if(name[j]==0)
            {
                pSemaphore->name[j]='\0';
                break;
            }
            pSemaphore->name[j]=name[j];                                                        //Storing the semaphore name in the semaphore structure directory
        }
    }
    return pSemaphore;                                                                          //Returning the semaphore structure pointer for that specific semaphore
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
    YIELD;                                                                                      //SVC call for kernel function yield to change to another task

}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    SLEEP;                                                                                      //SVC call for kernel function sleep where the task allows others to run while its in sleep mode

}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    WAIT;                                                                                       //SVC call for Wait kernel function where the task accesses the semaphore or gets in queue if
                                                                                                //semaphore already in use

}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    POST;                                                                                       //SVC call for Post/signal kernel function where the task using the semaphore releases it for
}                                                                                               // other tasks to run

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t i;
    timecount++;
    timecount2++;
    uint32_t temp = 0;                                                                         //initializing variables for different timers and CPU calculation done later in the code
    for (i = 0; i < MAX_TASKS; i++)
    {
        if(tcb[i].state == STATE_DELAYED)                                                      //For all tasks it checks for which task has delayed state ie its in sleep mode
        {
            if(tcb[i].ticks>0)                                                                 //Checks if the ticks or the amount of time it went to sleep mode is now zero
            {
                tcb[i].ticks --;
            }
            if(tcb[i].ticks == 0)
            {
                tcb[i].state = STATE_READY;                                                    //If the task ticks do become zero the task is made to ready state again to let it run once again
            }
        }
    }
    if(timecount==1000)                                                                        //After every 1second all the task schedule times are added togther
    {
        for(i=0;i<taskCount;i++)
        {

            T+=time[i];
        }
        for(i=0;i<taskCount;i++)
        {
            temp = (time[i]*100000)/T;                                                         //Each task average time is divided by the total time taken by all the tasks to calculate
            for(j=5;j>0;j--)                                                                   //CPU percentage of each task in 1 second
            {
                tcb[i].percent[j]=temp%10;                                                     //Storing each digit to tcb structure of each task
                temp=temp/10;
                if(temp==0)
                {
                    break;
                }
            }
            tcb[i].time=0;                                                                     //Resetting all the times back for new update the next second
            time[i]=0;
        }
        timecount=0;;
        T=0;
    }
    if(timecount2==250)
    {
        for(i=0;i<taskCount;i++)
        {
            time[i]=(time[i]*9+1*tcb[i].time);                                                 //Taking the moving average every 250ms
        }
        timecount2=0;
    }
    if(pre)
    {
        bool check = false;
        for(i=0;i<taskCount;i++)
        {
            if(tcb[i].state==STATE_READY)
            {
                check = true;                                                                  //Check if any task state ready then and then only will the pendSV bit be set for preemption
            }
        }
        if(check)
        {
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;                                              //If preemption is turned on PendSV bit is set for task switching
        }
    }
}
//-----------------------------------------------------------------------------
// Uart Printing Functions
//-----------------------------------------------------------------------------
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);                                                         //blocking function to print a character
    UART0_DR_R = c;
}

void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)                                                          //to printout each character in the string
        putcUart0(str[i]);
}
//-----------------------------------------------------------------------------
// Task Switching
//-----------------------------------------------------------------------------
// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()                                                                               //For Task switching
{
    __asm(" PUSH {R4,R5,R6,R7}");                                                              //Pushing the R4,R5,R6,R7 registers into the stack as the ISR already pushed the other registers
    tcb[taskCurrent].sp =(void*)getSP();                                                       //Saving the stack pointer of the task
    setSP((void*)(systemSP));                                                                  //Setting the stack pointer back to the system stack
    taskCurrent = rtosScheduler();                                                             //Scheduling the next task
    tcb[taskCurrent].time++;                                                                   //Incrementing the task schedule time
    if(tcb[taskCurrent].state == STATE_READY)
    {
        setSP(tcb[taskCurrent].sp);                                                            //Checks if task ready, if ready set the stack pointer to task stack
        __asm(" sub.w SP, SP , #8");
    }
    else if(tcb[taskCurrent].state == STATE_UNRUN)                                             //If task in un-run state, seed the task stack to make it look like it was interrupted
    {
        tcb[taskCurrent].state = STATE_READY;                                                  //Setting the task state to ready to make the task run
        stack[taskCurrent][255] = 0x41000000;                                                  //Filling the stack with all the registers (R0-R12, LR, PC, PSR)
        stack[taskCurrent][254] =(uint32_t)tcb[taskCurrent].pid;                               //Set address of function in the tcb PID to PC in the stack
        stack[taskCurrent][253] = 0x00000000;
        stack[taskCurrent][252] = 0x00000012;
        stack[taskCurrent][251] = 0x00000003;
        stack[taskCurrent][250] = 0x00000002;
        stack[taskCurrent][249] = 0x00000001;
        stack[taskCurrent][248] = 0x00000000;
        stack[taskCurrent][247] = 0xFFFFFFF9;                                                   //Seeding LR to FFFFFFF9 so as to make it look like it was interrupted by an ISR
        stack[taskCurrent][246] = 0x00000003;
        stack[taskCurrent][245] = 0x00000007;
        stack[taskCurrent][244] = 0x00000006;
        stack[taskCurrent][243] = 0x00000005;
        stack[taskCurrent][242] = 0x00000004;
        setSP(tcb[taskCurrent].sp);
        __asm(" sub.w sp ,sp , #60");                                                           //Getting the Stack pointer to the top of the stack
    }
    __asm(" POP {R4,R5,R6,R7}");                                                                //Poping the saved registers or the seeded registers to switch the task
    __asm(" POP {R3,LR}");
    __asm(" BX LR");
}
//-----------------------------------------------------------------------------
// Functions to Store Registers and SVC number
//-----------------------------------------------------------------------------
uint8_t getSVC()
{
    __asm(" LDR R1,[SP,#0x40]");                                                                //Getting the stack pointer to the location of the pushed PC
    __asm(" LDR R0,[R1,#-0x2]");                                                                //Moving the PC by 2 to get the SVC intruction
    __asm(" BIC R0,R0,#0XFF00");                                                                // Anding the instruction with FF00 to get the SVC number from the instruction
}

uint32_t getR0()
{
    __asm(" MOV R0,R0");                                                                         //saving the R0 register value
}

uint32_t getR1()
{
    __asm(" MOV R0,R1");                                                                         //saving the R1 register value
}

uint32_t getR2()
{
    __asm(" MOV R0,R2");                                                                         //Saving the R2 register value
}

//-----------------------------------------------------------------------------
// Service Call ISR
//-----------------------------------------------------------------------------
// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    uint32_t R0 = getR0();
    uint32_t R1 = getR1();
    uint32_t R2 = getR2();
    uint8_t SVCN = (int)getSVC();                                                           //Saving the R0,R1,R2 registers and getting the SVC call number from instruction for service switching
    switch(SVCN)
    {
    case SYS_YIELD:
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;                                           //Yield kernel function service sets PendSV bit for task switching
        break;
    case SYS_SLEEP:
        tcb[taskCurrent].ticks = R0;                                                        //Loads the R0 value to tcb ticks to calculate the length of task sleep
        tcb[taskCurrent].state = STATE_DELAYED;                                             //Make the task state delayed so that it doesn't get scheduled till its state is ready
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;                                           //set PendSV bit to allow task switching
        break;
    case SYS_WAIT:
        for(sem=0;sem<MAX_SEMAPHORES;sem++)
        {
            if(&(semaphores[sem].count)==R0)
            {
                s=sem;                                                                      //Checks the address of each semaphore with the address passed via the SVC call and selects the
                break;                                                                      //the right semaphore
            }
        }
        if(semaphores[s].count > 0)                                                         //Checks if the requested semaphore count is greater than Zero
        {
            if(PI && tcb[waitlog[s]].currentPriority > tcb[taskCurrent].currentPriority)    //For Priority Inheritance, checks the bit and checks if the task requesting the semaphore
            {                                                                               //has higher priority than the task that last had the semaphore
                if(waitlog[s]!=0)
                {
                tcb[waitlog[s]].currentPriority = tcb[taskCurrent].currentPriority;         //If the above condition is true then change the priority of the last user to the higher priority
                }
                semaphores[s].count--;                                                      //of the 2 tasks and decrement the semaphore count
                waitlog[s]=taskCurrent;                                                     //Storing the current task in waitlog variable to keep track of the semaphore users
                break;
            }
            else
            {
                semaphores[s].count--;
                waitlog[s]=taskCurrent;
                break;
            }
        }
        else                                                                                //Checks the count of the semaphore is less than one or Zero
        {
            if(PI && tcb[waitlog[s]].currentPriority > tcb[taskCurrent].currentPriority)
            {
                if(waitlog[s]!=0)                                                           //Checking if there is task in the wait log else do not change the priority
                {
                tcb[waitlog[s]].currentPriority = tcb[taskCurrent].currentPriority;
                }
                semaphores[s].processQueue[semaphores[s].queueSize]=taskCurrent;            //Adding the current task requesting the semaphore to process queue of the semaphore
                semaphores[s].queueSize++;                                                  //Incrementing the semaphore queue size after adding the task
                tcb[taskCurrent].state = STATE_BLOCKED;                                     //Making the task state blocked so as to prevent it from being scheduled
                tcb[taskCurrent].semaphore =(void *)R0;                                     //Store the semaphore blocking the function in the tcb table
                waitlog[s]=taskCurrent;
                NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;                                   //Set PendSV bit to allow task switch
                break;
            }
            else
            {
                semaphores[s].processQueue[semaphores[s].queueSize]=taskCurrent;
                semaphores[s].queueSize++;
                tcb[taskCurrent].state = STATE_BLOCKED;
                tcb[taskCurrent].semaphore = (void*)(R0);
                waitlog[s]=taskCurrent;
                NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
                break;
            }
        }
    case SYS_POST:
        for(sem=0;sem<MAX_SEMAPHORES;sem++)
        {
            if(&(semaphores[sem].count)==R0)
            {
                s=sem;
                break;
            }
        }
            semaphores[s].count++;                                                          //Incrementing the semaphore count as the task is releasing the semaphore
            if(semaphores[s].count == 1)                                                    //Checks if the semaphore count is 1
            {
                if(semaphores[s].queueSize > 0)                                             //Check if the queuesize is greater than zero to see if there are any blocked tasks
                {
                    if(PI)
                    {
                        tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;      //Returning the task priority to its base priority after priority inheritance
                        tcb[semaphores[s].processQueue[0]].state = STATE_READY;            //Make the first task state in the process queue ready
                        semaphores[s].queueSize--;                                          //Decrementing the queuesize as the task in Queue is now ready to run
                        semaphores[s].count--;                                              // Decrementing the semaphore count so that not letting the current task use the semaphore again
                        uint16_t i;
                        for(i=0;i<MAX_QUEUE_SIZE-1;i++)
                        {
                            semaphores[s].processQueue[i]=semaphores[s].processQueue[i+1];  //Shifting the process queue in FIFO
                        }
                        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;                           //Set PendSV bit to allow task switching
                        break;
                    }
                    else
                    {
                        tcb[semaphores[s].processQueue[0]].state = STATE_READY;
                        semaphores[s].queueSize--;
                        semaphores[s].count--;
                        uint16_t i;
                        for(i=0;i<MAX_QUEUE_SIZE-1;i++)
                        {
                            semaphores[s].processQueue[i]=semaphores[s].processQueue[i+1];
                        }
                        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
                        break;
                    }
                }
        }break;
    case SYS_DELETE_THREAD:
        for (d = 0; d < MAX_TASKS; d++)
        {
            if(tcb[d].pid==(void*)(R0))                                                     //Comparing the task pid with the passed address to find the right task
            {
                if(tcb[d].state==STATE_BLOCKED)                                             //Checking if the task state is blocked
                {
                    for(sem=0;sem<MAX_SEMAPHORES;sem++)
                    {
                        if(&(semaphores[sem].count)==tcb[d].semaphore)
                        {
                            s=sem;                                                          //Finding the semaphore the task uses or in its queue
                            break;
                        }
                    }
                    for (j=0;j<MAX_QUEUE_SIZE;j++)
                    {
                        if(semaphores[s].processQueue[j]==d)
                        {
                            semaphores[s].processQueue[j]=0;                                //After finding the correct semaphore removing the task from its queue
                            break;
                        }
                    }
                }
                tcb[d].state = STATE_INVALID;                                               //Making the task invalid
                tcb[d].pid = 0;                                                             //Setting task PID to zero
                if(taskCount>9)
                {
                    taskCount--;
                }
            }
        }
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;                                           //set PendSV bit to allow task switching
        break;
    case SYS_SET_PRIORITY:
        for (d = 0; d < taskCount; d++)
        {
            if(tcb[d].pid==(void*)(R0))                                                     //Checking if task PID matches the passed address
            {
                tcb[d].currentPriority=R1;                                                  //setting the task current priority to the new passed priority
                break;
            }
        }break;
    case SYS_UPDATE:
        putsUart0("Process name    ");
        putsUart0("PID    ");
        putsUart0("Priority    ");
        putsUart0("%CPU  ");
        putcUart0(0xA);
        putcUart0(13);

        for (d = 0; d < taskCount; d++)
        {
            char name[16];
            for(j=0;j<16;j++)
            {
                if(tcb[d].name[j]==0)
                {
                    name[j]='\0';
                    break;
                }
                name[j]=tcb[d].name[j];
            }
            putsUart0(name);                                                                //Printing the task name string
            putsUart0("\033[50D");
            putsUart0("\033[15C");
            snprintf(name,16,"%u",tcb[d].pid);                                              //Using thread-safe sprintf, where we give the size of memory for allocation
            putsUart0(name);                                                                //Using the same temporary variable to print the PID
            if(tcb[d].currentPriority<0)
            {
                tp=tcb[d].currentPriority-(2*tcb[d].currentPriority);                       //Printing the Current priority for negative priorities

                n=tp+'0';
                putsUart0("\033[50D");
                putsUart0("\033[25C");
                putsUart0("-");
                putcUart0(n);
            }
            else
            {
                n=tcb[d].currentPriority+'0';
                putsUart0("\033[50D");
                putsUart0("\033[25C");
                putcUart0(n);
            }
            putsUart0("\033[50D");
            putsUart0("\033[36C");
            for(j=0;j<5;j++)
            {
                if(j==2)
                {
                    n=tcb[d].percent[j] + '0';                                                  //Printing the CPU percentage digit by digit in xxx.yy% format
                    putcUart0(n);
                    putsUart0(".");
                }
                else
                    n=tcb[d].percent[j] + '0';
                putcUart0(n);
            }
            putcUart0(0xA);
            putcUart0(13);
        }
        break;


    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
    //           5 pushbuttons, and uart

    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOE |SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOC |  SYSCTL_RCGC2_GPIOB;

    //BLUE LED
    GPIO_PORTF_DIR_R |= 0X04; //OUTPUT
    GPIO_PORTF_DEN_R |= 0x04; //DIGITAL
    GPIO_PORTF_DR2R_R |= 0X04; //2mA DRIVE

    //RED,GREEN,YELLOW,ORANGE LEDS
    GPIO_PORTE_DIR_R |= 0X1E; //OUTPUT
    GPIO_PORTE_DEN_R |= 0x1E; //DIGITAL
    GPIO_PORTE_DR2R_R |= 0X1E; //2mA DRIVE

    GPIO_PORTA_DEN_R |= 0X7C;
    GPIO_PORTA_PUR_R |= 0X7C;

    // Configure UART0 pins(PA0,PA1)
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module


}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
    // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    return ((*(GPIO_PORTA_DATA_BITS_R + 0x07C)>>2) ^ 0x1F);                               //Reading the GPIO A register to find the bit change due to pushbuttons
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
    {
        yield();                                                                    //Using yield to allow task switch between Uart input
    }
    return UART0_DR_R & 0xFF;
}

void getstring()
{
    putsUart0("\033[s");
    uint8_t count=0;
    char c;
    start: c=getcUart0();                                                            //storing each character input
    c = tolower(c);
    putcUart0(c);                                                                    //Printing the input character
    if (c == 0x8)                                                                    //checks for backspace and moves the count back
    {
        if(count > 0)
        {
            count--;
            putcUart0(c);
            goto start;
        }
        else
            goto start;
    }
    else if(c == 0x0D)                                                                 //Checks for enter key to finish the string or typing
    {
        str[count]='\0';
        return;
    }
    else if(c<0x20)                                                                     //Checks for printable characters
    {
        goto start;
    }
    else if(c==0x21)                                                                    //Checks for ! function to execute the previous command
    {
        str[count++]=c;
        str[count]='\0';
    }
    else
    {
        str[count++]=c;
        if (count == MAX_CHAR)
        {
            str[count]='\0';
            return;
        }
        else
            goto start;
    }
    putcUart0(0xA);
    putcUart0(13);
}

void Shellstart()
{
    uint16_t i=0;
    putsUart0("\033[92m");
    putsUart0("[admin@user]: ");
    getstring();
    if(strcmp("!",str)==0)
    {
        for(i=0;i<MAX_CHAR;i++)
        {
            str[i]=buff[i];                                                 //Storing the current inputted string onto a buffer to execute the previous inputted command if needed
        }
    }
    putsUart0("\033[u");
    for(i=0;i<MAX_CHAR;i++)
    {
        buff[i]=str[i];
        if(hist>9)
        {
            hist=0;                                                         //checks if the history buffer is full
        }
        else
        {
            history[hist][i]=str[i];                                        //Stores the current string onto the history buffer
        }
    }
    hist++;
    putsUart0(str);
    putcUart0(0xA);
    putcUart0(13);
}

void parseshell()
{
    uint8_t i,j=0,k=0;
    uint8_t sl=strlen(str);
    for (i = 0; i < sl; i++)
    {
        if(i==0)
        {
            if(isspace(str[i]) | ispunct(str[i]))                               //checks for space and punctuations
            {
                str[i]='\0';                                                    //sets a delimiter or end of first part of the string
            }
            else if(isdigit(str[i]))                                            //checks if the character is a digit
            {
                argcount++;                                                     //Stores the argument
                pos[j++]=i;
                type[k++]='n';
            }
            else if(isalpha(str[i]))                                            //checks if alphabet
            {
                argcount++;                                                     //Stores the argument
                pos[j++]=i;
                type[k++]='a';
            }
        }
        else
        {
            if(isspace(str[i]) | ispunct(str[i]))
            {
                str[i]='\0';
                if (isdigit(str[i+1]))
                {
                    argcount++;
                    pos[j++]=1+i;
                    type[k++]='n';
                }
                else if(isalpha(str[i+1]))
                {
                    argcount++;
                    pos[j++]=1+i;
                    type[k++]='a';
                }
                else
                {
                    continue;
                }
            }
            else
            {
                continue;
            }

        }
    }
}

char Command()
{
    if(strcmp("pi",&str[pos[0]])==0)
    { if(strcmp("on",&str[pos[1]])==0)
    {
        return 'o';                                                     //checks if the string is pi on for turning on priority inheritance
    }
    else if(strcmp("off",&str[pos[1]])==0)
    {
        return 'f';                                                     //checks if the string is pi off for turning off priority inheritance
    }
    }
    else if(strcmp("reboot",&str[pos[0]])==0)
    {
        return 'q';                                                     //checks if the string is reboot to restart the RTOS
    }

    else if(strcmp("priority",&str[pos[0]])==0)
    { if(strcmp("rr",&str[pos[1]])==0)
    {
        return 'r';
    }
    else if(strcmp("on",&str[pos[1]])==0)
    {
        return 'p';                                                     //Checks if string asks for Priority scheduling or round robin scheduling
    }
    }
    else if(strcmp("preempt",&str[pos[0]])==0)
    {
        if (strcmp("on",&str[pos[1]])==0)
        {
            return 'e';
        }
        else if (strcmp("off",&str[pos[1]])==0)
        {
            return'c';                                                  //Checks if the string asks for preemption on or off(cooperative)
        }
    }
    else if(strcmp("pidof",&str[pos[0]])==0)
    {
        return 'w';                                                     //Checks if the string asks for the pid of task
    }
    else if(strcmp("kill",&str[pos[0]])==0)
    {
        return 'k';                                                     //checks for kill command
    }
    else if(strcmp("ps",&str[pos[0]])==0)
    {
        return 's';                                                     //Checks for process status command
    }
    else if(strcmp("ipcs",&str[pos[0]])==0)
    {
        return 'i';                                                     //Checks for interprocess communication command
    }
    else if(strcmp("help",&str[pos[0]])==0)
    {
        return 'h';                                                     //Checks for Help command
    }
    else if(strcmp("history",&str[pos[0]])==0)
    {
        return 'a';
    }
    else if(strcmp("idle",&str[pos[0]])==0)                             // checks if task name is on the command line for all tasks
    {
        if(strcmp("&",&str[pos[1]]))
        {
            return '0';
        }
    }
    else if(strcmp("lenghtyfn",&str[pos[0]])==0)
    {
        if(strcmp("&",&str[pos[1]]))
        {
            return '1';
        }
    }
    else if(strcmp("flash4hz",&str[pos[0]])==0)
    {
        if(strcmp("&",&str[pos[1]]))
        {
            return '2';
        }
    }
    else if(strcmp("oneshot",&str[pos[0]])==0)
    {
        if(strcmp("&",&str[pos[1]]))
        {
            return'3';
        }
    }
    else if(strcmp("readkeys",&str[pos[0]])==0)
    {
        if(strcmp("&",&str[pos[1]]))
        {
            return '4';
        }
    }
    else if(strcmp("debounce",&str[pos[0]])==0)
    {
        if(strcmp("&",&str[pos[1]]))
        {
            return '5';
        }
    }
    else if(strcmp("important",&str[pos[0]])==0)
    {
        if(strcmp("&",&str[pos[1]]))
        {
            return '6';
        }
    }
    else if(strcmp("uncoop",&str[pos[0]])==0)
    {
        if(strcmp("&",&str[pos[1]]))
        {
            return '7';
        }
    }
    else if(strcmp("shell",&str[pos[0]])==0)
    {
        if(strcmp("&",&str[pos[1]]))
        {
            return '8';
        }
    }
    else
    {
        putsUart0("Invalid\r\n");                                       //if non of the above match it will print invalid
    }
    return 0;
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            createThread(flash4Hz, "Flash4Hz", 0);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

void shell()
{
    putsUart0("\033[2;J\033[1;1H");                                             //Clears the screen on reboot
    putsUart0(" \033[92mDefault configuration: ");
    putcUart0(0xA);
    putcUart0(13);
    putsUart0(" Priority scheduling:  ON   ");
    putcUart0(0xA);
    putcUart0(13);
    putsUart0(" Priority Inheritance: ON   ");
    putcUart0(0xA);
    putcUart0(13);
    putsUart0(" RTOS Preemption: ON    ");
    putcUart0(0xA);
    putcUart0(13);
    putsUart0(" Please type help to know all the commands");
    putcUart0(0xA);
    putcUart0(13);
    while (true)
    {
        // REQUIRED: add processing for the shell commands through the UART here
        char C;
        Shellstart();                                                           //This function will get the input from Uart
        parseshell();                                                           //This function parses the string into arguments
        C=Command();                                                            //This function checks for the commands inputted
        if(C=='o')
        {
            PI=true;                                                            //Turn on PI
        }
        else if(C=='f')
        {
            PI=false;                                                           //Turn off PI
        }
        else if(C=='r')
        {
            PS=false;                                                           //Turn off priority scheduling
        }
        else if(C=='p')
        {
            PS=true;                                                            //Turn on priority scheduling
        }
        else if(C=='q')
        {
            NVIC_APINT_R = 0X04 | (0X05FA << 16);                               //Requesting Reset ISR to reboot the RTOS
        }
        else if(C=='w')
        {
            char name[16];
            char p[16];
            for(i=0;i<MAX_TASKS;i++)
            {
                for(j=0;j<16;j++)
                {
                    if(tcb[i].name[j]==0)
                    {
                        name[j]='\0';
                        break;
                    }
                    name[j]=tcb[i].name[j];
                }
                if(strcmp(name,&str[pos[1]])==0)                               //Checks if the name of the task matches with input string
                {

                    snprintf(p,16,"%u",tcb[i].pid);                             //Print the PID of the requested task
                    putsUart0(name);
                    putsUart0(" PID: ");
                    putsUart0(p);
                    putcUart0(0xA);
                    putcUart0(13);
                    break;
                }

            }
        }
        else if(C=='e')
        {
            pre = true;                                                         //Preemption on
        }
        else if(C=='c')
        {
            pre = false;                                                        //Cooperative
        }
        else if(C=='i')
        {
            putsUart0("Semaphore  ");
            putsUart0("Count  ");
            putsUart0("Queuesize  ");
            putsUart0("ProcessQueue  ");
            putcUart0(0xA);
            putcUart0(13);
            char name[16];
            char n;
            uint8_t temp;
            uint8_t digit;
            for(i=0;i<semaphoreCount;i++)
            {
                for(j=0;j<16;j++)
                {
                    if(semaphores[i].name[j]==0)
                    {
                        name[j]='\0';
                        break;
                    }
                    name[j]=semaphores[i].name[j];
                }
                putsUart0(name);                                                    //Printing semaphore name
                putsUart0("\033[50D");
                putsUart0("\033[13C");
                if(semaphores[i].count<10)
                {
                    n=semaphores[i].count + '0';                                    //Printing semaphore count
                    putcUart0(n);
                }
                else
                {
                    temp=semaphores[i].count;
                    digit=temp%10;
                    temp=temp/10;
                    n=temp +'0';
                    putcUart0(n);
                    n=digit+'0';
                    putcUart0(n);
                }
                putsUart0("\033[50D");
                putsUart0("\033[22C");
                n=semaphores[i].queueSize + '0';                                      //Printing semaphore Queuesize
                putcUart0(n);
                putsUart0("\033[50D");
                putsUart0("\033[31C");
                for(j=0;j<MAX_QUEUE_SIZE;j++)
                {
                    n= semaphores[i].processQueue[j] + '0';                           //Printing tasks in the process queue
                    putcUart0(n);
                    putsUart0(", ");
                }
                putcUart0(0xA);
                putcUart0(13);
                putcUart0(0xA);
                putcUart0(13);
            }
        }
        else if(C=='s')
        {
            UPDATE_PERCENT;                                                             //Calls SVC to update the CPU percentage and print the process status
        }
        else if(C=='h')
        {
            putsUart0("Supported Commands:");                                           //Printing the supported commands incase of help needed
            putcUart0(0xA);
            putcUart0(13);
            putsUart0("Priority Inheritance: pi on/off");
            putcUart0(0xA);
            putcUart0(13);
            putsUart0("Priority scheduling: priority on/rr");
            putcUart0(0xA);
            putcUart0(13);
            putsUart0("Preemption/Coop: preempt on/off");
            putcUart0(0xA);
            putcUart0(13);
            putsUart0("PID of process: pidof <processname>");
            putcUart0(0xA);
            putcUart0(13);
            putsUart0("Kill thread: kill <pid>");
            putcUart0(0xA);
            putcUart0(13);
            putsUart0("Process status: ps");
            putcUart0(0xA);
            putcUart0(13);
            putsUart0("Interprocess communication: ipcs");
            putcUart0(0xA);
            putcUart0(13);
            putsUart0("History: history");
            putcUart0(0xA);
            putcUart0(13);
        }
        else if(C=='a')
        {
            for(i=0;i<9;i++)
            {
                putsUart0(history[i]);                                              //Prints the last 10 inputted commands
                putcUart0(0xA);
                putcUart0(13);
            }
        }
        else if(C=='k')
        {
            char pid[10][16];
            for(i=0;i<MAX_TASKS;i++)
            {
                snprintf(pid[i],16,"%u",tcb[i].pid);

                if(strcmp(pid[i],&str[pos[1]])==0)                                  //Checks if the pid inputed matches the pid of the task and kills the task on request
                {
                    if(i==8)
                    {
                        putsUart0("Access Denied!");
                        putcUart0(0xA);
                        putcUart0(13);
                    }
                    else
                    {
                    destroyThread((_fn)(tcb[i].pid));
                    }
                }
            }
        }
        else if(C=='0')
        {
            createThread(idle,"Idle",0);                                            //Creating the task as requested in the commands
        }
        else if(C=='1')
        {
            createThread(lengthyFn,"Lenghtyfn",0);
        }
        else if(C=='2')
        {
            createThread(flash4Hz,"Flash4hz",0);
        }
        else if(C=='3')
        {
            createThread(oneshot,"Oneshot",0);
        }
        else if(C=='4')
        {
            createThread(readKeys,"Readkeys",0);
        }
        else if(C=='5')
        {
            createThread(debounce,"Debounce",0);
        }
        else if(C=='6')
        {
            createThread(important,"Important",0);
        }
        else if(C=='7')
        {
            createThread(uncooperative,"Uncoop",0);
        }
        else if(C=='8')
        {
            createThread(shell,"Shell",0);
        }

    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    rtosInit();

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore(1,"Keypressed");
    keyReleased = createSemaphore(0,"Keyreleased");
    flashReq = createSemaphore(5,"Flashreq");
    resource = createSemaphore(1,"Resource");

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7);

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 4);
    ok &= createThread(flash4Hz, "Flash4Hz", 0);
    ok &= createThread(oneshot, "OneShot", -4);
    ok &= createThread(readKeys, "ReadKeys", 4);
    ok &= createThread(debounce, "Debounce", 4);
    ok &= createThread(important, "Important", -8);
    ok &= createThread(uncooperative, "Uncoop", 2);
    ok &= createThread(shell, "Shell", 0);

    // Start up RTOS
    if (ok)
        rtosStart(); // never returns
    else
        RED_LED = 1;

    return 0;
}
