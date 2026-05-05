// read serial command C <float1> <float2>\n
// Output:    v1=float1
//            v2=float2
// wheelDesVel[0]=v1 (PI inputs)
// wheelDesVel[1]=v2
//
// read serial command T <task_name>\n
// Output: calls a function from the tasks.h header, which in turn commands the task motors on the robot to perform a task action
//
// read serial command D <R/G/B/P/''>,<R/G/B/P/''>,<R/G/B/P/''>,<R/G/B/P/''>\n
// Output: updates robot's LCD display with specified color

#pragma once

extern float wheelDesVel[2];

/* ================= ADDED ================= */
#include "tasks.h"
#include "lcd_display.h"
char taskName[15]={0};           // buffer for task command
char displayCmd[20]={0};         // buffer for display command
/* ========================================= */

char  command[30]={0};          // buffer to contain incoming serial command
char  param1[15]={0};           // buffer to contain string version of v1
char  param2[15]={0};           // buffer to contain string version of v2
float v1 = 0.;                  // float of left wheel velocity
float v2 = 0.;                  // float of right wheel velocity
int   cnt=0;                    // used to track size of v1/2
int   ch_count=0;               // used to track size of incoming serial command
int   i=0;                      // used to parse command for v1/2


void parse_serial(void) 
{
  if (Serial.available()>0)
    { 
      command[ch_count]=Serial.read();
      ch_count++;
      if (command[ch_count-1]==10)
      {
        // process command; extract two parameters 

          if ((command[0]=='C')&&(command[1]==32)&&(ch_count>5))
          {
            Serial.print(command);
            i = 2;
            cnt = 0;
            // extract 1st parameter
            while(command[i] !=32)
            {
              param1[cnt]=command[i];
              cnt++;
              i++; 
            }
            cnt = 0;
            i=i+1;
            while(command[i] !=10)
            {
              param2[cnt]=command[i];
              cnt++;
              i++; 
            }

            // convert string to float 

            v1 = atof(param1);
            v2 = atof(param2);
             
            
            Serial.print(v1);
            Serial.print(" ");
            Serial.println(v2);  

            // set wheel velocity
            wheelDesVel[0]=v1;  
            wheelDesVel[1]=v2;  

           // reset string variables 

            memset(param1,0,sizeof(param1));
            memset(param2,0,sizeof(param2));
      
          }

/* ================= TASK COMMAND ================= */

          else if ((command[0]=='T')&&(command[1]==32))
          {
            // extract task name
            i = 2;
            cnt = 0;

            while((command[i] !=10) && (cnt < 14))
            {
              taskName[cnt]=command[i];
              cnt++;
              i++; 
            }

            // execute matching task
            if (strcmp(taskName,"Push")==0)
            {
              pushTaskControl();
            }
            else if (strcmp(taskName,"Crank")==0)
            {
              crankTaskControl();
            }
            else if (strcmp(taskName,"Flag")==0)
            {
              flagTaskControl();
            }
            else
            {
              Serial.println("Unknown task");
            }

            // reset task buffer
            memset(taskName,0,sizeof(taskName));
          }

/* ======================================================= */

/* ================= ADDED: DISPLAY COMMAND ============== */

          else if ((command[0]=='D')&&(command[1]==32))
          {
            // extract display payload after "D "
            i = 2;
            cnt = 0;

            while((command[i] !=10) && (cnt < 19))
            {
              displayCmd[cnt]=command[i];
              cnt++;
              i++;
            }

            lcdUpdateFromCommand(displayCmd);
            Serial.println("Display updated");

            // reset display buffer
            memset(displayCmd,0,sizeof(displayCmd));
          }

/* ======================================================= */

          else
              Serial.println("Not a valid command");


        //Serial.print("No. of Char: ");
        //  Serial.println(ch_count);
        
        memset(command,0,sizeof(command));
        ch_count = 0;
      }

        
    }
} // end parse_serial