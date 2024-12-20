/* File: msgq-prod-cons.cpp
 *
 * This program sets up producer/consumer problem using message queues.
 * Message queues are synchronized by OS.
 *
 * Contains both the producer and the consumer code The producer
 * (parent) remains the foreground process, so to stop this program,
 * send it SIGINT (Ctrl-C) and it will send SIGINT to kill the
 * consumer.  The message queue is removed from the system by the
 * producer
 */

#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <stdlib.h>  
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <iostream>
using namespace std;

// Prototypes
void producerHandler (int sig);  // Signal handler for the producer
void consumerHandler (int sig);  // Signal handler for the consumer

// Global definitions and variables
struct Message           // Format of the messages
{
   long type;            // message type, required
   int data;             // item is an int, can by anything
};
enum MessageType { PROD_MSG=1, CONS_MSG };
// CONS_MSG is not used in this program, since the consumer doesn't
// send messages to the producer.  Can also have more than 2 types
// of messages if needed.

int msgqid;                // Message queue id
pid_t child_pid;           // Result of fork(); global for sig handler
   
int main()
{
   // create message queue

   msgqid = msgget(IPC_PRIVATE, 0666);   
   if (msgqid == -1) {
      cerr << "msgget failed\n";
      exit(EXIT_FAILURE);
   }

   /* fork a child process */
   child_pid = fork();
   if (child_pid < 0) { /* error occurred */
      cerr << "Fork Failed\n";
      exit(-1);
   }

   if (child_pid == 0) // child process - the consumer
   { 
      signal (SIGINT, consumerHandler);  // catch SIGINT
      cout << "I am the child\n";

      // consuming loop
      while (true)
      {
         cout << "Consumer attempting to read message\n";
	 			 Message consMsg;

	 // receive message - should test for error
	       msgrcv(msgqid, &consMsg, sizeof(int), PROD_MSG, 0);
         cout << "Consumer read: " << consMsg.data << endl;
         sleep(rand()%3);  // 0-2 seconds
      }  // end consuming loop
   }  // end consumer code
   else  // parent process - the producer
   { 
      signal (SIGINT, producerHandler);  // catch SIGINT
      cout << "I am the producer\n";

      // producing loop
      while (true)
      {
         cout << "Producer attempting write\n";
         Message prodMsg;
         prodMsg.type = PROD_MSG;
         prodMsg.data = rand()%10000; // random integer between 0 and 9999

	 // send message - should test for error
         msgsnd(msgqid, &prodMsg, sizeof(int), 0);
         cout << "Producer sent: " << prodMsg.data << endl;
         sleep(rand()%3);  // 0-3 seconds
      }  // end producing loop
   }  // end producer code
}  // end main

// Signal handler for the consumer
void consumerHandler (int sig)
{
   cout << "Consumer exiting\n";
   exit(EXIT_SUCCESS);
}  // end consumerHandler

// Signal handler for the producer
void producerHandler (int sig)
{
   // kill child and wait for it
   cout << "Producer killing consumer\n";
   kill (child_pid, SIGINT);
   wait (NULL);

   // remove message queue
   cout << "Producer removing message queue\n";
   if (msgctl(msgqid, IPC_RMID, 0) == -1) {
      cerr << "msgctl(IPC_RMID) failed\n";
      exit(EXIT_FAILURE);
   }
   exit(EXIT_SUCCESS);
}  // end producerHandler
