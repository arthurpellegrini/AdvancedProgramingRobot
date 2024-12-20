/* File: shm-prod-cons.cpp
 *
 * This program sets up producer/consumer problem using shared memory.
 * It is incomplete as it does not have any synchronization, but the
 * likelihood that ++ or -- will be interrupted is fairly remote
 *
 * Contains both the producer and the consumer code
 * The producer (parent) remains the foreground process, so to stop
 * this program, send it SIGINT (Ctrl-C) and it will send SIGINT to
 * kill the consumer.  Both will detach the shared memory and the
 * producer will remove it from the system.
 */

#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <stdlib.h>  
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <iostream>
using namespace std;

// Prototypes
void producerHandler (int sig);  // Signal handler for the producer
void consumerHandler (int sig);  // Signal handler for the consumer

// Global definitions and variables
const int BUFFER_SIZE = 5;   // Number of buffer slots
struct SharedMemory          // Format of the shared memory
{
   int numItems;             // Count of items in buffer
   int in, out;              // Indexes to back and front
   int buffer[BUFFER_SIZE];  // Items are integers
};

int shmid;                 // Shared memory id
SharedMemory *shmptr;      // Pointer to shared memory
pid_t child_pid;           // Result of fork()
   
int main(void )
{
   // create shared memory
   shmid = shmget(IPC_PRIVATE, sizeof(struct SharedMemory), 0666|IPC_CREAT);   
   if (shmid == -1) {
      cerr << "shmget failed\n";
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
      cout << "I am the consumer\n"
           << "Consumer will sleep for a bit to make sure that producer "
           << "initializes shared memory" << endl;
      sleep(1);

      // attach shared memory at shmptr
      cout << "Consumer: attaching shared memory\n";
      shmptr = (SharedMemory*)shmat(shmid, NULL, 0);
      if (shmptr == (SharedMemory *)-1) {
         cerr << "shmat failed\n";
         exit(EXIT_FAILURE);
      }
      cout << "Consumer: memory attached\n";

      // consuming loop
      while (true)
      {
         cout << "Consumer attempting to read\n";
         while (shmptr->numItems == 0)
            ;  // do nothing
         shmptr->numItems--;
         int nextConsumed = shmptr->buffer[shmptr->out];
         cout << "Consumer read: " << nextConsumed
              << " at index " << shmptr->out << endl;
         shmptr->out = (shmptr->out + 1) % BUFFER_SIZE;
         sleep(rand()%3);  // 0-3 seconds
      }  // end consuming loop
   }  // end consumer code
   else  // parent process - the producer
   { 
      signal (SIGINT, producerHandler);  // catch SIGINT
      cout << "I am the producer\n";

      // attach shared memory
      cout << "Producer: attaching shared memory\n";
      shmptr = (SharedMemory *)shmat(shmid, (void *)0,0);
      if (shmptr == (SharedMemory *)-1) {
         cerr << "shmat failed\n";
         exit(EXIT_FAILURE);
      }
      cout << "Producer: memory attached\n";

      // initialize shared memory
      cout << "Producer: initializing shared memory\n";
      shmptr->in = 0;
      shmptr->out = 0;
      shmptr->numItems = 0;
      cout << "Producer: shared memory initialized\n";
      
      // producing loop
      while (true)
      {
         cout << "Producer attempting write\n";
         int nextProduced = rand()%10000; // random integer between 0 and 9999
         while (shmptr->numItems == BUFFER_SIZE)
            ;  // do nothing
         shmptr->buffer[shmptr->in] = nextProduced;
         cout << "Producer wrote: " << nextProduced 
              << " at index " << shmptr->in << endl;
         shmptr->in = (shmptr->in + 1) % BUFFER_SIZE;
         shmptr->numItems++;
         sleep(rand()%3);  // 0-3 seconds
      }  // end producing loop
   }  // end producer code
}  // end main

// Signal handler for the consumer
void consumerHandler (int sig)
{
   // detach shared memory
   cout << "Consumer detaching shared memory\n";
   if (shmdt(shmptr) == -1) {
      cerr << "shmdt failed\n";
      exit(EXIT_FAILURE);
   }
   exit(EXIT_SUCCESS);
}  // end consumerHandler

// Signal handler for the producer
void producerHandler (int sig)
{
   // kill child and wait for it
   cout << "Producer killing consumer\n";
   kill (child_pid, SIGINT);
   wait (NULL);

   cout << "Producer detaching and removing shared memory\n";
   // detach shared memory
   if (shmdt(shmptr) == -1) {
      cerr << "shmdt failed\n";
      exit(EXIT_FAILURE);
   }
   // remove shared memory
   if (shmctl(shmid, IPC_RMID, 0) == -1) {
      cerr << "shmctl(IPC_RMID) failed\n";
      exit(EXIT_FAILURE);
   }
   exit(EXIT_SUCCESS);
}  // end producerHandler
