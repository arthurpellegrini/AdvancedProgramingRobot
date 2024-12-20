#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>

#define EINTR -1

// These variables need to be global, since they are used by the threads
long int COUNT_MAX;
volatile long int in;

// One semaphore... to rule them all
sem_t turn;

void enter_criticalregion(long process)
{
  int err = 0;
  // Wait until turn > 0 
  // (the loop just makes sure no interrupt messes up things)
  do { err = sem_wait(&turn); } while ( err == EINTR ); // semaphore P()
  // we're inside now !
  fprintf(stderr, "%ld", process);
}

void leave_criticalregion(long process)
{
  sem_post(&turn); // semaphore V()
}


void *increase_in(void *arg)    // Called as thread
{
  long int i;
  long int next_free_slot;
  for (i = 0; i < COUNT_MAX; ++i)
    {

      enter_criticalregion((long) arg);

      next_free_slot = in;
      sched_yield();					// schelude better !!
      next_free_slot = next_free_slot + 1;
      in = next_free_slot;

      leave_criticalregion((long) arg);

    }
  return NULL;
}

int main(int argc, char *argv[])
{
  pthread_t thread1_id;
  pthread_t thread2_id;

  if ( sem_init(&turn, 0, 1) == -1 )
  {
    fprintf(stderr, "Error initialising the Semaphore\n");
    exit(EXIT_FAILURE);
  }

  if (argc != 2)
    {
      printf("Usage: %s COUNT_MAX\n", argv[0]);
      exit(EXIT_FAILURE);
    }
  sscanf(argv[1], "%lu", &COUNT_MAX);

  in = 0;
  pthread_create(&thread1_id, NULL, &increase_in, (void *) 0);
  pthread_create(&thread2_id, NULL, &increase_in, (void *) 1);
  pthread_join(thread1_id, NULL);
  pthread_join(thread2_id, NULL);
  printf("\nin=%ld\n", in);

  sem_destroy(&turn);
  return EXIT_SUCCESS;
}
