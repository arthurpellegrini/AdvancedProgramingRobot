#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

long int COUNT_MAX;
volatile long int in;

void *increase_in(void *arg)    // Called as thread
{
  long int i;
  long int next_free_slot;
  for (i = 0; i < COUNT_MAX; ++i)
    {
      next_free_slot = in;
      next_free_slot = next_free_slot + 1;
      in = next_free_slot;
    }
  return NULL;
}

int main(int argc, char *argv[])
{
  pthread_t thread1_id;
  pthread_t thread2_id;

  if (argc != 2)
    {
      printf("Usage: %s COUNT_MAX\n", argv[0]);
      exit(EXIT_FAILURE);
    }
  sscanf(argv[1], "%lu", &COUNT_MAX);

  in = 0;
  pthread_create(&thread1_id, NULL, &increase_in, NULL);
  pthread_create(&thread2_id, NULL, &increase_in, NULL);
  pthread_join(thread1_id, NULL);
  pthread_join(thread2_id, NULL);
  printf("in=%ld\n", in);
  return EXIT_SUCCESS;
}
