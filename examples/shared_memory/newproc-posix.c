/**
 * This program forks a separate process using the fork()/exec() system calls.
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>

#include <stdlib.h>    
#include <sys/wait.h>  

int main()
{
   pid_t pid;
   
   /* fork a child process */
   pid = fork();
   
   if (pid < 0) { /* error occurred */
      fprintf(stderr, "Fork Failed\n");
      exit(-1);
   }
   else if (pid == 0) { /* child process */
      printf("I am the child %d\n",pid);   /* addition */
      execlp("/bin/ls","ls",NULL);
   }
   else { /* parent process */
      /* parent will wait for the child to complete */
      printf("I am the parent %d\n",pid);  /* addition */
      wait(NULL);
      
      printf("Child Complete\n");
      exit(0);
   }
}
