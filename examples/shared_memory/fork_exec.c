#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

int main(int argc, char** argv) {
	struct timeval tv1;
	struct timeval tv2;
	float timediff;
	int pid;
	int status;

	printf("program executed: %s\n", *(argv+1));

	gettimeofday(&tv1, NULL);
	pid = fork();
	gettimeofday(&tv2, NULL);

	// Zeitdifferenz in Sekunden
	timediff = (tv2.tv_sec + (double)tv2.tv_usec / 1000000) - (tv1.tv_sec + (double)tv1.tv_usec / 1000000);

	// Child-Prozess
	if (pid == 0) {
		printf("Child: timediff=%f\n", timediff);
		execv(*(argv+1), argv+2);
	}
	// Parent-Prozess
	else {
		printf("Parent: timediff=%f\n", timediff);
		wait(&status);
		printf("Parent: Child finished, status %d\n", status);
	}

	return 0;
}
