gcc thrd-posix.c -lpthread -o thrd-posix

./thrd-posix 1
./thrd-posix 2
./thrd-posix 3
./thrd-posix 4
./thrd-posix 5
./thrd-posix 6
./thrd-posix 7
./thrd-posix 8
./thrd-posix 9
./thrd-posix 10

gcc fork_exec.c -o fork_exec

./fork_exec ls

gcc newproc-posix.c -o newproc-posix

./newproc-posix 

g++ shm-prod-cons.cpp -o shm-prod-cons

#./shm-prod-cons

g++ shm-sem-prod-cons.cpp -o shm-sem-prod-cons

#./shm-sem-prod-cons
# ipcs --shmems
# ipcrm shm 12

g++ msgq-prod-cons.cpp -o msgq-prod-cons

./msgq-prod-cons

# ipcs --queues
# ipcrm msg 12
