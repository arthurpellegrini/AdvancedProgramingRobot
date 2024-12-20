gcc pthread1.c -lpthread -o pthread1
./pthread1
./pthread1
./pthread1
./pthread1
./pthread1
./pthread1
./pthread1
./pthread1
./pthread1
./pthread1
./pthread1
./pthread1
./pthread1
./pthread1
./pthread1

gcc race_demo.c -lpthread -o race_demo
./race_demo 1
./race_demo 2
./race_demo 3
./race_demo 4
./race_demo 5
./race_demo 1000
./race_demo 2000
./race_demo 3000
./race_demo 4000
./race_demo 5000
./race_demo 6000
./race_demo 7000
./race_demo 8000
./race_demo 9000
./race_demo 10000

gcc mutex1.c -lpthread -o mutex1
./mutex1
./mutex1
./mutex1
./mutex1
./mutex1
./mutex1
./mutex1
./mutex1
./mutex1
./mutex1
./mutex1
./mutex1
./mutex1
./mutex1
./mutex1
./mutex1
./mutex1

gcc join1.c -lpthread -o join1
./join1

ipcs -ls
ipcs --semaphores
# http://www.freekb.net/Article?id=3281
# ipcrm -s 131073

gcc sem1.c -lpthread -o sem1
./sem1 10
./sem1 100
./sem1 1000
./sem1 10000

gcc sem2_1.c -lpthread -o sem2_1
./sem2_1 10
./sem2_1 100
./sem2_1 1000
./sem2_1 10000

gcc sem2_2.c -lpthread -o sem2_2
./sem2_2 10
./sem2_2 100
./sem2_2 1000
./sem2_2 10000

gcc sem3.c -lpthread -o sem3
./sem3 10
./sem3 100
./sem3 1000
./sem3 10000

gcc semabinit.c -lpthread -o semabinit
gcc sema.c -lpthread -o sema
gcc semb.c -lpthread -o semb

./semabinit

ipcs --semaphores

# Semaphore example program 
# We have two programs, sema and semb. Semb may be initiated at any 
# time, but will be forced to wait until sema is executed. Sema and
# semb do not have to be executed by the same user! 
#
# HOW TO TEST:
#   Execute semb &
#   The & is important - otherwise you would have have to move to
#   a different terminal to execute sema.
#
#   Then execute sema.
#   
#   or exetute semb in one terminal and then sema in another terminal.
