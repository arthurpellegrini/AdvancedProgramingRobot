# Advanced Programming for Robots 

Google C++ Style Guideline: https://google.github.io/styleguide/cppguide.html

# How to compile provided c-scripts 

```bash
cd sockets
./compileme.sh
```

# How to commit to the current branch -> use conventional commit-messages https://www.conventionalcommits.org/en/v1.0.0/

```bash
git add .

git commit -m "feat: TITLE" \
-m "DESCRIPTION" \
-m "Co-authored-by: nils93 <n.fandrey@gmail.com>" \
-m "Co-authored-by: Arthur PELLEGRINI <arthur.pellegrini@outlook.fr>" \
-m "Co-authored-by:  cfriedl1 <ch.friedl1010@gmail.com>" \  
-m "Co-authored-by:  Felix130899 <felixhaier1308@gmail.com>"
#only 1 author + 1 co-author 

git push
```

# Makefile
This Makefile automates the compilation process for two programs: tcp_server and tcp_client. It defines the rules for compiling source files (.cpp and .c) into object files (.o) and linking them into executable programs. The Makefile also includes a clean rule to remove generated files for a fresh build. It simplifies the workflow for managing multiple targets and dependencies in a structured project.

# TCP-Server and TCP-Client
To run the tcp-server, just type in:
```bash
./tcp_server 10000
Server is listening on port 10000
```
To run the tcp_client, just type in:
```bash
./tcp_client 127.0.0.1 hello! 10000
Received: hello!
```