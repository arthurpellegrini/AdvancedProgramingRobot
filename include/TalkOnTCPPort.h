#ifndef TALK_ON_TRCP_PORT_H
#define TALK_ON_TRCP_PORT_H

#include <string>

// Send a movement command to the TurtleBot
void startTalkOnTCPPort(int socket, float linear_x, float angular_z);

#endif // TALK_ON_TRCP_PORT_H
