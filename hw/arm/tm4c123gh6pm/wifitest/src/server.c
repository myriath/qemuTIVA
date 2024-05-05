#include "socket_handler.h"

int main(int argc, char **argv) 
{
    if (argc != 3) exit(EXIT_FAILURE);
    
    int s_port = atoi(argv[1]);
    int i_port = atoi(argv[2]);

    if (create_servers(s_port, i_port) != 0) {
        exit(EXIT_FAILURE);
    }

    char buf[1024];

    char data;
    int i = 0;
    while (true) {
        if (is_wifi_empty(SERVER_STUDENT)) {
            usleep(10);
            continue;
        }
        data = wifi_recv(SERVER_STUDENT);
        buf[i++] = data;

        if (data == '\n' || i >= 1025) {
            for (int j = 0; j < i; j++) {
                while (is_wifi_full(SERVER_STUDENT)) {
                    usleep(10);
                }
                wifi_send(buf[j], SERVER_STUDENT);
            }
            i = 0;
        }
    }
}
