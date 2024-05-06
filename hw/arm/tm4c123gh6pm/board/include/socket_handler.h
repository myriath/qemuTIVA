#ifndef CYBOT_SOCKETS_H_
#define CYBOT_SOCKETS_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdbool.h>
#include <pthread.h>
#include <errno.h>

#define SUCCESS         0
#define ERR_SOCK_FAILED -1
#define ERR_CONNECT     -2
#define ERR_BIND        -3
#define ERR_LISTEN      -4
#define ERR_ACCEPT      -5

#define FIFO_SIZE 1024

#define SERVER_INSTRUCTOR   0
#define SERVER_STUDENT      1
#define COUNT_SERVERS       2

struct fifo {
    pthread_mutex_t mutex;
    bool empty;
    bool full;
    char *data;
    ssize_t size;
    uint32_t pos;
    uint32_t count;
};

struct server {
    int sock;
    int port;
    bool connected;
    struct fifo *send_fifo;
    struct fifo *recv_fifo;
};

struct server_creator {
    struct server *server;
    pthread_t *server_thread;
    void (*callback)(void *opaque);
    void *opaque;
};

/**
 * Creates the student and instructor servers for reading from and sending to.
 * @param server_num constant, either SERVER_STUDENT or SERVER_INSTRUCTOR
 * @param port port for the server to run on
 * @param opaque Data to pass back via the callback function
 * @param connect_callback callback function for when the instructor connects
 * @return pointer to the thread object, NULL if failed.
*/
pthread_t *create_server(int server_num, int port, void *opaque, void (*connect_callback)(void *opaque));

/**
 * Frees all servers 
*/
void free_all_servers(void);

/**
 * Frees the given server
 * @param server either SERVER_STUDENT or SERVER_INSTRUCTOR
*/
void free_server_num(int server);

/**
 * Asynchronously sends the given data to the correct server.
 * @param data character to send to the client
 * @param server constant, either SERVER_STUDENT or SERVER_INSTRUCTOR
*/
void wifi_send(char data, int server);
/**
 * Asynchronously receives data from the correct server.
 * @param server constant, either SERVER_STUDENT or SERVER_INSTRUCTOR
 * @return data from the client. If no data available, returns -1.
*/
char wifi_recv(int server);

/**
 * See if there is space to send data via the requested server.
 * @param server constant, either SERVER_STUDENT or SERVER_INSTRUCTOR
 * @return true if there is no space to send more data yet.
*/
bool is_wifi_full(int server);
/**
 * See if there is any data ready to read from the requested server.
 * @param server constant, either SERVER_STUDENT or SERVER_INSTRUCTOR
 * @return true if there is no data to read yet.
*/
bool is_wifi_empty(int server);
/**
 * See if the given server is connected. Use the macros from the top of this file.
 * @param server constant, either SERVER_STUDENT or SERVER_INSTRUCTOR
 * @return True if the server is connected, false if not, or the server number was invalid.
*/
bool is_wifi_connected(int server);

#endif
