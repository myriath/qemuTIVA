#ifndef __CYBOT_WIFI_H_
#define __CYBOT_WIFI_H_

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

/**
 * Creates the student and instructor servers for reading from and sending to.
 * @param student_port port to listen for the student code on.
 * @param instructor_port port to listen for the instructor interface on.
 * @return errno if either server startup thread fails to be created.
*/
int create_servers(int student_port, int instructor_port);

/**
 * Asynchronously sends the given data to the correct server.
 * @param data character to send to the client
 * @param use_student_server true if the server used is the student server. False to send to instructor server.
*/
void wifi_send(char data, bool use_student_server);
/**
 * Asynchronously receives data from the correct server.
 * @param use_student_server true if the server used is the student server. False to send to instructor server.
 * @return data from the client. If no data available, returns -1.
*/
char wifi_recv(bool use_student_server);

/**
 * See if there is space to send data via the requested server.
 * @param use_student_server true if the server checked is the student server. False to check the instructor server.
 * @return true if there is no space to send more data yet.
*/
bool is_wifi_full(bool use_student_server);
/**
 * See if there is any data ready to read from the requested server.
 * @param use_student_server true if the server checked is the student server. False to check the instructor server.
 * @return true if there is no data to read yet.
*/
bool is_wifi_empty(bool use_student_server);

#endif
