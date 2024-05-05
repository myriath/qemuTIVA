#ifndef __MAIN_H_
#define __MAIN_H_

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

#endif
