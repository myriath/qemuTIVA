#include "socket_handler.h"

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
    struct fifo send_fifo;
    struct fifo recv_fifo;
};

struct server student_server;
struct server instructor_server;

void put_fifo(char data, struct fifo *fifo)
{
    pthread_mutex_lock(&fifo->mutex);
    if (fifo->count < FIFO_SIZE) {
        fifo->empty = false;
        fifo->data[(fifo->pos + fifo->count) % fifo->size] = data;
        fifo->count++;
        if (fifo->count >= fifo->size) {
            fifo->full = true;
        }
    } else {
        fifo->full = true;
    }
    pthread_mutex_unlock(&fifo->mutex);
}

char pop_fifo(struct fifo *fifo)
{
    char ret = -1;
    pthread_mutex_lock(&fifo->mutex);
    if (fifo->count > 0) {
        fifo->full = false;
        ret = fifo->data[fifo->pos];
        fifo->pos = (fifo->pos + 1) % fifo->size;
        fifo->count--;
        if (fifo->count <= 0) {
            fifo->empty = true;
        }
    } else {
        fifo->empty = true;
    }
    pthread_mutex_unlock(&fifo->mutex);
    return ret;
}

void wifi_send(char data, bool use_student_server)
{
    if (use_student_server) {
        put_fifo(data, &student_server.send_fifo);
    } else {
        put_fifo(data, &instructor_server.send_fifo);
    }
}

bool is_wifi_full(bool use_student_server)
{
    if (use_student_server) {
        return student_server.send_fifo.full;
    } else {
        return instructor_server.send_fifo.full;
    }
}

char wifi_recv(bool use_student_server)
{
    if (use_student_server) {
        return pop_fifo(&student_server.recv_fifo);
    } else {
        return pop_fifo(&instructor_server.recv_fifo);
    }
}

bool is_wifi_empty(bool use_student_server)
{
    if (use_student_server) {
        return student_server.recv_fifo.full;
    } else {
        return instructor_server.recv_fifo.full;
    }
}

void *thread_send(void *opaque)
{
    struct server *server = (struct server *) opaque;
    while (true) {
        if (server->send_fifo.empty) {
            usleep(10);
            continue;
        }
        char data = pop_fifo(&server->send_fifo);
        if (send(server->sock, &data, 1, 0) < 0) {
            printf("Failed to send: %d\n", errno);
            return NULL;
        }
    }
}

void *thread_recv(void *opaque)
{
    struct server *server = (struct server *) opaque;
    while (true) {
        if (server->recv_fifo.full) {
            usleep(10);
            continue;
        }
        char data;
        int e = recv(server->sock, &data, 1, 0);
        if (e <= 0) {
            printf("Failed to read: %d, %d\n", e, errno);
            return NULL;
        }
        put_fifo(data, &server->recv_fifo);
    }
}


void *thread_start_server(void *opaque)
{
    struct server *server = (struct server *) opaque;
    
    int fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) {
        perror("Failed to create socket");
        exit(EXIT_FAILURE);
    }
    int option = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(struct sockaddr_in);
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(server->port),
        .sin_addr.s_addr = INADDR_ANY
    };
    memset(&(server_addr.sin_zero), 0, 8);
    
    printf("Binding...\n");
    if (bind(fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
        perror("Failed to bind port");
        exit(EXIT_FAILURE);
    }
    printf("Listening...\n");
    if (listen(fd, 1) < 0) {
        perror("Failed to listen");
        exit(EXIT_FAILURE);
    }

    server->sock = accept(fd, (struct sockaddr *) &client_addr, &client_addr_len);
    printf("Accepted\n");
    if (server->sock < 0) {
        perror("Failed to accept connection");
        exit(EXIT_FAILURE);           
    }

    pthread_t send_thread;
    pthread_t recv_thread;

    if (pthread_create(&send_thread, NULL, thread_send, (void*) server) < 0) {
        perror("Failed to create send thread");
        exit(EXIT_FAILURE);
    }
    if (pthread_create(&recv_thread, NULL, thread_recv, (void*) server) < 0) {
        perror("Failed to create recv thread");
        exit(EXIT_FAILURE);
    }

    pthread_join(send_thread, NULL);
    pthread_join(recv_thread, NULL);

    close(server->sock);
    close(fd);
    return SUCCESS;
}

void initialize_fifo(struct fifo *fifo, ssize_t size)
{
    fifo->count = 0;
    fifo->pos = 0;
    fifo->full = false;
    fifo->empty = true;
    pthread_mutex_init(&fifo->mutex, NULL);
    fifo->data = (char *) malloc(size * sizeof(char));
    fifo->size = size;
}

void initialize_server(bool student, int port, int size)
{
    struct server *server;
    if (student) {
        server = &student_server;
    } else {
        server = &instructor_server;
    }

    server->port = port;
    initialize_fifo(&server->recv_fifo, size);
    initialize_fifo(&server->send_fifo, size);
}

int create_servers(int student_port, int instructor_port)
{
    initialize_server(true, student_port, FIFO_SIZE);
    initialize_server(false, instructor_port, FIFO_SIZE);

    printf("Ports: %d, %d\n", student_port, instructor_port);

    pthread_t student_thread;
    pthread_t instructor_thread;

    if (pthread_create(&student_thread, NULL, thread_start_server, (void*) &student_server) < 0) {
        perror("Failed to create student server thread");
        return errno;
    }
    if (pthread_create(&instructor_thread, NULL, thread_start_server, (void*) &instructor_server) < 0) {
        perror("Failed to create student server thread");
        return errno;
    }

    // pthread_join(student_thread, NULL);
    // pthread_join(instructor_thread, NULL);
    return 0;
}
