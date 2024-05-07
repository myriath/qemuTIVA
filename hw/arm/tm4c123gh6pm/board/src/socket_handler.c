#include "hw/arm/tm4c123gh6pm/board/include/socket_handler.h"

struct server_creator *creators[COUNT_SERVERS];

static void put_fifo(char data, struct fifo *fifo)
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

static char pop_fifo(struct fifo *fifo)
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

void wifi_send(char data, int server)
{
    put_fifo(data, creators[server]->server->send_fifo);
}

char wifi_recv(int server)
{
    struct server *s = creators[server]->server;
    if (!s->connected) {
        return -1;
    }
    return pop_fifo(creators[server]->server->recv_fifo);
}

bool is_wifi_full(int server)
{
    return creators[server]->server->send_fifo->full;
}

bool is_wifi_empty(int server)
{
    return creators[server]->server->recv_fifo->empty;
}

bool is_wifi_connected(int server)
{
    return creators[server]->server->connected;
}

static void *thread_send(void *opaque)
{
    struct server *server = (struct server *) opaque;
    while (true) {
        if (server->send_fifo->empty) {
            usleep(10);
            continue;
        }
        char data = pop_fifo(server->send_fifo);
        if (send(server->sock, &data, 1, 0) < 0) {
            printf("Failed to send: %d\n", errno);
            return NULL;
        }
    }
}

static void *thread_recv(void *opaque)
{
    struct server *server = (struct server *) opaque;
    while (true) {
        if (server->recv_fifo->full) {
            usleep(10);
            continue;
        }
        char data;
        int e = recv(server->sock, &data, 1, 0);
        if (e <= 0) {
            printf("Failed to read: %d, %d\n", e, errno);
            return NULL;
        }
        put_fifo(data, server->recv_fifo);
    }
}


static int initialize_fifo(struct fifo *fifo, ssize_t size)
{
    fifo->count = 0;
    fifo->pos = 0;
    fifo->full = false;
    fifo->empty = true;
    pthread_mutex_init(&fifo->mutex, NULL);
    fifo->data = (char *) malloc(size * sizeof(char));
    if (fifo->data == NULL) {
        return -1;
    }
    fifo->size = size;
    return 0;
}

static int initialize_creator(struct server_creator *ret, struct server *server, int port, int size, void *opaque, void (*connect_callback)(void *opaque)) 
{
    server->port = port;
    server->connected = false;
    server->recv_fifo = malloc(sizeof(struct fifo));
    server->send_fifo = malloc(sizeof(struct fifo));
    if (server->recv_fifo == NULL || server->send_fifo == NULL) {
        return -1;
    }
    if (initialize_fifo(server->recv_fifo, size) < 0) {
        return -1;
    }
    if (initialize_fifo(server->send_fifo, size) < 0) {
        return -1;
    }

    ret->server = server;
    ret->callback = connect_callback;
    ret->opaque = opaque;
    return 0;
}

static void free_fifo(struct fifo *fifo)
{
    free(fifo->data);
    free(fifo);
}

static void free_server(struct server *server)
{
    free_fifo(server->recv_fifo);
    free_fifo(server->send_fifo);
    free(server);
}

static void free_creator(struct server_creator *creator) 
{
    free_server(creator->server);
    free(creator->server_thread);
    free(creator);
}

void free_server_num(int server) 
{
    free_creator(creators[server]);
}

void free_all_servers(void)
{
    for (int i = 0; i < COUNT_SERVERS; i++) {
        free_server_num(i);
    }
}

static void *thread_start_server(void *opaque)
{
    struct server_creator *creator = (struct server_creator *) opaque;
    struct server *server = creator->server;
    
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
    
    if (bind(fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
        free_creator(creator);
        perror("Failed to bind port");
        exit(EXIT_FAILURE);
    }
    printf("Listening on Port %d\n", server->port);
    if (listen(fd, 1) < 0) {
        free_creator(creator);
        perror("Failed to listen");
        exit(EXIT_FAILURE);
    }

    server->sock = accept(fd, (struct sockaddr *) &client_addr, &client_addr_len);
    printf("Accepted on Port %d\n", server->port);
    if (server->sock < 0) {
        close(fd);
        free_creator(creator);
        perror("Failed to accept connection");
        exit(EXIT_FAILURE);           
    }
    server->connected = true;
    if (creator->callback != NULL) {
        creator->callback(creator->opaque);
    }

    pthread_t send_thread;
    pthread_t recv_thread;

    if (pthread_create(&send_thread, NULL, thread_send, (void*) server) < 0) {
        close(server->sock);
        close(fd);
        free_creator(creator);
        perror("Failed to create send thread");
        exit(EXIT_FAILURE);
    }
    if (pthread_create(&recv_thread, NULL, thread_recv, (void*) server) < 0) {
        close(server->sock);
        close(fd);
        free_creator(creator);
        perror("Failed to create recv thread");
        exit(EXIT_FAILURE);
    }

    pthread_join(send_thread, NULL);
    pthread_join(recv_thread, NULL);

    server->connected = false;

    close(server->sock);
    close(fd);

    free_creator(creator);
    return NULL;
}

pthread_t *create_server(int server_num, int port, void *opaque, void (*connect_callback)(void *opaque))
{
    if (server_num < 0 || server_num >= COUNT_SERVERS) {
        return NULL;
    }

    struct server_creator *creator = malloc(sizeof(struct server_creator));
    struct server *server = malloc(sizeof(struct server));
    if (creator == NULL || server == NULL) {
        perror("Unable to allocate memory");
        return NULL;
    }
    if (initialize_creator(creator, server, port, FIFO_SIZE, opaque, connect_callback) < 0) {
        free_creator(creator);
        perror("Unable to allocate memory");
        return NULL;
    }

    pthread_t *thread = malloc(sizeof(pthread_t));
    if (thread == NULL) {
        free_creator(creator);
        perror("Unable to allocate memory");
        return NULL;
    }

    if (pthread_create(thread, NULL, thread_start_server, (void *) creator) < 0) {
        free_creator(creator);
        perror("Failed to create server thread");
        return NULL;
    }
    creator->server_thread = thread;
    
    creators[server_num] = creator;

    return thread;
}
