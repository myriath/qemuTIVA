#include "main.h"

int create_server(int port, void (*handle)())
{
    int fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) {
        return ERR_SOCK_FAILED;
    }

    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(struct sockaddr_in);
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = INADDR_ANY
    };
    memset(&(server_addr.sin_zero), 0, 8);
    
    printf("Binding...\n");
    if (bind(fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
        return ERR_BIND;
    }
    printf("Listening...\n");
    if (listen(fd, 1) < 0) {
        return ERR_LISTEN;
    }
    int fd_client = accept(fd, (struct sockaddr *) &client_addr, &client_addr_len);
    printf("Accepted\n");
    if (fd_client < 0) {
        return ERR_ACCEPT;
    }

    handle();

    close(fd_client);
    close(fd);
    return SUCCESS;
}

void handler(void) 
{
    printf("Success\n");
}

int main(int argc, char **argv) 
{
    if (argc != 2) exit(EXIT_FAILURE);
    
    int port = atoi(argv[1]);

    switch (create_server(port, handler)) {
        case SUCCESS:
            exit(EXIT_SUCCESS);
        case ERR_SOCK_FAILED:
            perror("Failed to create socket");
            break;
        case ERR_BIND:
            perror("Failed to bind");
            break;
        case ERR_LISTEN:
            perror("Failed to listen");
            break;
        case ERR_ACCEPT:
            perror("Failed to accept");
            break;
        default:
            perror("Error");
    }
    exit(EXIT_FAILURE);
}