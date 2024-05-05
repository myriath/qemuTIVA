all: bin/client bin/server

bin/client: src/client.c
	gcc src/client.c -o bin/client

bin/server: src/server.c src/socket_handler.c
	gcc src/server.c src/socket_handler.c -o bin/server