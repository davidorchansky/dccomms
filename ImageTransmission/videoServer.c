#include<stdio.h>
#include<string.h>    //strlen
#include<stdlib.h>    //strlen
#include<sys/socket.h>
#include<arpa/inet.h> //inet_addr
#include<unistd.h>    //write
#include<pthread.h> //for threading , link with lpthread
#include<ctype.h>
#include<signal.h>
#include<videoServer.h>

/*******************************/
//the thread function
void *connection_handler(void *);

pthread_mutex_t lock;

int socket_desc;
int server_grabber_pipe[2],
	server_transmitter_pipe[2],
	grabber_transmitter_pipe[2];

videoTransmissionConfig config;

void sigint_handler (int sig)
{ 
	printf ("Recibida la senyal %d.\nCerrando socket del servidor...\n",sig);
	close(socket_desc);
	printf("Servidor cerrado correctamente.\n");
	fflush(stdout);
	exit(0);
}


/*
 * This will handle connection for each client
 * */
void *connection_handler(void *socket_desc)
{
    //Get the socket descriptor
    int sock = *(int*)socket_desc;
    int read_size,n;

    char c = ' ';
    char buffer[500];
     
    //Receive a message from client
    while( (read_size = recv(sock , &c , 1, 0)) > 0 )
    {

	if( c == '{')
	{
		if(getVideoTransmissionConfig(sock, &config)==-1)continue;
		n = buildVideoTransmissionConfigMsg(buffer, &config);
		fprintf(stdout, "SERVIDOR DE CONFIGURACION DE VIDEO: Enviando configuracion (%d): %s\n",n,buffer);

		fflush(stdout);
    		pthread_mutex_lock(&lock); 
		write(server_transmitter_pipe[1], buffer, n);
    		pthread_mutex_unlock(&lock); 
	}

    }
     
    if(read_size == 0)
    {
        puts("Client disconnected");
        fflush(stdout);
    }
    else if(read_size == -1)
    {
        perror("recv failed");
    }
         
    return 0;
}

int runServer()
{
    struct sigaction act;
    act.sa_handler=sigint_handler; //Hay que cerrar el servidor correctamente
    act.sa_flags=0;
    sigemptyset(&act.sa_mask);
    sigaction(SIGINT,&act,NULL);

    int client_sock , c;
    struct sockaddr_in server , client;


    if (pthread_mutex_init(&lock, NULL) != 0)
    {
	    printf("\n mutex init failed\n");
	    return 1;
    }

    //Create socket
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (socket_desc == -1)
    {
	    printf("Could not create socket");
    }
    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( 8083 );

    //Bind
    if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
    {
	    //print the error message
	    perror("bind failed. Error");
	    return 1;
    }
    puts("bind done");
    puts("Socket created");

    //Listen
    listen(socket_desc , 10);

    //Accept and incoming connection
    puts("Waiting for incoming connections...");
    c = sizeof(struct sockaddr_in);
    pthread_t thread_id;



    while( (client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c)) )
    {
	    puts("Connection accepted");

	    if( pthread_create( &thread_id , NULL ,  connection_handler , (void*) &client_sock) < 0)
	    {
		    perror("could not create thread");
		    return 1;
	    }
	    //Now join the thread , so that we dont terminate before the thread
	    //pthread_join( thread_id , NULL);
	    puts("Handler assigned");
    }

    if (client_sock < 0)
    {
	    perror("accept failed");
	    return 1;
    }
    fflush(stdout);
    pthread_mutex_destroy(&lock); 
    return 0;

}


int main(int argc , char *argv[])
{

	pid_t grabber_pid;
	int status;
	char c;

	pipe(server_grabber_pipe);
	pipe(server_transmitter_pipe);
	pipe(grabber_transmitter_pipe);

	if(argc != 3)
	{
		fprintf(stderr, "Error: Arguments needed: <ImageWidth> <ImageHeight>\n");
		exit(1);
	}
	char * width = argv[1];
	char * height = argv[2];


	if((grabber_pid = fork())!=0)
	{
		pid_t rfTransmitter_pid;
		if((rfTransmitter_pid = fork())!=0)
		{
			close(server_grabber_pipe[0]);
			close(server_transmitter_pipe[0]);
			close(grabber_transmitter_pipe[0]);
			close(grabber_transmitter_pipe[1]);

			runServer();

			wait(&status);
			wait(&status);
			printf("Mis hijos han terminado... Adiós\n");
			return 0;
		}
		else
		{
			//THE RF TRANSMITTER PROCESS
			close(server_transmitter_pipe[1]);
			close(server_grabber_pipe[0]);
			close(server_grabber_pipe[1]);
			close(grabber_transmitter_pipe[1]);

			close(0);
			dup(grabber_transmitter_pipe[0]);
	
			fprintf(stderr, "Soy transmitter, la lectura con server esta en %d\n", server_transmitter_pipe[0]);
			fflush(stderr);


			execlp("./bin/TX_dynamic", "TX_dynamic", "imagen", "100", "150", "0","0",NULL);

			fprintf(stderr, "Ha pasado algun problema al ejecutar el transmitter\n");
			fflush(stderr);

			return 1;
		}

	}
	else
	{
		//THIS PROCESS CAPTURES THE VIDEO FRAMES, ENCODES THEM AND SENDS THEM TO THE TRANSMITTER
		close(server_grabber_pipe[1]);
		close(server_transmitter_pipe[0]);
		close(server_transmitter_pipe[1]);
		close(grabber_transmitter_pipe[0]);
		close(1);
		dup(grabber_transmitter_pipe[1]);

		fprintf(stderr, "Soy grabber, la lectura con server esta en %d\n", server_grabber_pipe[0]);
		fflush(stderr);

		execlp("./bin/grabberDebter_dynamic", "grabberDebter_dynamic", "-I", "imagen","-W",width, "-H",height, NULL);

		fprintf(stderr, "Ha pasado algun problema al ejecutar el grabber\n");
		fflush(stderr);

		return 1;		
	}
}
