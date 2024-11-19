/*
 * goose_publisher_example.c
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "mms_value.h"
#include "goose_publisher.h"
#include "hal_thread.h"
#include <wiringPi.h>
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>

// Configuración del puerto serie
#define TERMINAL    "/dev/ttyACM0"

// Función para configurar los parámetros del puerto serie
int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

// Función para leer datos del puerto serie y convertirlos a flotante
float read_serial_data(int fd)
{
    unsigned char buf[80];
    int rdlen;
    rdlen = read(fd, buf, sizeof(buf) - 1);
    if (rdlen > 0) {
        buf[rdlen] = '\0';  // Terminar la cadena de caracteres

        // Convertir el dato leído a flotante
        float received_float = atof((char*)buf);

        if (received_float != 0.0f) {
            return received_float;
        }
    }
    return 0.0f;
}

int main(int argc, char **argv)
{
    // Inicializar la biblioteca WiringPi
    if (wiringPiSetup() == -1) {
        printf("Error al inicializar wiringPi.\n");
        return -1;
    }

    // Configurar los pines 25 y 21 como entrada
    pinMode(25, INPUT);
    pinMode(21, INPUT);

    char *interface;

    if (argc > 1)
        interface = argv[1];
    else
        interface = "eth0";

    printf("Using interface %s\n", interface);

    // Parámetros de comunicación GOOSE
    CommParameters gooseCommParameters;
    gooseCommParameters.appId = 1000;
    gooseCommParameters.dstAddress[0] = 0x01;
    gooseCommParameters.dstAddress[1] = 0x0c;
    gooseCommParameters.dstAddress[2] = 0xcd;
    gooseCommParameters.dstAddress[3] = 0x01;
    gooseCommParameters.dstAddress[4] = 0x00;
    gooseCommParameters.dstAddress[5] = 0x01;
    gooseCommParameters.vlanId = 0;
    gooseCommParameters.vlanPriority = 4;

    // Crear el publicador GOOSE
    GoosePublisher publisher = GoosePublisher_create(&gooseCommParameters, interface);

    if (publisher) {
        GoosePublisher_setGoCbRef(publisher, "simpleIOGenericIO/LLN0$GO$gcbAnalogValues");
        GoosePublisher_setConfRev(publisher, 1);
        GoosePublisher_setDataSetRef(publisher, "simpleIOGenericIO/LLN0$AnalogValues");
        GoosePublisher_setTimeAllowedToLive(publisher, 500);

        // Abrir el puerto serie
        int fd = open(TERMINAL, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) {
            printf("Error al abrir el puerto serie: %s\n", strerror(errno));
            return -1;
        }
        set_interface_attribs(fd, B115200);

        // Loop de lectura y publicación de datos
        while (true) {
            Thread_sleep(10);

            // Crear lista de valores para publicar
            LinkedList dataSetValues = LinkedList_create();

            // Leer valores de los pines
            LinkedList_add(dataSetValues, MmsValue_newBoolean(digitalRead(25)));
            LinkedList_add(dataSetValues, MmsValue_newBoolean(digitalRead(21)));

            // Leer el valor flotante del puerto serie
            float serial_data = read_serial_data(fd);
            if (serial_data != 0.0f) {
                printf("%.5f\n", serial_data);
                LinkedList_add(dataSetValues, MmsValue_newFloat(serial_data));
            }

            // Publicar los datos
            GoosePublisher_publish(publisher, dataSetValues);

            // Limpiar los datos del conjunto
            LinkedList_destroyDeep(dataSetValues, (LinkedListValueDeleteFunction) MmsValue_delete); 
        }

        // Cerrar el puerto serie y destruir el publicador
        close(fd);
        GoosePublisher_destroy(publisher);
    } else {
        printf("Failed to create GOOSE publisher. Reason can be that the Ethernet interface doesn't exist or root permission are required.\n");
    }

    return 0;
}
