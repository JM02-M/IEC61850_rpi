/*
 * goose_sub_gpio.c
 */

#include "goose_receiver.h"
#include "goose_subscriber.h"
#include "hal_thread.h"
#include "linked_list.h"
#include <wiringPi.h> // Incluir la biblioteca WiringPi para manipulación de GPIO

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>

static int running = 1;

static void sigint_handler(int signalId) {
    running = 0;
}

static void gooseListener(GooseSubscriber subscriber, void* parameter) {
    printf("GOOSE event:\n");
    printf("  stNum: %u sqNum: %u\n", GooseSubscriber_getStNum(subscriber),
           GooseSubscriber_getSqNum(subscriber));
    printf("  timeToLive: %u\n", GooseSubscriber_getTimeAllowedToLive(subscriber));

    uint64_t timestamp = GooseSubscriber_getTimestamp(subscriber);
    printf("  timestamp: %u.%u\n", (uint32_t)(timestamp / 1000), (uint32_t)(timestamp % 1000));
    printf("  message is %s\n", GooseSubscriber_isValid(subscriber) ? "valid" : "INVALID");

    MmsValue* values = GooseSubscriber_getDataSetValues(subscriber);

    // Verificar que el conjunto de datos contiene valores esperados
    if (MmsValue_getType(values) == MMS_ARRAY && MmsValue_getArraySize(values) >= 2) {
        // Leer los estados de los pines uno y dos
        bool pin1State = MmsValue_getBoolean(MmsValue_getElement(values, 0));
        bool pin2State = MmsValue_getBoolean(MmsValue_getElement(values, 1));

        // Activar los pines tres y cuatro según los estados de los pines uno y dos
        digitalWrite(28, pin1State ? HIGH : LOW); // Activar o desactivar el pin 3 según el estado del pin 1
        digitalWrite(29, pin2State ? HIGH : LOW); // Activar o desactivar el pin 4 según el estado del pin 2

        printf("Estado del Pin 1: %s, Estado del Pin 2: %s\n", 
               pin1State ? "HIGH" : "LOW", 
               pin2State ? "HIGH" : "LOW");
    } else {
        printf("Error: no se recibieron suficientes valores en el conjunto de datos o el tipo no es el esperado.\n");
    }

    char buffer[1024];
    MmsValue_printToBuffer(values, buffer, 1024);
    printf("  allData: %s\n", buffer);
}

int main(int argc, char** argv) {
    // Inicializar la biblioteca WiringPi
    if (wiringPiSetup() == -1) {
        printf("Error al inicializar wiringPi.\n");
        return -1;
    }

    // Configurar los pines 3 y 4 como salida
    pinMode(28, OUTPUT);
    pinMode(29, OUTPUT);

    GooseReceiver receiver = GooseReceiver_create();

    if (argc > 1) {
        printf("Set interface id: %s\n", argv[1]);
        GooseReceiver_setInterfaceId(receiver, argv[1]);
    } else {
        printf("Using interface eth0\n");
        GooseReceiver_setInterfaceId(receiver, "eth0");
    }

    GooseSubscriber subscriber = GooseSubscriber_create("simpleIOGenericIO/LLN0$GO$gcbAnalogValues", NULL);

    uint8_t dstMac[6] = {0x01, 0x0c, 0xcd, 0x01, 0x00, 0x01};
    GooseSubscriber_setDstMac(subscriber, dstMac);
    GooseSubscriber_setAppId(subscriber, 1000);

    GooseSubscriber_setListener(subscriber, gooseListener, NULL);

    GooseReceiver_addSubscriber(receiver, subscriber);

    GooseReceiver_start(receiver);

    if (GooseReceiver_isRunning(receiver)) {
        signal(SIGINT, sigint_handler);

        while (running) {
            Thread_sleep(100);
            
        }
    } else {
        printf("Failed to start GOOSE subscriber. Reason can be that the Ethernet interface doesn't exist or root permission are required.\n");
    }

    GooseReceiver_stop(receiver);
    GooseReceiver_destroy(receiver);

    return 0;
}
