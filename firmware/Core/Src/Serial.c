/*
 * Serial.c
 *
 *  Created on: Mar 14, 2024
 *      Author: ihsan
 */

#include "Serial.h"

// Fonction pour transmettre des données sur le port série
uint8_t SerialTransmit(char *pData, uint16_t Size) {
    while (Size--) {
        // Attente de l'envoi du caractère précédent
        while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
        // Transmission du caractère
        LL_USART_TransmitData8(USART2, *pData++);
    }
    // Attente de la fin de la transmission
    while (!LL_USART_IsActiveFlag_TC(USART2)) {}
    return 0; // Retourne 0 pour indiquer que la transmission a réussi
}

// Fonction pour recevoir un caractère sur le port série
char SerialReceiveChar(void) {
    // Attente de la réception d'un caractère
    while (!LL_USART_IsActiveFlag_RXNE(USART2)) {}
    // Lecture du caractère reçu
    return (char)LL_USART_ReceiveData8(USART2);
}
