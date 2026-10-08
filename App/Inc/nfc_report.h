/**
 ******************************************************************************
 * @file    nfc_report.h
 * @brief   Serial report of NFC events and tag content.
 ******************************************************************************
 */
#ifndef NFC_REPORT_H
#define NFC_REPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nfc_app.h"

/** Prints a human-readable report of the event on the serial port. */
void nfc_report_print(const nfc_event_t *evt);

#ifdef __cplusplus
}
#endif

#endif /* NFC_REPORT_H */
