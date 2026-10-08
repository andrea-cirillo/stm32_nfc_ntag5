/**
 ******************************************************************************
 * @file    ndef.h
 * @brief   NDEF parser for NFC Forum Type 5 Tags (NTAG 5 user memory).
 *
 *  Layout: Capability Container (4 or 8 bytes) followed by TLVs.
 *   CC[0] = E1h (1-byte addressing) / E2h (2-byte), CC[1] = version+access,
 *   CC[2] = MLEN (data area size / 8; 0 -> 8-byte extended CC),
 *   CC[3] = features. TLV: 00h NULL, 03h NDEF, FEh terminator.
 *  No dynamic allocation: the parser works on the buffer read over I2C.
 ******************************************************************************
 */
#ifndef NDEF_H
#define NDEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
  NDEF_OK = 0,
  NDEF_ERR_NO_CC,          /* no E1h/E2h magic: tag not NDEF formatted    */
  NDEF_ERR_NO_MESSAGE,     /* valid CC but no (or empty) NDEF TLV           */
  NDEF_ERR_TRUNCATED,      /* message longer than the buffer read           */
  NDEF_ERR_MALFORMED,
} ndef_result_t;

typedef struct {
  uint8_t  cc[8];
  uint8_t  cc_len;         /* 4 or 8                                         */
  uint8_t  version_major;
  uint8_t  version_minor;
  uint8_t  read_access;    /* 0 = free                                       */
  uint8_t  write_access;   /* 0 = free                                       */
  uint32_t data_area_size; /* bytes                                          */
  size_t   msg_offset;     /* offset of the NDEF message in the buffer       */
  size_t   msg_len;        /* length declared by the TLV                     */
} ndef_t5t_info_t;

typedef enum {
  NDEF_TNF_EMPTY = 0, NDEF_TNF_WELL_KNOWN = 1, NDEF_TNF_MIME = 2,
  NDEF_TNF_URI = 3, NDEF_TNF_EXTERNAL = 4, NDEF_TNF_UNKNOWN = 5,
  NDEF_TNF_UNCHANGED = 6, NDEF_TNF_RESERVED = 7,
} ndef_tnf_t;

typedef struct {
  ndef_tnf_t     tnf;
  bool           mb, me, chunked;
  const uint8_t *type;     size_t type_len;
  const uint8_t *id;       size_t id_len;
  const uint8_t *payload;  size_t payload_len;
} ndef_record_t;

typedef struct {
  const uint8_t *p;
  size_t         left;
  bool           done;
} ndef_iter_t;

/** Parses the CC and looks for the NDEF TLV in `mem` (user memory content). */
ndef_result_t ndef_t5t_parse(const uint8_t *mem, size_t len, ndef_t5t_info_t *info);

/** Iterates over the records of an NDEF message. */
void ndef_iter_init(ndef_iter_t *it, const uint8_t *msg, size_t len);
bool ndef_iter_next(ndef_iter_t *it, ndef_record_t *rec, ndef_result_t *err);

/** URI prefix for the "U" record identifier code (NFC Forum URI RTD). */
const char *ndef_uri_prefix(uint8_t code);

/** TNF name. */
const char *ndef_tnf_name(ndef_tnf_t tnf);

#ifdef __cplusplus
}
#endif

#endif /* NDEF_H */
