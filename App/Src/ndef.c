/**
 ******************************************************************************
 * @file    ndef.c
 * @brief   Type 5 NDEF parser (see ndef.h).
 ******************************************************************************
 */
#include "ndef.h"

#include <string.h>

#define TLV_NULL        0x00U
#define TLV_NDEF        0x03U
#define TLV_PROPRIETARY 0xFDU
#define TLV_TERMINATOR  0xFEU

ndef_result_t ndef_t5t_parse(const uint8_t *mem, size_t len, ndef_t5t_info_t *info)
{
  size_t off;

  if ((mem == NULL) || (info == NULL) || (len < 4U)) { return NDEF_ERR_NO_CC; }
  memset(info, 0, sizeof *info);

  if ((mem[0] != 0xE1U) && (mem[0] != 0xE2U)) { return NDEF_ERR_NO_CC; }

  info->version_major = (uint8_t)(mem[1] >> 6);
  info->version_minor = (uint8_t)((mem[1] >> 4) & 0x03U);
  info->read_access   = (uint8_t)((mem[1] >> 2) & 0x03U);
  info->write_access  = (uint8_t)(mem[1] & 0x03U);

  if (mem[2] != 0U) {
    info->cc_len = 4U;
    info->data_area_size = (uint32_t)mem[2] * 8U;
  } else {
    /* Extended CC: 16-bit MLEN in CC[6..7] */
    if (len < 8U) { return NDEF_ERR_TRUNCATED; }
    info->cc_len = 8U;
    info->data_area_size = (((uint32_t)mem[6] << 8) | mem[7]) * 8U;
  }
  memcpy(info->cc, mem, info->cc_len);

  /* TLV scan */
  off = info->cc_len;
  while (off < len) {
    uint8_t  t = mem[off++];
    uint32_t l;

    if (t == TLV_NULL)       { continue; }
    if (t == TLV_TERMINATOR) { return NDEF_ERR_NO_MESSAGE; }
    if (off >= len)          { return NDEF_ERR_TRUNCATED; }

    l = mem[off++];
    if (l == 0xFFU) {
      if (off + 2U > len) { return NDEF_ERR_TRUNCATED; }
      l = ((uint32_t)mem[off] << 8) | mem[off + 1U];
      off += 2U;
    }

    if (t == TLV_NDEF) {
      info->msg_offset = off;
      info->msg_len    = l;
      if (l == 0U)        { return NDEF_ERR_NO_MESSAGE; }
      if (off + l > len)  { return NDEF_ERR_TRUNCATED; }
      return NDEF_OK;
    }
    off += l;                         /* proprietary / unknown TLV */
  }
  return NDEF_ERR_NO_MESSAGE;
}

void ndef_iter_init(ndef_iter_t *it, const uint8_t *msg, size_t len)
{
  it->p = msg;
  it->left = len;
  it->done = (msg == NULL) || (len == 0U);
}

bool ndef_iter_next(ndef_iter_t *it, ndef_record_t *rec, ndef_result_t *err)
{
  const uint8_t *p;
  size_t left, need;
  uint8_t hdr;
  uint32_t plen;

  *err = NDEF_OK;
  if (it->done) { return false; }
  p = it->p;
  left = it->left;
  memset(rec, 0, sizeof *rec);

  if (left < 3U) { goto malformed; }
  hdr = p[0];
  rec->mb      = (hdr & 0x80U) != 0U;
  rec->me      = (hdr & 0x40U) != 0U;
  rec->chunked = (hdr & 0x20U) != 0U;
  rec->tnf     = (ndef_tnf_t)(hdr & 0x07U);
  rec->type_len = p[1];
  p += 2; left -= 2U;

  if (hdr & 0x10U) {                       /* SR: payload length 1 byte */
    plen = p[0];
    p += 1; left -= 1U;
  } else {
    if (left < 4U) { goto malformed; }
    plen = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
    p += 4; left -= 4U;
  }
  rec->payload_len = plen;

  if (hdr & 0x08U) {                       /* IL: ID length present */
    if (left < 1U) { goto malformed; }
    rec->id_len = p[0];
    p += 1; left -= 1U;
  }

  need = rec->type_len + rec->id_len;
  if ((need > left) || (plen > left - need)) { goto malformed; }

  rec->type    = p;             p += rec->type_len;
  rec->id      = p;             p += rec->id_len;
  rec->payload = p;             p += plen;
  left -= need + plen;

  it->p = p;
  it->left = left;
  it->done = rec->me || (left == 0U);
  return true;

malformed:
  it->done = true;
  *err = NDEF_ERR_MALFORMED;
  return false;
}

const char *ndef_uri_prefix(uint8_t code)
{
  static const char *const tbl[] = {
    "", "http://www.", "https://www.", "http://", "https://", "tel:", "mailto:",
    "ftp://anonymous:anonymous@", "ftp://ftp.", "ftps://", "sftp://", "smb://",
    "nfs://", "ftp://", "dav://", "news:", "telnet://", "imap:", "rtsp://",
    "urn:", "pop:", "sip:", "sips:", "tftp:", "btspp://", "btl2cap://",
    "btgoep://", "tcpobex://", "irdaobex://", "file://", "urn:epc:id:",
    "urn:epc:tag:", "urn:epc:pat:", "urn:epc:raw:", "urn:epc:", "urn:nfc:",
  };
  return (code < (sizeof tbl / sizeof tbl[0])) ? tbl[code] : "";
}

const char *ndef_tnf_name(ndef_tnf_t tnf)
{
  switch (tnf) {
    case NDEF_TNF_EMPTY:      return "Empty";
    case NDEF_TNF_WELL_KNOWN: return "NFC Well-Known";
    case NDEF_TNF_MIME:       return "MIME";
    case NDEF_TNF_URI:        return "Absolute URI";
    case NDEF_TNF_EXTERNAL:   return "External";
    case NDEF_TNF_UNKNOWN:    return "Unknown";
    case NDEF_TNF_UNCHANGED:  return "Unchanged";
    default:                  return "Reserved";
  }
}
