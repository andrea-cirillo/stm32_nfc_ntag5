/**
 ******************************************************************************
 * @file    nfc_report.c
 * @brief   Serial report of NFC events (see nfc_report.h).
 *
 *  Sample output:
 *
 *   --------------------------------------------------
 *   [RF ] #3 Tag READ  (9 RF commands in 62 ms, I2C+DMA acquisition 3900 us)
 *   [TAG]   #1 Well-Known "T"  lang=en  text: "hello"
 *   --------------------------------------------------
 *   [RF ] #4 Tag WRITE  (21 RF commands in 180 ms, I2C+DMA acquisition 4400 us)
 *   [TAG]   New content:
 *   [TAG]   #1 Well-Known "U"  URI: https://www.nxp.com
 ******************************************************************************
 */
#include "nfc_report.h"
#include "ndef.h"
#include "ntag5.h"
#include "app_log.h"

#include <stdio.h>
#include <string.h>

#define TXT_MAX   96U   /* maximum characters printed for text/URI/payload */

/* Copies a printable string into `out` (UTF-8 bytes are passed through). */
static const char *printable(const uint8_t *p, size_t n, char out[TXT_MAX + 4U])
{
  size_t i, k = 0;
  for (i = 0; (i < n) && (k < TXT_MAX); i++) {
    uint8_t c = p[i];
    out[k++] = ((c >= 0x20U) && (c != 0x7FU)) ? (char)c : '.';
  }
  if (i < n) { out[k++] = '.'; out[k++] = '.'; out[k++] = '.'; }
  out[k] = '\0';
  return out;
}

static bool is_text(const uint8_t *p, size_t n)
{
  for (size_t i = 0; i < n; i++) {
    if ((p[i] < 0x20U) && (p[i] != '\r') && (p[i] != '\n') && (p[i] != '\t')) { return false; }
  }
  return n > 0U;
}

static const char *access_str(uint8_t a)
{
  return (a == 0U) ? "free" : (a == 2U) ? "proprietary" : (a == 3U) ? "denied" : "RFU";
}

static void print_record(unsigned idx, const ndef_record_t *r)
{
  char type[TXT_MAX + 4U], txt[TXT_MAX + 4U];

  (void)printable(r->type, r->type_len, type);

  if ((r->tnf == NDEF_TNF_WELL_KNOWN) && (r->type_len == 1U) && (r->type[0] == 'U') &&
      (r->payload_len >= 1U)) {
    LOGI("[TAG]   #%u Well-Known \"U\"  URI: %s%s", idx, ndef_uri_prefix(r->payload[0]),
         printable(&r->payload[1], r->payload_len - 1U, txt));
    return;
  }

  if ((r->tnf == NDEF_TNF_WELL_KNOWN) && (r->type_len == 1U) && (r->type[0] == 'T') &&
      (r->payload_len >= 1U)) {
    uint8_t st   = r->payload[0];
    size_t  lang = st & 0x3FU;
    char    lng[TXT_MAX + 4U];
    if (1U + lang <= r->payload_len) {
      (void)printable(&r->payload[1], lang, lng);
      LOGI("[TAG]   #%u Well-Known \"T\"  lang=%s%s  text: \"%s\"", idx, lng,
           (st & 0x80U) ? " (UTF-16)" : "",
           printable(&r->payload[1U + lang], r->payload_len - 1U - lang, txt));
      return;
    }
  }

  LOGI("[TAG]   #%u %s \"%s\"  payload %u B%s", idx, ndef_tnf_name(r->tnf), type,
       (unsigned)r->payload_len, r->chunked ? " (chunk)" : "");
  if (r->payload_len > 0U) {
    if (is_text(r->payload, r->payload_len)) {
      LOGI("[TAG]       \"%s\"", printable(r->payload, r->payload_len, txt));
    } else {
      app_log_hexdump(r->payload, (r->payload_len > 64U) ? 64U : r->payload_len);
    }
  }
}

/* Decodes and prints the NDEF content held in e->data */
static void print_ndef(const nfc_event_t *e)
{
  ndef_t5t_info_t info;
  ndef_result_t res = ndef_t5t_parse(e->data, e->len, &info);

  if (res == NDEF_ERR_NO_CC) {
    LOGW("[TAG]   No NDEF Capability Container (tag not formatted). First bytes:");
    app_log_hexdump(e->data, (e->len > 32U) ? 32U : e->len);
    return;
  }
  if (res == NDEF_ERR_NO_MESSAGE) {
    LOGI("[TAG]   NDEF formatted tag, no message (empty)");
    return;
  }
  if (res == NDEF_ERR_TRUNCATED) {
    LOGW("[TAG]   NDEF message of %u B exceeds the %u B acquired: increase "
         "NFC_APP_TAG_READ_SIZE", (unsigned)info.msg_len, (unsigned)e->len);
    return;
  }
  if (res != NDEF_OK) {
    LOGW("[TAG]   Invalid TLV");
    return;
  }

  ndef_iter_t it;
  ndef_record_t rec;
  ndef_result_t err = NDEF_OK;
  unsigned n = 0;
  ndef_iter_init(&it, &e->data[info.msg_offset], info.msg_len);
  while (ndef_iter_next(&it, &rec, &err)) {
    print_record(++n, &rec);
  }
  if (err != NDEF_OK) {
    LOGW("[TAG]   Malformed record %u", n + 1U);
  }
}

static void print_cc(const nfc_event_t *e)
{
  ndef_t5t_info_t info;
  if (ndef_t5t_parse(e->data, e->len, &info) != NDEF_ERR_NO_CC) {
    LOGI("[TAG]   CC %02X %02X %02X %02X -> Type 5 v%u.%u, data area %lu B, R:%s W:%s",
         info.cc[0], info.cc[1], info.cc[2], info.cc[3], info.version_major,
         info.version_minor, (unsigned long)info.data_area_size,
         access_str(info.read_access), access_str(info.write_access));
  }
}

static void print_read_error(const nfc_event_t *e)
{
  LOGE("[TAG]   Memory acquisition failed: %s", ntag5_strerror((ntag5_status_t)e->read_err));
  LOGE("[TAG]   NTAG5 STATUS0=0x%02X STATUS1=0x%02X, HAL I2C error=0x%08lX state=0x%02lX",
       e->status0, e->status1, (unsigned long)e->hal_err, (unsigned long)e->hal_state);
}

static void print_header(const char *what, const nfc_event_t *e)
{
  app_log_raw("--------------------------------------------------\r\n");
  LOGI("[RF ] #%lu %s  (%lu RF commands in %lu ms, I2C+DMA acquisition %lu us)",
       (unsigned long)e->seq, what, (unsigned long)e->rf_cmds, (unsigned long)e->rf_ms,
       (unsigned long)e->read_us);
}

void nfc_report_print(const nfc_event_t *e)
{
  switch (e->type) {
    case NFC_EVT_READY:
      if (e->ed_name == NULL) {
        LOGI("[NFC] NTAG5 ready, ED pin excluded: detection through the RF field.");
      } else if (e->ed_found) {
        LOGI("[NFC] ED self-test on %s: OK (release %s)", e->ed_name,
             e->ed_clear_ok ? "OK" : "FAILED");
        if (e->ed_clear_ok) {
          LOGI("[NFC]     working ED_INTR_CLEAR register: %04Xh", (unsigned)e->ed_clear_reg);
        }
        LOGI("[NFC] NTAG5 ready, ED = start of RF command. Bring an NFC reader close...");
      } else {
        LOGW("[NFC] ED self-test on %s: the pin does not see ED (level=%u). Check the "
             "wiring; using the RF field only.", e->ed_name, (unsigned)e->ed_level);
        LOGI("[NFC] NTAG5 ready. Bring an NFC reader close...");
      }
      break;

    case NFC_EVT_TAG_CONTENT:
      if (e->read_err != (int)NTAG5_OK) {
        print_read_error(e);
        break;
      }
      LOGI("[TAG] Current tag content (%u B, I2C+DMA %lu us, DMA ok=%lu IT fallbacks=%lu):",
           (unsigned)e->len, (unsigned long)e->read_us, (unsigned long)e->dma_ok,
           (unsigned long)e->dma_fallback_it);
      print_cc(e);
      print_ndef(e);
      break;

    case NFC_EVT_TAG_READ:
      print_header("Tag READ", e);
      if (e->read_err != (int)NTAG5_OK) {
        print_read_error(e);
        break;
      }
      print_ndef(e);
      break;

    case NFC_EVT_TAG_WRITE:
      print_header("Tag WRITE", e);
      LOGI("[TAG]   New content:");
      print_ndef(e);
      break;

    case NFC_EVT_SRAM_WRITE: {
      char txt[TXT_MAX + 4U];
      print_header("SRAM WRITE (RF -> I2C data)", e);
      app_log_hexdump(e->data, e->len);
      size_t n = strnlen((const char *)e->data, e->len);
      if ((n > 0U) && is_text(e->data, n)) {
        LOGI("[SRAM]  text: \"%s\"", printable(e->data, n, txt));
      }
      break;
    }

    case NFC_EVT_SESSION_END:
      LOGI("[RF ] reader session ended: field present ~%lu ms, ED edges=%lu, "
           "STATUS1 or=0x%02X and=0x%02X", (unsigned long)e->rf_ms, (unsigned long)e->rf_cmds,
           e->s1_or, e->s1_and);
      break;

    case NFC_EVT_IGNORED:
      LOGI("[RF ] short transaction ignored (%lu ED edges in %lu ms, presence check)",
           (unsigned long)e->rf_cmds, (unsigned long)e->rf_ms);
      break;

    case NFC_EVT_ERROR:
    default:
      LOGE("[NFC] %s: %s", (e->what != NULL) ? e->what : "error",
           ntag5_strerror((ntag5_status_t)e->read_err));
      break;
  }
}
