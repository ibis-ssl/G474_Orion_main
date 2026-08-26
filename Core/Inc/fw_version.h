/* Mainアプリに埋め込む開発用build IDの最小形式を定義する。 */
/* アプリケーションに埋め込む開発用FW識別子の共通形式を定義する。 */
#ifndef INC_FW_VERSION_H_
#define INC_FW_VERSION_H_

#include <stdint.h>

#define FW_VERSION_MAGIC UINT32_C(0x52565746) /* "FWVR" little endian */

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t build_id;
} fw_version_t;

extern const fw_version_t g_fw_version;

#endif
