#include <lwip/err.h>
#include <lwip/dns.h>

#define DNS_FALLBACK_SERVER_INDEX (DNS_MAX_SERVERS - 1)

void dns_clear_servers(bool keep_fallback)
{
  u8_t numdns = 0; 
  
  for (numdns = 0; numdns < DNS_MAX_SERVERS; numdns ++) {
    if (keep_fallback && numdns == DNS_FALLBACK_SERVER_INDEX) {
      continue;
    }

    dns_setserver(numdns, NULL);
  }
}


