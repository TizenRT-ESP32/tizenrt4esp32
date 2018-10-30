#include "esp_partition.h"
#include <assert.h>

const esp_partition_t *esp_ota_get_running_partition(void)
{
	// Return first instance of an app partition
	const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP,
									   ESP_PARTITION_SUBTYPE_ANY,
									   NULL);
	ASSERT(partition != NULL);

	return partition;
}
