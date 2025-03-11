/*
 * This file is part of GB-Drone project (https://github.com/GBeetle/GB-Drone).
 * Copyright (c) 2022 GBeetle.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/errno.h>
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_partition.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "soc/spi_periph.h"
#include "file_system.h"
#include "error_handle.h"
#include "log_sys.h"
#include <errno.h>

#define MAX_FILE_PATH_LEN 256
// Maximum time to wait for the mutex in a logging statement.
//
// We don't expect this to happen in most cases, as contention is low. The most likely case is if a
// log function is called from an ISR (technically caller should use the ISR-friendly logging macros but
// possible they use the normal one instead and disable the log type by tag).
#define MAX_MUTEX_WAIT_MS 10
#define MAX_MUTEX_WAIT_TICKS ((MAX_MUTEX_WAIT_MS + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS)

// Handle of the wear levelling library instance
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

// Mount path for the partition
const char *base_path = "/storage";

const esp_partition_t* gb_add_partition(esp_flash_t* ext_flash, const char* partition_label)
{
    GB_DEBUGI(FS_TAG, "Adding external Flash as a partition, label=\"%s\", size=%d KB", partition_label, ext_flash->size / 1024);
    const esp_partition_t* fat_partition;
    ESP_ERROR_CHECK(esp_partition_register_external(ext_flash, 0, ext_flash->size, partition_label, ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, &fat_partition));
    return fat_partition;
}

void gb_list_data_partitions(void)
{
    GB_DEBUGI(FS_TAG, "Listing data partitions:");
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, NULL);

    for (; it != NULL; it = esp_partition_next(it)) {
        const esp_partition_t *part = esp_partition_get(it);
        GB_DEBUGI(FS_TAG, "- partition '%s', subtype %d, offset 0x%x, size %d kB",
        part->label, part->subtype, part->address, part->size / 1024);
    }

    esp_partition_iterator_release(it);
}

bool gb_mount_fatfs(const char* partition_label)
{
    GB_DEBUGI(FS_TAG, "Mounting FAT filesystem");
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = true,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(base_path, partition_label, &mount_config, &s_wl_handle);
    if (err != ESP_OK) {
        GB_DEBUGE(FS_TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return false;
    }
    return true;
}

void gb_get_fatfs_usage(size_t* out_total_bytes, size_t* out_free_bytes)
{
    FATFS *fs;
    DWORD free_clusters;
    int res = f_getfree("0:", &free_clusters, &fs);
    assert(res == FR_OK);
    size_t total_sectors = (fs->n_fatent - 2) * fs->csize;
    size_t free_sectors = free_clusters * fs->csize;

    // assuming the total size is < 4GiB, should be true for SPI Flash
    if (out_total_bytes != NULL) {
        *out_total_bytes = total_sectors * fs->ssize;
    }
    if (out_free_bytes != NULL) {
        *out_free_bytes = free_sectors * fs->ssize;
    }
}

GB_RESULT GB_FileSystem_Init(const char* partition_label)
{
    GB_RESULT res = GB_OK;

    // List the available partitions
    gb_list_data_partitions();

    // Initialize FAT FS in the partition
    if (!gb_mount_fatfs(partition_label)) {
        return GB_FS_MOUNT_FAIL;
    }

    // Print FAT FS size information
    size_t bytes_total, bytes_free;
    gb_get_fatfs_usage(&bytes_total, &bytes_free);
    GB_DEBUGI(FS_TAG, "FAT FS: %d kB total, %d kB free", bytes_total / 1024, bytes_free / 1024);

    return res;
}

GB_RESULT GB_FileSystem_Write(const char* file, const uint8_t *data, uint32_t len)
{
    GB_RESULT res = GB_OK;
    char full_path[MAX_FILE_PATH_LEN] = {0};
    FILE *f = NULL;

    strcpy(full_path, base_path);
    strcat(full_path, "/");
    strcat(full_path, file);

    f = fopen(full_path, "wb");
    if (f == NULL) {
        GB_DEBUGE(FS_TAG, "Failed to open file for writing");
        CHK_RES(GB_FS_RE_FOPEN_FAIL);
    }

    fwrite(data, len, 1, f);

#if FS_DEBUG
    struct stat info;
    if (stat(full_path, &info) < 0) {
        GB_DEBUGE(FS_TAG, "Failed to stat file: %s", strerror(errno));
        CHK_RES(GB_FS_FOPEN_FAIL);
    }

    GB_DEBUGI(
        FS_TAG,
        "File stats:\n"
        "\tFile size:                %ld bytes\n"
        "\tFile modification time:   %s",
        info.st_size,
        ctime(&info.st_mtime)
    );
#endif

error_exit:
    if (f) fclose(f);
    return res;
}

GB_RESULT GB_FileSystem_Read(const char* file, uint8_t *data, uint32_t len)
{
    GB_RESULT res = GB_OK;
    char full_path[MAX_FILE_PATH_LEN] = {0};
    FILE *f = NULL;
    int read_len = 0;

    strcpy(full_path, base_path);
    strcat(full_path, "/");
    strcat(full_path, file);

#if FS_DEBUG
    struct stat info;
    if (stat(full_path, &info) < 0) {
        GB_DEBUGE(FS_TAG, "Failed to stat file: %s", strerror(errno));
        CHK_RES(GB_FS_FOPEN_FAIL);
    }

    GB_DEBUGI(
        FS_TAG,
        "File stats:\n"
        "\tFile size:                %ld bytes\n"
        "\tFile modification time:   %s",
        info.st_size,
        ctime(&info.st_mtime)
    );
#endif

    f = fopen(full_path, "rb");
    if (f == NULL) {
        GB_DEBUGE(FS_TAG, "Failed to open file for reading");
        CHK_RES(GB_FS_RE_FOPEN_FAIL);
    }

    read_len = fread(data, len, 1, f);
    if (read_len != 1) {
        GB_DEBUGE(FS_TAG, "Failed to read file");
        CHK_RES(GB_FS_READ_FAIL);
    }

error_exit:
    if (f) fclose(f);
    return res;
}

GB_RESULT GB_FileSystem_ListDir()
{
    GB_RESULT res = GB_OK;

    GB_DEBUGI(FS_TAG, "Listing files in %s:", base_path);

    DIR *dir = opendir(base_path);
    if (!dir) {
        GB_DEBUGE(FS_TAG, "Failed to open directory: %s", strerror(errno));
        CHK_RES(GB_FS_FOPEN_FAIL);
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        GB_DEBUGI(FS_TAG,
            "    %s: %s\n",
            (entry->d_type == DT_DIR)
                ? "directory"
                : "file     ",
            entry->d_name
        );
    }

error_exit:
    closedir(dir);
    return res;
}