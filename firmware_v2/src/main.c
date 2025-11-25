//TEST PROGRAM   https://community.st.com/t5/stm32cubemx-mcus/how-to-fix-the-problem-quot-invalid-storage-class-for-function/td-p/262682




#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <lvgl_input_device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_interface.h>
#include <zephyr/sd/sd.h>
#include <zephyr/sd/sdmmc.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(threads);

// Zephyr 3.7.0 uses different disk naming convention
#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT "/SD:"

// Card detect configuration
#define CARD_DETECT_NODE DT_GPIO_CTLR(DT_NODELABEL(sdmmc1), cd_gpios)
#if DT_NODE_HAS_PROP(DT_NODELABEL(sdmmc1), cd_gpios)
static const struct gpio_dt_spec card_detect = GPIO_DT_SPEC_GET(DT_NODELABEL(sdmmc1), cd_gpios);
#define HAS_CARD_DETECT 1
#else
#define HAS_CARD_DETECT 0
#endif


// File system mount point
static struct fs_mount_t fs_mnt = {
    .type = FS_FATFS,
    .mnt_point = DISK_MOUNT_PT,
};

static bool sd_card_mounted = false;
static const struct device *sd_dev;


// Check card presence
static bool is_card_present(void)
{
#if HAS_CARD_DETECT
    if (!device_is_ready(card_detect.port)) {
        return true;  // Assume present if detect not available
    }
    return !gpio_pin_get_dt(&card_detect);  // Usually active low
#else
    return true;  // Always assume present if no detect pin
#endif
}

// Initialize card detect GPIO
static int init_card_detect(void)
{
#if HAS_CARD_DETECT
    if (!device_is_ready(card_detect.port)) {
        LOG_ERR("Card detect GPIO not ready");
        return -ENODEV;
    }
    
    int ret = gpio_pin_configure_dt(&card_detect, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure card detect pin: %d", ret);
        return ret;
    }
    
    LOG_INF("Card detect initialized on %s pin %d", 
            card_detect.port->name, card_detect.pin);
#endif
    return 0;
}

// Modern SD card initialization for Zephyr 3.7.0
static int setup_sd_card(void)
{
    int ret;
    
    // Get SD device
    sd_dev = DEVICE_DT_GET(DT_NODELABEL(sdmmc1));
    if (!device_is_ready(sd_dev)) {
        LOG_ERR("SD device not ready");
        return -ENODEV;
    }
    
    // Power on SD card
    // ret = sd_power_on();
    // if (ret < 0) {
    //     LOG_WRN("SD power control failed: %d", ret);
    // }
    
    // Check card presence
    if (!is_card_present()) {
        LOG_WRN("SD card not detected");
        return -ENODEV;
    }
    
    // Initialize disk access
    ret = disk_access_init(DISK_DRIVE_NAME);
    if (ret != 0) {
        LOG_ERR("SD disk access init failed: %d", ret);
        return ret;
    }
    
    // Get disk info
    uint32_t sector_count, sector_size;
    ret = disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
    if (ret) {
        LOG_ERR("Failed to get sector count: %d", ret);
        return ret;
    }
    
    ret = disk_access_ioctl(DISK_DRIVE_NAME, DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
    if (ret) {
        LOG_ERR("Failed to get sector size: %d", ret);
        return ret;
    }
    
    uint64_t total_size = (uint64_t)sector_count * sector_size;
    LOG_INF("SD card detected: %u sectors, %u bytes/sector, %u MB total",
            sector_count, sector_size, (uint32_t)(total_size >> 20));
    
    // Mount file system
    ret = fs_mount(&fs_mnt);
    if (ret != 0) {
        LOG_ERR("Failed to mount file system: %d", ret);
        return ret;
    }
    
    LOG_INF("SD card mounted at %s", DISK_MOUNT_PT);
    sd_card_mounted = true;
    return 0;
}

// LVGL file system driver implementation
static void* lvgl_sd_open(lv_fs_drv_t* drv, const char* path, lv_fs_mode_t mode)
{
    if (!sd_card_mounted) {
        LOG_WRN("SD card not mounted");
        return NULL;
    }
    
    char full_path[256];
    snprintf(full_path, sizeof(full_path), "%s/%s", DISK_MOUNT_PT, path);
    
    struct fs_file_t* file = k_malloc(sizeof(struct fs_file_t));
    if (!file) {
        LOG_ERR("Failed to allocate file structure");
        return NULL;
    }
    
    int flags = 0;
    if (mode & LV_FS_MODE_RD) flags |= FS_O_READ;
    if (mode & LV_FS_MODE_WR) flags |= FS_O_WRITE | FS_O_CREATE;
    
    int ret = fs_open(file, full_path, flags);
    if (ret < 0) {
        LOG_ERR("Failed to open %s: %d", full_path, ret);
        k_free(file);
        return NULL;
    }
    
    LOG_DBG("Opened file: %s", full_path);
    return file;
}

static lv_fs_res_t lvgl_sd_close(lv_fs_drv_t* drv, void* file_p)
{
    struct fs_file_t* file = (struct fs_file_t*)file_p;
    fs_close(file);
    k_free(file);
    return LV_FS_RES_OK;
}

static lv_fs_res_t lvgl_sd_read(lv_fs_drv_t* drv, void* file_p, void* buf, 
                                uint32_t btr, uint32_t* br)
{
    struct fs_file_t* file = (struct fs_file_t*)file_p;
    ssize_t result = fs_read(file, buf, btr);
    if (result < 0) {
        LOG_ERR("File read error: %d", (int)result);
        return LV_FS_RES_UNKNOWN;
    }
    *br = result;
    return LV_FS_RES_OK;
}

static lv_fs_res_t lvgl_sd_write(lv_fs_drv_t* drv, void* file_p, const void* buf, 
                                 uint32_t btw, uint32_t* bw)
{
    struct fs_file_t* file = (struct fs_file_t*)file_p;
    ssize_t result = fs_write(file, buf, btw);
    if (result < 0) {
        LOG_ERR("File write error: %d", (int)result);
        return LV_FS_RES_UNKNOWN;
    }
    *bw = result;
    return LV_FS_RES_OK;
}

static lv_fs_res_t lvgl_sd_seek(lv_fs_drv_t* drv, void* file_p, uint32_t pos, 
                                lv_fs_whence_t whence)
{
    struct fs_file_t* file = (struct fs_file_t*)file_p;
    int zephyr_whence;
    
    switch (whence) {
        case LV_FS_SEEK_SET: zephyr_whence = FS_SEEK_SET; break;
        case LV_FS_SEEK_CUR: zephyr_whence = FS_SEEK_CUR; break;
        case LV_FS_SEEK_END: zephyr_whence = FS_SEEK_END; break;
        default: return LV_FS_RES_UNKNOWN;
    }
    
    int ret = fs_seek(file, pos, zephyr_whence);
    return (ret == 0) ? LV_FS_RES_OK : LV_FS_RES_UNKNOWN;
}

static lv_fs_res_t lvgl_sd_tell(lv_fs_drv_t* drv, void* file_p, uint32_t* pos_p)
{
    struct fs_file_t* file = (struct fs_file_t*)file_p;
    off_t pos = fs_tell(file);
    if (pos < 0) return LV_FS_RES_UNKNOWN;
    *pos_p = pos;
    return LV_FS_RES_OK;
}

// Register LVGL file system driver
static void setup_lvgl_fs_driver(void)
{
    static lv_fs_drv_t fs_drv;
    lv_fs_drv_init(&fs_drv);
    
    fs_drv.letter = 'S';  // Access SD files with "S:" prefix
    fs_drv.open_cb = lvgl_sd_open;
    fs_drv.close_cb = lvgl_sd_close;
    fs_drv.read_cb = lvgl_sd_read;
    fs_drv.write_cb = lvgl_sd_write;
    fs_drv.seek_cb = lvgl_sd_seek;
    fs_drv.tell_cb = lvgl_sd_tell;
    
    lv_fs_drv_register(&fs_drv);
    LOG_INF("LVGL SD file system driver registered");
}


// Shell command for SD card info
static int cmd_sd_info(const struct shell *shell, size_t argc, char **argv)
{
    if (!sd_card_mounted) {
        printk(shell, "SD card not mounted");
        return -ENODEV;
    }
    
    struct fs_statvfs stats;
    int ret = fs_statvfs(DISK_MOUNT_PT, &stats);
    if (ret == 0) {
        uint64_t total_space = (uint64_t)stats.f_frsize * stats.f_blocks;
        uint64_t free_space = (uint64_t)stats.f_frsize ;
        
        printk(shell, "SD Card Information:");
        printk(shell, "  Total space: %u MB", (uint32_t)(total_space >> 20));
        printk(shell, "  Free space:  %u MB", (uint32_t)(free_space >> 20));
        printk(shell, "  Used space:  %u MB", 
                    (uint32_t)((total_space - free_space) >> 20));
    }
    
    return 0;
}

//SHELL_CMD_REGISTER(sd_info, NULL, "Show SD card information", cmd_sd_info);

// Performance benchmark
static void benchmark_sd_card(void)
{
    if (!sd_card_mounted) {
        LOG_WRN("SD card not mounted, skipping benchmark");
        return;
    }
    
    const size_t test_size = 64 * 1024;  // 64KB test
    uint8_t* buffer = k_malloc(test_size);
    if (!buffer) {
        LOG_ERR("Failed to allocate benchmark buffer");
        return;
    }
    
    // Fill with test data
    for (size_t i = 0; i < test_size; i++) {
        buffer[i] = i & 0xFF;
    }
    
    char test_file[256];
    snprintf(test_file, sizeof(test_file), "%s/benchmark.dat", DISK_MOUNT_PT);
    
    // Write test
    struct fs_file_t file;
    uint32_t start_time = k_uptime_get_32();
    
    int ret = fs_open(&file, test_file, FS_O_CREATE | FS_O_WRITE);
    if (ret == 0) {
        ssize_t written = fs_write(&file, buffer, test_size);
        fs_close(&file);
        
        uint32_t write_time = k_uptime_get_32() - start_time;
        if (written == test_size && write_time > 0) {
            uint32_t write_speed = (test_size * 1000) / (write_time * 1024);
            LOG_INF("Write speed: %u KB/s (%u bytes in %u ms)",
                    write_speed, test_size, write_time);
        }
    }
    
    // Read test
    start_time = k_uptime_get_32();
    ret = fs_open(&file, test_file, FS_O_READ);
    if (ret == 0) {
        ssize_t read_bytes = fs_read(&file, buffer, test_size);
        fs_close(&file);
        
        uint32_t read_time = k_uptime_get_32() - start_time;
        if (read_bytes == test_size && read_time > 0) {
            uint32_t read_speed = (test_size * 1000) / (read_time * 1024);
            LOG_INF("Read speed: %u KB/s (%u bytes in %u ms)",
                    read_speed, test_size, read_time);
        }
    }
    
    fs_unlink(test_file);  // Clean up
    k_free(buffer);
}


void sdCard_thread(void*a, void*b, void*c){
	k_sleep(K_MSEC(1000));
 LOG_INF("Starting Zephyr 3.7.0 LVGL with STM32 SDMMC");
    
    // Initialize display
    // const struct device* display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    // if (!device_is_ready(display_dev)) {
    //     LOG_ERR("Display device not ready");
    //     return -ENODEV;
    // }
    
    // Initialize card detect
    init_card_detect();
    
    // Initialize SD card
    int sd_result = setup_sd_card();
    if (sd_result == 0) {
        LOG_INF("SD card initialized successfully");
        setup_lvgl_fs_driver();
        //benchmark_sd_card();
    } else {
        LOG_ERR("SD card initialization failed: %d", sd_result);
    }
    
    
    LOG_INF("Entering main loop");
    
    while (1) {
        // Handle card insertion/removal
        static bool last_card_state = false;
        bool current_card_state = is_card_present();
        
        if (current_card_state != last_card_state) {
            if (current_card_state && !sd_card_mounted) {
                LOG_INF("Card inserted, mounting...");
                if (setup_sd_card() == 0) {
                    printk("Card setup ok.");
                }
            } else if (!current_card_state && sd_card_mounted) {
                LOG_INF("Card removed");
                fs_unmount(&fs_mnt);
                sd_card_mounted = false;
                printk("card removed");
            }
            last_card_state = current_card_state;
        }
        
        lv_timer_handler();
        k_msleep(10);
    }
    
    return 0;
}